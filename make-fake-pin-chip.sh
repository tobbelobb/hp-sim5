#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./make-fake-pin-chip.sh [path/to/config]
# Default config path:
CFG_PATH="${1:-examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg}"

# Placeholders in the config
GPIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"
ANALOG_PLACEHOLDER_PREFIX="REPLACE_ME_WITH_FAKE_ANALOG_PIN_"  # suffixed with 0..7

# Names for the simulated devices
GPIO_SIM_NAME="slideprinter-sim"   # configfs device name (folder under gpio-sim)
GPIO_BANK="gpio-bank0"
GPIO_LINES=64
IIO_DEV_NAME="slideprinter-iio-sim"

reexec_as_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root..."
    exec sudo --preserve-env=CFG_PATH,GPIO_PLACEHOLDER,ANALOG_PLACEHOLDER_PREFIX,GPIO_SIM_NAME,GPIO_BANK,GPIO_LINES,IIO_DEV_NAME bash "$0" "$CFG_PATH"
  fi
}
reexec_as_root

ensure_configfs() {
  local base="/sys/kernel/config"
  if ! grep -q " $base " /proc/mounts 2>/dev/null; then
    modprobe configfs 2>/dev/null || true
    mkdir -p "$base"
    mount -t configfs none "$base"
  fi
  # Some distros have /config -> /sys/kernel/config symlink; prefer it if present.
  if [[ -d /config ]]; then
    echo "/config"
  else
    echo "$base"
  fi
}

die() {
  echo "Error: $*" >&2
  exit 1
}

# --- Setup/cleanup GPIO simulator (gpio-sim) ---
create_gpio_sim() {
  local CONFIGFS_BASE="$1"
  modprobe gpio-sim || true
  local SIM_ROOT="$CONFIGFS_BASE/gpio-sim"
  local DEV_DIR="$SIM_ROOT/$GPIO_SIM_NAME"
  local BANK_DIR="$DEV_DIR/$GPIO_BANK"

  # Tear down any old instance we created before
  if [[ -d "$DEV_DIR" ]]; then
    if [[ -w "$DEV_DIR/live" ]]; then echo 0 > "$DEV_DIR/live" || true; fi
    if [[ -d "$BANK_DIR" ]]; then
      find "$BANK_DIR" -maxdepth 1 -type d -name 'line*' -exec rmdir {} + 2>/dev/null || true
      rmdir "$BANK_DIR" 2>/dev/null || true
    fi
    rmdir "$DEV_DIR" 2>/dev/null || true
  fi

  # Create fresh device + bank
  mkdir -p "$BANK_DIR"
  echo "$GPIO_LINES" > "$BANK_DIR/num_lines"
  echo 1 > "$DEV_DIR/live"

  # Read the bank's chip_name (e.g., "gpiochip3")
  [[ -r "$BANK_DIR/chip_name" ]] || die "chip_name attribute not found under $BANK_DIR"
  local CHIP_SYM
  CHIP_SYM="$(cat "$BANK_DIR/chip_name")"
  if [[ ! "$CHIP_SYM" =~ ^gpiochip([0-9]+)$ ]]; then
    die "unexpected chip_name format: '$CHIP_SYM'"
  fi
  local CHIP_NUM
  CHIP_NUM="${BASH_REMATCH[1]}"

  # Wait briefly for /dev/gpiochipN to appear (udev)
  for i in {1..40}; do
    [[ -e "/dev/$CHIP_SYM" ]] && break
    sleep 0.05
  done

  echo "$CHIP_NUM:$CHIP_SYM:$DEV_DIR"
}

# --- Setup/cleanup IIO dummy device for analog I/O ---
create_iio_dummy() {
  local CONFIGFS_BASE="$1"
  # Load the IIO dummy software device support
  modprobe iio_dummy 2>/dev/null || true
  modprobe iio_dummy_evgen 2>/dev/null || true

  local IIO_CFG_ROOT="$CONFIGFS_BASE/iio"
  local DUMMY_DIR="$IIO_CFG_ROOT/devices/dummy/$IIO_DEV_NAME"

  # Sanity check for configfs layout
  [[ -d "$IIO_CFG_ROOT/devices/dummy" ]] || die "IIO configfs not available at $IIO_CFG_ROOT. Is CONFIG_IIO_CONFIGFS enabled and iio_dummy module present?"

  # If our dummy already exists, remove it to force a clean state
  if [[ -d "$DUMMY_DIR" ]]; then
    rmdir "$DUMMY_DIR" 2>/dev/null || true
  fi

  # Create our dummy device (instantiates iio:deviceX)
  mkdir -p "$DUMMY_DIR"

  # Wait until an iio:deviceX shows up with our name and has in_voltage0_raw
  local tries=0
  local dev_path=""
  while (( tries < 60 )); do
    for p in /sys/bus/iio/devices/iio:device*; do
      [[ -e "$p/name" ]] || continue
      local n
      n="$(cat "$p/name" 2>/dev/null || true)"
      if [[ "$n" == "$IIO_DEV_NAME" && -e "$p/in_voltage0_raw" ]]; then
        dev_path="$p"
        break
      fi
    done
    [[ -n "$dev_path" ]] && break
    sleep 0.05
    ((tries++))
  done

  if [[ -z "$dev_path" ]]; then
    rmdir "$DUMMY_DIR" 2>/dev/null || true
    die "Failed to instantiate $IIO_DEV_NAME with in_voltage0_raw present."
  fi
  echo "$dev_path"
}

patch_cfg() {
  local cfg="$1" chip_num="$2"
  [[ -f "$cfg" ]] || die "config file not found: $cfg"
  cp -n "$cfg" "$cfg.bak" 2>/dev/null || true

  # sed portability: detect GNU sed
  if sed --version >/dev/null 2>&1; then
    sed -i "s/${GPIO_PLACEHOLDER}/${chip_num}/g" "$cfg"
    # Replace up to 8 analog placeholders with analog0..analog7
    for i in {0..7}; do
      sed -i "s/${ANALOG_PLACEHOLDER_PREFIX}${i}/analog${i}/g" "$cfg"
    done
  else
    sed -i '' "s/${GPIO_PLACEHOLDER}/${chip_num}/g" "$cfg"
    for i in {0..7}; do
      sed -i '' "s/${ANALOG_PLACEHOLDER_PREFIX}${i}/analog${i}/g" "$cfg"
    done
  fi
}

main() {
  local CONFIGFS_BASE
  CONFIGFS_BASE="$(ensure_configfs)"

  # 1) Create GPIO simulator
  IFS=":" read -r CHIP_NUM CHIP_SYM GPIO_DEV_DIR < <(create_gpio_sim "$CONFIGFS_BASE")

  # 2) Create IIO dummy device
  local IIO_DEV_PATH
  IIO_DEV_PATH="$(create_iio_dummy "$CONFIGFS_BASE")"

  # 3) Patch the config file with the discovered identifiers
  patch_cfg "$CFG_PATH" "$CHIP_NUM"

  echo "Created simulated GPIO chip: /dev/${CHIP_SYM} (${GPIO_LINES} lines)"
  echo "GPIO configfs: $GPIO_DEV_DIR  (set 'live' to 0 to tear down)"
  echo "Updated ${CFG_PATH}: ${GPIO_PLACEHOLDER} -> ${CHIP_NUM}"

  echo
  echo "Created IIO dummy device for analog I/O: $IIO_DEV_PATH (name: $IIO_DEV_NAME)"
  echo "Replaced any ${ANALOG_PLACEHOLDER_PREFIX}N placeholders with analogN (N=0..7) in ${CFG_PATH}"
  echo
  echo "How to read/write analog values (12-bit scale by default):"
  echo "- Read analog input 0:    cat $IIO_DEV_PATH/in_voltage0_raw"
  echo "- Read analog input 1:    cat $IIO_DEV_PATH/in_voltage1_raw  (if present)"
  echo "- Set analog output 0:    echo 2048 > $IIO_DEV_PATH/out_voltage0_raw"
  echo
  echo "Notes:"
  echo "- Klipper's Linux MCU reads analogN from an IIO device with in_voltageN_raw."
  echo "- If an existing IIO device is iio:device0, Klipper (after our patch) auto-detects a usable device."
  echo "  You can force a specific device via env vars: KLIPPER_IIO_DEVICE=/sys/bus/iio/devices/iio:deviceX or KLIPPER_IIO_NAME=$IIO_DEV_NAME"
  echo "- The IIO dummy typically exposes at least in_voltage0_raw and out_voltage0_raw."
  echo "- If your kernel's iio_dummy exposes additional inputs (in_voltage1_raw ..), they map to analog1..analog7."
  echo "- For heater outputs on Linux MCU, prefer pwmchipX/pwmY pins (hardware PWM)."
}

main "$@"
