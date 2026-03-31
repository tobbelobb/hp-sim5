#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./make-fake-pin-chip.sh [path/to/config]
# Default config path:
CFG_PATH="${1:-examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg}"

# Placeholders in the config
GPIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"

# Names for the simulated devices
GPIO_SIM_NAME="slideprinter-sim"   # configfs device name (folder under gpio-sim)
GPIO_BANK="gpio-bank0"
GPIO_LINES=64

reexec_as_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root..."
    exec sudo --preserve-env=CFG_PATH,GPIO_PLACEHOLDER,GPIO_SIM_NAME,GPIO_BANK,GPIO_LINES bash "$0" "$CFG_PATH"
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

  if [[ -e "/dev/$CHIP_SYM" ]]; then
    chmod 666 "/dev/$CHIP_SYM" || true
  fi

  echo "$CHIP_NUM:$CHIP_SYM:$DEV_DIR"
}

patch_cfg() {
  local cfg="$1" chip_num="$2"
  [[ -f "$cfg" ]] || die "config file not found: $cfg"
  cp -n "$cfg" "$cfg.bak" 2>/dev/null || true

  # sed portability: detect GNU sed
  if sed --version >/dev/null 2>&1; then
    sed -i "s/${GPIO_PLACEHOLDER}/${chip_num}/g" "$cfg"
  else
    sed -i '' "s/${GPIO_PLACEHOLDER}/${chip_num}/g" "$cfg"
  fi
}

main() {
  local CONFIGFS_BASE
  CONFIGFS_BASE="$(ensure_configfs)"

  # 1) Create GPIO simulator
  IFS=":" read -r CHIP_NUM CHIP_SYM GPIO_DEV_DIR < <(create_gpio_sim "$CONFIGFS_BASE")

  # 2) Patch the config file with the discovered identifier
  patch_cfg "$CFG_PATH" "$CHIP_NUM"

  echo "Created simulated GPIO chip: /dev/${CHIP_SYM} (${GPIO_LINES} lines)"
  echo "GPIO configfs: $GPIO_DEV_DIR  (set 'live' to 0 to tear down)"
  echo "Updated ${CFG_PATH}: ${GPIO_PLACEHOLDER} -> ${CHIP_NUM}"
}

main "$@"
