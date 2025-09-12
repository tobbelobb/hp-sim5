#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./make-fake-gpio-iio-sim.sh [path/to/config]
# Default:
CFG_PATH="${1:-examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg}"

# Placeholders to replace in your config file
GPIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"
IIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_IIO_DEVICE_NAME"

# Names for the simulated devices
GPIO_SIM_NAME="slideprinter-gpio-sim" # configfs name for the digital device
IIO_SIM_NAME="slideprinter-iio-sim" # configfs name for the analog device

# Number of lines/channels for each device
GPIO_LINES=32 # Pins 0-31
IIO_CHANNELS=32 # Pins 32-63

# --- Helper function to re-run as root ---
reexec_as_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root..."
    exec sudo --preserve-env=CFG_PATH,GPIO_PLACEHOLDER,IIO_PLACEHOLDER,GPIO_SIM_NAME,IIO_SIM_NAME,GPIO_LINES,IIO_CHANNELS bash "$0" "$CFG_PATH"
  fi
}
reexec_as_root

# --- Ensure configfs is mounted and modules are loaded ---
CONFIGFS_BASE="/sys/kernel/config"
if ! grep -q " $CONFIGFS_BASE " /proc/mounts 2>/dev/null; then
  modprobe configfs 2>/dev/null || true
  mkdir -p "$CONFIGFS_BASE"
  mount -t configfs none "$CONFIGFS_BASE"
fi

if [[ -d /config/gpio-sim ]]; then
  CONFIGFS_BASE="/config"
fi

modprobe gpio-sim
modprobe iio_dummy

# --- Define paths for both devices ---
GPIO_DEV_DIR="$CONFIGFS_BASE/gpio-sim/$GPIO_SIM_NAME"
GPIO_BANK_DIR="$GPIO_DEV_DIR/gpio-bank0"

IIO_DEV_DIR="$CONFIGFS_BASE/iio/devices/dummy/$IIO_SIM_NAME"

# --- (Re)create a clean simulated device for GPIO (Digital) ---
echo "Cleaning up any old GPIO device..."
if [[ -d "$GPIO_DEV_DIR" ]]; then
  if [[ -w "$GPIO_DEV_DIR/live" ]]; then
    echo 0 > "$GPIO_DEV_DIR/live" || true
  fi
  if [[ -d "$GPIO_BANK_DIR" ]]; then
    find "$GPIO_BANK_DIR" -maxdepth 1 -type d -name 'line*' -exec rmdir {} + 2>/dev/null || true
    rmdir "$GPIO_BANK_DIR" 2>/dev/null || true
  fi
  rmdir "$GPIO_DEV_DIR" 2>/dev/null || true
fi

# Create fresh GPIO device with the specified number of lines
echo "Creating new simulated GPIO chip..."
mkdir -p "$GPIO_DEV_DIR"
mkdir -p "$GPIO_BANK_DIR"
echo "$GPIO_LINES" > "$GPIO_BANK_DIR/num_lines"

# Instantiate the simulated GPIO device
echo 1 > "$GPIO_DEV_DIR/live"

# --- Discover the created GPIO chip name and number ---
if [[ ! -r "$GPIO_BANK_DIR/chip_name" ]]; then
  echo "Error: chip_name attribute not found for GPIO. Is gpio-sim available?" >&2
  exit 1
fi

GPIO_CHIP_SYM="$(cat "$GPIO_BANK_DIR/chip_name")"
if [[ ! "$GPIO_CHIP_SYM" =~ ^gpiochip([0-9]+)$ ]]; then
  echo "Error: unexpected GPIO chip_name format: '$GPIO_CHIP_SYM'" >&2
  exit 1
fi
GPIO_CHIP_NUM="${BASH_REMATCH[1]}"

# Wait briefly for /dev/gpiochipN to appear (udev)
for i in {1..20}; do
  [[ -e "/dev/$GPIO_CHIP_SYM" ]] && break
  sleep 0.05
done

# --- (Re)create a clean simulated device for IIO (Analog) ---
echo "Cleaning up any old IIO device..."
if [[ -d "$IIO_DEV_DIR" ]]; then
  rmdir "$IIO_DEV_DIR" 2>/dev/null || true
fi

# Create fresh IIO dummy device with the specified number of channels
echo "Creating new simulated IIO (Analog) device..."
mkdir -p "$IIO_DEV_DIR"
echo "$IIO_CHANNELS" > "$IIO_DEV_DIR/num_channels"
# The iio_dummy driver creates a device for us to interact with.

# --- Discover the created IIO device name ---
IIO_DEV_PATH=""
for i in /sys/bus/iio/devices/iio:device*; do
  if [[ -f "$i/name" ]] && grep -q "$IIO_SIM_NAME" "$i/name"; then
    IIO_DEV_PATH="$i"
    break
  fi
done

if [[ -z "$IIO_DEV_PATH" ]]; then
  echo "Error: Could not find created IIO device. Is iio_dummy available?" >&2
  exit 1
fi
IIO_DEV_NAME=$(basename "$IIO_DEV_PATH")

# --- Patch your config file ---
if [[ ! -f "$CFG_PATH" ]]; then
  echo "Error: config file not found: $CFG_PATH" >&2
  exit 1
fi

cp -n "$CFG_PATH" "$CFG_PATH.bak" 2>/dev/null || true

# Patch the GPIO placeholder
if sed --version >/dev/null 2>&1; then
  sed -i "s/${GPIO_PLACEHOLDER}/${GPIO_CHIP_NUM}/g" "$CFG_PATH"
else
  sed -i '' "s/${GPIO_PLACEHOLDER}/${GPIO_CHIP_NUM}/g" "$CFG_PATH"
fi

# Patch the IIO placeholder
if sed --version >/dev/null 2>&1; then
  sed -i "s/${IIO_PLACEHOLDER}/${IIO_DEV_NAME}/g" "$CFG_PATH"
else
  sed -i '' "s/${IIO_PLACEHOLDER}/${IIO_DEV_NAME}/g" "$CFG_PATH"
fi

# --- Final output ---
echo "Created simulated GPIO chip: /dev/${GPIO_CHIP_SYM} (${GPIO_LINES} lines)"
echo "Created simulated IIO device: ${IIO_DEV_PATH} (${IIO_CHANNELS} channels)"
echo "Updated ${CFG_PATH}:"
echo "  - ${GPIO_PLACEHOLDER} -> ${GPIO_CHIP_NUM}"
echo "  - ${IIO_PLACEHOLDER} -> ${IIO_DEV_NAME}"
echo ""
echo "To tear down both devices, set 'live' to 0 for the GPIO device and remove the IIO directory:"
echo "  sudo echo 0 > $GPIO_DEV_DIR/live"
echo "  sudo rmdir $IIO_DEV_DIR"
echo ""
echo "To mock an analog pin (e.g., pin 32, which corresponds to channel 0):"
echo "  sudo echo 1.23 > ${IIO_DEV_PATH}/in_voltage0_raw"
echo ""
echo "Check the values with:"
echo "  cat ${IIO_DEV_PATH}/in_voltage0_raw"
