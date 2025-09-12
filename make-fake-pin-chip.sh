#!/usr/bin/env bash
set -euo pipefail

# Usage:
#    ./make-fake-gpio-iio-sim.sh [path/to/config]
# Default:
CFG_PATH="${1:-examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg}"

# Placeholders to replace in your config file
GPIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"
IIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_IIO_DEVICE_NAME"

# Names for the simulated devices
GPIO_SIM_NAME="slideprinter-gpio-sim" # configfs name for the digital device
IIO_SIM_NAME="slideprinter-iio-sim"   # configfs name for the analog device

# Number of lines/channels for each device
GPIO_LINES=32   # Digital pins 0-31
IIO_CHANNELS=32 # Analog pins (e.g., Klipper's analog 32-63)

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

# --- (Re)create a clean simulated device for IIO (Analog) ---
# This must be done BEFORE creating the GPIO device, as we need to reload the iio_dummy module
# with specific parameters, which is cleaner to do first.
IIO_DEV_DIR="$CONFIGFS_BASE/iio/devices/dummy/$IIO_SIM_NAME"

echo "Cleaning up any old IIO device..."
if [[ -d "$IIO_DEV_DIR" ]]; then
  rmdir "$IIO_DEV_DIR" 2>/dev/null || true
fi

# Unload module if it's already loaded to ensure we can set parameters
echo "Reloading iio_dummy module to create ${IIO_CHANNELS} channels..."
rmmod iio_dummy 2>/dev/null || true
# Load iio_dummy with the num_channels parameter to create N simple voltage inputs
modprobe iio_dummy num_channels=$IIO_CHANNELS
modprobe gpio-sim

# --- Define paths for GPIO device ---
GPIO_DEV_DIR="$CONFIGFS_BASE/gpio-sim/$GPIO_SIM_NAME"
GPIO_BANK_DIR="$GPIO_DEV_DIR/gpio-bank0"

# --- (Re)create a clean simulated device for GPIO (Digital) ---
echo "Cleaning up any old GPIO device..."
if [[ -d "$GPIO_DEV_DIR" ]]; then
  if [[ -w "$GPIO_DEV_DIR/live" ]]; then
    echo 0 | tee "$GPIO_DEV_DIR/live" > /dev/null || true
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
echo "$GPIO_LINES" | tee "$GPIO_BANK_DIR/num_lines" > /dev/null

# Instantiate the simulated GPIO device
echo 1 | tee "$GPIO_DEV_DIR/live" > /dev/null

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

# --- Create and discover the IIO device ---
echo "Creating new simulated IIO (Analog) device..."
mkdir -p "$IIO_DEV_DIR"

IIO_DEV_PATH=""
# Wait for the device to appear
for i in {1..20}; do
    for dev_path in /sys/bus/iio/devices/iio:device*; do
      # The name attribute is created by configfs
      if [[ -f "$dev_path/name" ]] && grep -q "^$IIO_SIM_NAME$" "$dev_path/name"; then
        IIO_DEV_PATH="$dev_path"
        break 2
      fi
    done
    sleep 0.05
done

if [[ -z "$IIO_DEV_PATH" ]]; then
  echo "Error: Could not find created IIO device. Is iio_dummy available?" >&2
  exit 1
fi
IIO_DEV_NAME=$(basename "$IIO_DEV_PATH")

# --- Discover the associated HWMON device for writing mock values ---
# The hwmon device is the key to setting the mock sensor values
HWMON_PATH=""
# The real device path links the iio device to its hwmon sibling
REAL_DEV_PATH=$(readlink -f "$IIO_DEV_PATH/device")
if [[ -d "$REAL_DEV_PATH/hwmon" ]]; then
    HWMON_PATH=$(find "$REAL_DEV_PATH/hwmon" -mindepth 1 -maxdepth 1 -name 'hwmon*' | head -n 1)
fi

if [[ -z "$HWMON_PATH" ]]; then
    echo "Warning: Could not find associated HWMON device. You may not be able to mock input values." >&2
    echo "This can happen on older kernels. The created pins will still exist but may have a value of 0."
fi

# --- Patch your config file ---
if [[ ! -f "$CFG_PATH" ]]; then
  echo "Error: config file not found: $CFG_PATH" >&2
  exit 1
fi

echo "Backing up config to ${CFG_PATH}.bak"
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
echo ""
echo "--- Success! ---"
echo "Created simulated GPIO chip: /dev/${GPIO_CHIP_SYM} (${GPIO_LINES} lines)"
echo "Created simulated IIO device: ${IIO_DEV_PATH} (${IIO_CHANNELS} channels)"
echo "Updated ${CFG_PATH}:"
echo "  - ${GPIO_PLACEHOLDER} -> ${GPIO_CHIP_NUM}"
echo "  - ${IIO_PLACEHOLDER} -> ${IIO_DEV_NAME}"
echo ""
echo "To tear down both devices, run:"
echo "  sudo rmdir $IIO_DEV_DIR && echo 0 | sudo tee $GPIO_DEV_DIR/live"
echo ""
if [[ -n "$HWMON_PATH" ]]; then
  # Pin 32 in the config would be the first analog pin, which is channel 0.
  echo ">>> IMPORTANT: How to Mock Analog Values <<<"
  echo "The 'in_voltageX_raw' files are READ-ONLY. To set a value, you must write to the HWMON interface."
  echo "To mock an analog pin (e.g., config pin 32, which is channel 0) to 1.23 Volts:"
  echo "  echo 1230 | sudo tee ${HWMON_PATH}/in0_input"
  echo ""
  echo "Then, you can check the value (this is what applications like Klipper will read):"
  echo "  cat ${IIO_DEV_PATH}/in_voltage0_raw"
  echo "(Note: The value read from '_raw' is a unitless ADC value, not millivolts)"
else
    echo "NOTE: No HWMON device was found. You will not be able to set mock values for the analog pins."
fi
