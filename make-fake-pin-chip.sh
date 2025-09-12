#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./make-fake-gpio-sim.sh [path/to/config]
# Default:
CFG_PATH="${1:-examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg}"
PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"
SIM_NAME="slideprinter-sim"   # configfs device name (folder under gpio-sim)
BANK="gpio-bank0"
LINES=64

reexec_as_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root..."
    exec sudo --preserve-env=CFG_PATH,PLACEHOLDER,SIM_NAME,BANK,LINES bash "$0" "$CFG_PATH"
  fi
}
reexec_as_root

# --- Ensure configfs is mounted and gpio-sim module loaded ---
CONFIGFS_BASE="/sys/kernel/config"
if ! grep -q " $CONFIGFS_BASE " /proc/mounts 2>/dev/null; then
  modprobe configfs 2>/dev/null || true
  mkdir -p "$CONFIGFS_BASE"
  mount -t configfs none "$CONFIGFS_BASE"
fi

# Some distros have /config -> /sys/kernel/config symlink; prefer it if present.
if [[ -d /config/gpio-sim ]]; then
  CONFIGFS_BASE="/config"
fi

modprobe gpio-sim

SIM_ROOT="$CONFIGFS_BASE/gpio-sim"
DEV_DIR="$SIM_ROOT/$SIM_NAME"
BANK_DIR="$DEV_DIR/$BANK"

# --- (Re)create a clean simulated device ---
# If an old one exists, tear it down.
if [[ -d "$DEV_DIR" ]]; then
  if [[ -w "$DEV_DIR/live" ]]; then
    # Disable live if active
    echo 0 > "$DEV_DIR/live" || true
  fi
  # Remove any old bank dirs if present
  if [[ -d "$BANK_DIR" ]]; then
    # Remove optional per-line configs if any
    find "$BANK_DIR" -maxdepth 1 -type d -name 'line*' -exec rmdir {} + 2>/dev/null || true
    rmdir "$BANK_DIR" 2>/dev/null || true
  fi
  rmdir "$DEV_DIR" 2>/dev/null || true
fi

# Create fresh device + bank with 64 lines
mkdir -p "$DEV_DIR"
mkdir -p "$BANK_DIR"
echo "$LINES" > "$BANK_DIR/num_lines"

# Instantiate the simulated device
echo 1 > "$DEV_DIR/live"

# --- Discover the created chip name and number ---
# Read the bank's chip_name (e.g., "gpiochip3")
if [[ ! -r "$BANK_DIR/chip_name" ]]; then
  echo "Error: chip_name attribute not found. Is gpio-sim available on your kernel?" >&2
  exit 1
fi

CHIP_SYM="$(cat "$BANK_DIR/chip_name")"   # e.g. gpiochip3
if [[ ! "$CHIP_SYM" =~ ^gpiochip([0-9]+)$ ]]; then
  echo "Error: unexpected chip_name format: '$CHIP_SYM'" >&2
  exit 1
fi
CHIP_NUM="${BASH_REMATCH[1]}"

# Wait briefly for /dev/gpiochipN to appear (udev)
for i in {1..20}; do
  [[ -e "/dev/$CHIP_SYM" ]] && break
  sleep 0.05
done

# --- Patch your config file ---
if [[ ! -f "$CFG_PATH" ]]; then
  echo "Error: config file not found: $CFG_PATH" >&2
  exit 1
fi
cp -n "$CFG_PATH" "$CFG_PATH.bak" 2>/dev/null || true

# GNU/BSD sed compatibility
if sed --version >/dev/null 2>&1; then
  sed -i "s/${PLACEHOLDER}/${CHIP_NUM}/g" "$CFG_PATH"
else
  sed -i '' "s/${PLACEHOLDER}/${CHIP_NUM}/g" "$CFG_PATH"
fi

echo "Created simulated GPIO chip: /dev/${CHIP_SYM} (${LINES} lines)"
echo "Updated ${CFG_PATH}: ${PLACEHOLDER} -> ${CHIP_NUM}"
echo "ConfigFS node: $DEV_DIR  (set 'live' to 0 to tear down)"
