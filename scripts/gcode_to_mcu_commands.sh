#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/gcode_to_mcu_commands.sh [options] <gcode-file>
  scripts/gcode_to_mcu_commands.sh -c config_hp4_w_line_layers.g part.gcode
  scripts/gcode_to_mcu_commands.sh -m skycam --no-line-layers part.gcode

Options:
  -c, --config CONFIG
      RRF config file. Accepts either config_name.g or sys/config_name.g.
  -m, --machineType MACHINE
      Machine type used when --config is omitted.
      Supported: hp3, hp4, hp5, slideprinter, skycam, cubecorners.
      hp3/hp4/hp5 also accept hangprinter aliases such as hangprinter_3.
  --buildup, --line-layers
      Use config_<machine>_w_line_layers.g when --config is omitted.
  --no-buildup, --no_buildup, no_buildup, --no-line-layers
      Use config_<machine>.g when --config is omitted.
  -h, --help
      Show this help.
EOF
}

die() {
  printf 'gcode_to_mcu_commands.sh: %s\n' "$*" >&2
  printf 'Try scripts/gcode_to_mcu_commands.sh --help for usage.\n' >&2
  exit 2
}

normalize_machine_type() {
  local value="$1"
  case "$value" in
    hp3|hangprinter3|hangprinter_3|hangprinter-v3|hangprinter_v3)
      printf 'hp3\n'
      ;;
    hp4|hangprinter4|hangprinter_4|hangprinter-v4|hangprinter_v4)
      printf 'hp4\n'
      ;;
    hp5|hangprinter5|hangprinter_5|hangprinter-v5|hangprinter_v5)
      printf 'hp5\n'
      ;;
    slideprinter|skycam|cubecorners)
      printf '%s\n' "$value"
      ;;
    *)
      die "unsupported machine type: $value"
      ;;
  esac
}

resolve_klipper_config() {
  local machine="$1"
  local line_layers="$2"
  local suffix=""
  if [[ "$line_layers" == "1" ]]; then
    suffix="-with-buildup"
  fi
  printf 'public/klipper/%s/printer-%s-linux-mcu%s.cfg\n' "$machine" "$machine" "$suffix"
}

resolve_config_selector() {
  local value="$1"
  local name machine line_layers
  case "$value" in
    sys/config_*.g)
      name="${value#sys/config_}"
      ;;
    config_*.g)
      name="${value#config_}"
      ;;
    *)
      die "--config must look like config_<name>.g or sys/config_<name>.g"
      ;;
  esac
  name="${name%.g}"
  line_layers="0"
  if [[ "$name" == *_w_line_layers ]]; then
    line_layers="1"
    name="${name%_w_line_layers}"
  fi
  machine="$(normalize_machine_type "$name")"
  resolve_klipper_config "$machine" "$line_layers"
}

config=""
machine_type="hp3"
line_layers="1"
gcode_file=""

while (($#)); do
  case "$1" in
    -c|--config)
      (($# >= 2)) || die "$1 requires an argument"
      config="$(resolve_config_selector "$2")"
      shift 2
      ;;
    --config=*)
      config="$(resolve_config_selector "${1#*=}")"
      shift
      ;;
    -m|--machineType)
      (($# >= 2)) || die "$1 requires an argument"
      machine_type="$(normalize_machine_type "$2")"
      shift 2
      ;;
    --machineType=*)
      machine_type="$(normalize_machine_type "${1#*=}")"
      shift
      ;;
    --buildup|--line-layers)
      line_layers="1"
      shift
      ;;
    --no-buildup|--no_buildup|no_buildup|--no-line-layers)
      line_layers="0"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      die "unknown option: $1"
      ;;
    *)
      [[ -z "$gcode_file" ]] || die "unexpected extra argument: $1"
      gcode_file="$1"
      shift
      ;;
  esac
done

[[ -n "$gcode_file" ]] || die "missing <gcode-file>"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "$config" ]]; then
  config="$(resolve_klipper_config "$machine_type" "$line_layers")"
fi

CONFIG_PATH="${ROOT_DIR}/${config}"
[[ -f "$CONFIG_PATH" ]] || die "Klipper config not found: $config"

BASENAME="$(basename "$gcode_file" .gcode)"
PYTHON="${KLIPPY_PYTHON:-${ROOT_DIR}/.venv/bin/python}"

"$PYTHON" "$ROOT_DIR/klipper/klippy/klippy.py" \
  "$CONFIG_PATH" \
  -i "$gcode_file" \
  -o "$ROOT_DIR/public/mcu_commands/${BASENAME}.serial" \
  -v -d "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict"

"$PYTHON" "$ROOT_DIR/klipper/klippy/parsedump.py" \
  "$ROOT_DIR/public/klipper/linux_mcu/klipper.dict" \
  "$ROOT_DIR/public/mcu_commands/${BASENAME}.serial" \
  > "$ROOT_DIR/public/mcu_commands/${BASENAME}.txt"
