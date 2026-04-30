#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/rrf_server.sh -c config_slideprinter.g
  scripts/rrf_server.sh -m hp3 --line-layers
  scripts/rrf_server.sh -m slideprinter --no-line-layers

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
  printf 'rrf_server.sh: %s\n' "$*" >&2
  printf 'Try scripts/rrf_server.sh --help for usage.\n' >&2
  exit 2
}

normalize_config() {
  local value="$1"
  case "$value" in
    sys/config_*.g)
      printf '%s\n' "$value"
      ;;
    config_*.g)
      printf 'sys/%s\n' "$value"
      ;;
    *)
      die "--config must look like config_<name>.g or sys/config_<name>.g"
      ;;
  esac
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

config=""
machine_type=""
line_layers=""

while (($#)); do
  case "$1" in
    -c|--config)
      (($# >= 2)) || die "$1 requires an argument"
      config="$(normalize_config "$2")"
      shift 2
      ;;
    --config=*)
      config="$(normalize_config "${1#*=}")"
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
    *)
      die "unknown option: $1"
      ;;
  esac
done

if [[ -z "$config" ]]; then
  [[ -n "$machine_type" ]] || die "--machineType is required when --config is omitted"
  [[ -n "$line_layers" ]] || die "--line-layers or --no-line-layers is required when --config is omitted"

  if [[ "$line_layers" == "1" ]]; then
    config="sys/config_${machine_type}_w_line_layers.g"
  else
    config="sys/config_${machine_type}.g"
  fi
fi

exec ./RRF/build/rrf_simulator --vsd RRF/run/vsd -c "$config" --server -p 8080
