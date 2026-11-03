#!/usr/bin/env bash
# Build all pre-compiled firmware variants with PlatformIO.
#
# Usage: ./build.sh [-o output_dir] [firmware_dir ...]
#
# Every [env:emon*] in each firmware's platformio.ini is built and the hex is
# copied to <output_dir>/<hardware>/<env>_v<version>.ino.hex, where <hardware>
# is the env name up to the first underscore (emonTx4, emonTx5, emonPi2) and
# <version> comes from firmware_version in the .ino source.
#
# output_dir defaults to compiled/, firmware_dir defaults to all firmwares.

set -euo pipefail

cd "$(dirname "$0")"

out=compiled
while getopts "o:" opt; do
  case $opt in
    o) out=$OPTARG ;;
    *) echo "Usage: $0 [-o output_dir] [firmware_dir ...]" >&2; exit 1 ;;
  esac
done
shift $((OPTIND - 1))

firmwares=("$@")
if [ ${#firmwares[@]} -eq 0 ]; then
  firmwares=(emon_DB_6CT emon_DB_12CT emon_CM_6CT_temperature)
fi

pio=$(command -v pio || command -v platformio || echo "$HOME/.platformio/penv/bin/pio")

built=()
for fw in "${firmwares[@]}"; do
  fw=${fw%/}
  version=$(sed -n 's/.*firmware_version = {"\([0-9.]*\)\\n\\r"}.*/\1/p' "$fw/$fw.ino")
  if [ -z "$version" ]; then
    echo "Could not read firmware_version from $fw/$fw.ino" >&2
    exit 1
  fi
  version=${version//./_}

  envs=($(sed -n 's/^\[env:\(emon[A-Za-z0-9_]*\)\]$/\1/p' "$fw/platformio.ini"))
  echo "== $fw v$version: ${#envs[@]} variants"

  args=()
  for env in "${envs[@]}"; do args+=(-e "$env"); done
  "$pio" run -d "$fw" "${args[@]}"

  for env in "${envs[@]}"; do
    hw=${env%%_*}
    mkdir -p "$out/$hw"
    dest="$out/$hw/${env}_v${version}.ino.hex"
    cp "$fw/.pio/build/$env/firmware.hex" "$dest"
    built+=("$dest")
  done
done

echo
echo "Built ${#built[@]} firmwares:"
printf '  %s\n' "${built[@]}"
