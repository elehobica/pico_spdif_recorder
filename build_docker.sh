#!/bin/bash
#------------------------------------------------------
# Copyright (c) 2026, Elehobica
# Released under the BSD-2-Clause
# refer to https://opensource.org/licenses/BSD-2-Clause
#------------------------------------------------------
#
# Local Docker build script that mirrors .github/workflows/build-binaries.yml.
# Uses the same SDK image as CI (elehobica/pico-sdk-dev-docker:sdk-2.1.1-1.0.0)
# and runs cmake/make inside the container.

set -e

IMAGE="elehobica/pico-sdk-dev-docker:sdk-2.1.1-1.0.0"
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

usage() {
  cat <<EOF
Usage: $0 [target] [options]

Targets:
  pico    Build for Pico / Pico W       (rp2040, output: build/)
  pico2   Build for Pico 2 / Pico 2 W   (rp2350, output: build2/)
  all     Build both (default)

Options:
  -h, --help     Show this help
  -k, --keep     Keep build directory contents (incremental build)
EOF
}

TARGET=all
KEEP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    pico|pico2|all) TARGET="$1" ;;
    -h|--help)      usage; exit 0 ;;
    -k|--keep)      KEEP=1 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

# Run cmake/make inside the SDK container.
# Args: $1=build_dir  $2=extra cmake options (may be empty)
run_build() {
  local build_dir="$1"
  local cmake_extra="$2"

  if [[ "$KEEP" -eq 0 ]]; then
    rm -rf "$PROJECT_ROOT/$build_dir"
  fi
  mkdir -p "$PROJECT_ROOT/$build_dir"

  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e PICO_SDK_PATH=/home/rp2dev/pico/pico-sdk \
    -e PICO_EXTRAS_PATH=/home/rp2dev/pico/pico-extras \
    -e PICO_EXAMPLES_PATH=/home/rp2dev/pico/pico-examples \
    -v "$PROJECT_ROOT":/workspace \
    -w "/workspace/$build_dir" \
    "$IMAGE" \
    bash -c "cmake $cmake_extra /workspace && make -j\$(nproc)"
}

BUILT_DIRS=()

case "$TARGET" in
  pico|all)
    echo "===== Build Pico (rp2040) ====="
    run_build build ""
    BUILT_DIRS+=(build)
    ;;
esac
case "$TARGET" in
  pico2|all)
    echo "===== Build Pico 2 (rp2350) ====="
    run_build build2 "-DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2"
    BUILT_DIRS+=(build2)
    ;;
esac

echo ""
echo "===== Output ====="
for d in "${BUILT_DIRS[@]}"; do
  [[ -f "$PROJECT_ROOT/$d/pico_spdif_recorder.uf2" ]] && echo "  $d/pico_spdif_recorder.uf2"
done
