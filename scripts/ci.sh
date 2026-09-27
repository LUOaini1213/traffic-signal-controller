#!/usr/bin/env bash
# One entry point for CI and for local runs, so both execute exactly the same commands.
#
#   scripts/ci.sh test       gcc Debug build (-Werror) + full test suite
#   scripts/ci.sh asan       ASan + UBSan build + full test suite
#   scripts/ci.sh tsan       TSan build (clang) + full test suite
#   scripts/ci.sh tidy       clang-tidy over src/ and apps/ (uses the Debug build's compile_commands.json)
#   scripts/ci.sh sumo       Release build with the SUMO harness + short SUMO smoke run
#   scripts/ci.sh mutation   tests/mutate.py (plants bugs one by one, every one must be caught)
#   scripts/ci.sh all        all of the above
#
# Build directories go under $TSC_BUILD_ROOT (default: ./build).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ROOT="${TSC_BUILD_ROOT:-$ROOT/build}"
JOBS="$(nproc 2>/dev/null || echo 2)"
cd "$ROOT"

configure_build() {  # preset
  cmake --preset "$1" -B "$BUILD_ROOT/$1" >/dev/null
  cmake --build "$BUILD_ROOT/$1" -j "$JOBS"
}

run_tests() {  # preset
  ctest --test-dir "$BUILD_ROOT/$1" --output-on-failure -j "$JOBS" --timeout 900
}

job_test() {
  configure_build debug
  run_tests debug
  "$BUILD_ROOT/debug/tsc" validate configs/intersection.json
}

job_asan() {
  configure_build asan
  ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:strict_string_checks=1 \
  UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
    run_tests asan
}

job_tsan() {
  configure_build tsan
  # Kernels with 32-bit mmap randomisation break TSan's shadow memory layout; CI lowers it.
  TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1 run_tests tsan
}

job_tidy() {
  configure_build debug
  local files
  files=$(ls src/*.cpp apps/tsc_cli.cpp)
  # shellcheck disable=SC2086
  clang-tidy -p "$BUILD_ROOT/debug" --quiet --warnings-as-errors='*' $files
}

job_sumo() {
  configure_build release
  export TSC_BUILD_DIR="$BUILD_ROOT/release"
  python3 sim/smoke.py
}

job_mutation() {
  TSC_BUILD_DIR="$BUILD_ROOT/mutation" python3 tests/mutate.py
}

case "${1:-all}" in
  test) job_test ;;
  asan) job_asan ;;
  tsan) job_tsan ;;
  tidy) job_tidy ;;
  sumo) job_sumo ;;
  mutation) job_mutation ;;
  all) job_test; job_tidy; job_asan; job_tsan; job_sumo; job_mutation ;;
  *) echo "unknown job '$1'" >&2; exit 64 ;;
esac
