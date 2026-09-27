#!/usr/bin/env bash
# One entry point for CI and for local runs, so both execute exactly the same commands.
#
#   scripts/ci.sh test       gcc Debug build (-Werror) + full test suite
#   scripts/ci.sh asan       ASan + UBSan build + full test suite
#   scripts/ci.sh tsan       TSan build (clang) + full test suite + short SUMO run of the TSan harness
#   scripts/ci.sh tidy       clang-tidy over src/, apps/ and tests/
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

configure_build() {  # preset [build-dir-name] [extra cmake args...]
  local preset="$1" dir="${2:-$1}"
  shift $(( $# < 2 ? $# : 2 ))
  cmake --preset "$preset" -B "$BUILD_ROOT/$dir" "$@" >/dev/null
  cmake --build "$BUILD_ROOT/$dir" -j "$JOBS"
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
  # The SUMO harness is built with TSan too: its producer threads are exercised in a short
  # SUMO run below, not only in the unit tests.
  configure_build tsan tsan -DTSC_BUILD_SUMO=ON
  # Kernels with 32-bit mmap randomisation break TSan's shadow memory layout; CI lowers it.
  export TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1
  run_tests tsan
  TSC_BUILD_DIR="$BUILD_ROOT/tsan" python3 sim/smoke.py --end 420
}

job_tidy() {
  # Separate tree configured with the SUMO harness, so apps/tsc_sumo.cpp has compile commands.
  cmake --preset debug -B "$BUILD_ROOT/tidy" -DTSC_BUILD_SUMO=ON >/dev/null
  # One clang-tidy per file, in parallel; xargs fails if any of them reports a finding.
  ls src/*.cpp apps/*.cpp tests/*.cpp |
    xargs -P "$JOBS" -n 1 clang-tidy -p "$BUILD_ROOT/tidy" --quiet --warnings-as-errors='*'
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
