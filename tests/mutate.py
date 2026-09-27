"""Mutation check: each deliberate bug below must make at least one unit test fail.

For every mutant: patch one line of the library source, rebuild the test binary, run the
suite, and restore the file. The mutant is "killed" if the build succeeds and at least one test
fails (a hang past the timeout also counts as killed, and is reported as such). Before
anything else the unmodified code must build and pass, otherwise every mutant would look
killed.

  python3 tests/mutate.py            # build dir: $TSC_BUILD_DIR or build/mutation
"""
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
INC = ROOT / "include" / "tsc"
BUILD = Path(os.environ.get("TSC_BUILD_DIR", ROOT / "build" / "mutation"))
TEST_TIMEOUT_S = 900

MUTANTS = [
    # ---- safety guard: every invariant check must be load-bearing
    # (Weakening the conflict check to green-vs-green only is an *equivalent* mutant: the
    # transition rules already make a yellow next to a conflicting green unreachable, so no
    # test can tell the difference. Removing the check is not equivalent: two conflicting
    # groups released in the same tick pass every per-group transition rule.)
    (SRC / "safety_guard.cpp", "if (conflicts_[a][b] && cmd[a] != Signal::Red && cmd[b] != Signal::Red)",
     "if (false)"),
    (SRC / "safety_guard.cpp", "if (held < min_green_[g])", "if (held < 0)"),
    (SRC / "safety_guard.cpp", "if (held < yellow_[g])", "if (held + 500 < yellow_[g])"),
    (SRC / "safety_guard.cpp", "if (shown_[h] != Signal::Red || now - since_[h] < all_red_[h])",
     "if (shown_[h] != Signal::Red)"),
    (SRC / "safety_guard.cpp", "if (from == Signal::Green && to == Signal::Yellow) {",
     "if (from == Signal::Green && to != Signal::Green) {"),
    (SRC / "safety_guard.cpp", "if (last_ ? now <= *last_ : now < start_)", "if (false)"),
    (SRC / "safety_guard.cpp", "      shown_[g] = Signal::Yellow;\n", "      shown_[g] = Signal::Red;\n"),
    (SRC / "safety_guard.cpp", "if (shown_[g] == Signal::Yellow && now - since_[g] >= yellow_[g])",
     "if (shown_[g] == Signal::Yellow)"),
    (SRC / "safety_guard.cpp", "  if (!failsafe_) enter_failsafe(now, reason);", "  (void)now, (void)reason;"),
    # ---- actuated controller logic
    (SRC / "actuated.cpp", "if (e.on && !serving) calls_[p] = true;", "if (e.on && (serving || !serving)) calls_[p] = true;"),
    (SRC / "actuated.cpp", "if (green_elapsed < cfg.min_green) return std::nullopt;",
     "if (green_elapsed < cfg.passage) return std::nullopt;"),
    (SRC / "actuated.cpp", "if (!conflicting_call) return std::nullopt;",
     "if (!conflicting_call && false) return std::nullopt;"),
    (SRC / "actuated.cpp", "if (green_elapsed >= cfg.max_green) return Termination::MaxOut;",
     "if (green_elapsed > cfg.max_green) return Termination::MaxOut;"),
    (SRC / "actuated.cpp", "  if (effective_recall(p) == Recall::Max) return std::nullopt;  // held to max green\n", ""),
    (SRC / "actuated.cpp", "if (gap(p, now) >= cfg.passage) return Termination::GapOut;",
     "if (gap(p, now) > cfg.passage) return Termination::GapOut;"),
    (SRC / "actuated.cpp", "    if (monitor_.on(d)) return 0;\n", ""),
    (SRC / "actuated.cpp", "if (monitor_.faulty(d) || !cfg_.detectors[d].extends) continue;",
     "if (monitor_.faulty(d)) continue;"),
    (SRC / "actuated.cpp", "return std::max(base, cfg_.faults.fallback);", "return base;"),
    (SRC / "actuated.cpp", "return !monitor_.faulty(d) && monitor_.on(d); });", "return (void)d, false; });"),
    (SRC / "actuated.cpp", "void ActuatedController::on_green_start(PhaseId p, TimeMs /*now*/) { calls_[p] = false; }",
     "void ActuatedController::on_green_start(PhaseId /*p*/, TimeMs /*now*/) {}"),
    (SRC / "actuated.cpp", "return monitor_.faulty_count() > cfg_.faults.max_faulty;",
     "return monitor_.faulty_count() >= cfg_.faults.max_faulty;"),
    # ---- shared phase sequencing
    (SRC / "sequencer.cpp", "if (now - stage_start_ >= p.yellow) {", "if (now - stage_start_ >= p.yellow - 500) {"),
    (SRC / "sequencer.cpp", "if (now - stage_start_ >= p.all_red) try_start_green(now);", "try_start_green(now);"),
    (SRC / "sequencer.cpp", "startup_red_ = std::max(startup_red_, p.all_red);", "(void)p;"),
    # ---- detector fault monitor
    (SRC / "detector_monitor.cpp", "if (s.on && held >= cfg_.stuck_on) {", "if (s.on && held > cfg_.stuck_on) {"),
    (SRC / "detector_monitor.cpp",
     "} else if (!s.on && held >= cfg_.stuck_off && arrivals_ - s.arrivals_at_change >= cfg_.stuck_off_min_others) {",
     "} else if (false) {"),
    (SRC / "detector_monitor.cpp", "arrivals_ - s.arrivals_at_change >= cfg_.stuck_off_min_others", "true"),
    (SRC / "detector_monitor.cpp", "  if (!cfg_.enabled) return fresh;\n", ""),
    (SRC / "detector_monitor.cpp", "  if (s.fault) return false;\n", ""),
    (SRC / "detector_monitor.cpp", "if (e.t < s.last_change) {", "if (false) {"),
    # ---- multi-threaded runtime and queue
    (SRC / "runtime.cpp", "[&](const DetectorEvent& e) { return e.t <= next; });",
     "[&](const DetectorEvent& e) { return e.t < next; });"),
    (SRC / "runtime.cpp", "return a.on && !b.on;", "return !a.on && b.on;"),
    (SRC / "runtime.cpp", "for (DetectorId d : owned_[msg->producer]) pipeline_.controller().detector_feed_lost(d, next);",
     "(void)next;"),
    (SRC / "runtime.cpp", "if (e.t <= watermark_) {", "if (e.t < watermark_) {"),
    (INC / "blocking_queue.hpp", "    if (closed_) return false;\n    items_.push_back", "    items_.push_back"),
    # ---- config validation
    (SRC / "config.cpp", "if (c.conflicts[i][k] != c.conflicts[k][i]) {", "if (false) {"),
    (SRC / "config.cpp", "            d.extends = false;\n", "            d.extends = true;\n"),
    (SRC / "config.cpp", "if (p.min_green > p.max_green) {", "if (p.min_green >= p.max_green) {"),
    (SRC / "config.cpp", "if (p.groups[b] < n && c.conflicts[p.groups[a]][p.groups[b]]) {", "if (false) {"),
    (SRC / "config.cpp", "if (c.conflicts[i][i]) e.push_back", "if (false) e.push_back"),
    (SRC / "config.cpp", "error(where + \": unknown key '\" + key + \"'\");", "(void)where, (void)key;"),
    (SRC / "config.cpp", "if (p.yellow < 3000 || p.yellow > 6000)", "if (p.yellow < 2000 || p.yellow > 6000)"),
    (SRC / "config.cpp", "if (p.all_red < c.tick || p.all_red > 6000)", "if (p.all_red < 0 || p.all_red > 6000)"),
    # ---- Webster plan and the independent auditor
    (SRC / "webster.cpp", "(1.5 * plan.L + 5.0) / (1.0 - plan.Y)", "(1.5 * plan.L + 5.0) / (1.0 + plan.Y)"),
    (SRC / "webster.cpp", "y[p] = std::max(y[p], d.flow_vph / (d.lanes * d.saturation_vphpl));",
     "y[p] += d.flow_vph / (d.lanes * d.saturation_vphpl);"),
    (SRC / "auditor.cpp", "if (!failsafe && lasted < t.min_green)", "if (!failsafe && lasted < 0)"),
    (SRC / "auditor.cpp", "now - before[h].entered >= timing(h).all_red;", "true;"),
]


def sh(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)


def build():
    return sh(["cmake", "--build", BUILD, "--target", "tsc_tests"])


def tests():
    try:
        r = sh([BUILD / "tsc_tests", "--gtest_brief=1", "--gtest_fail_fast"], timeout=TEST_TIMEOUT_S)
        return r.returncode, ""
    except subprocess.TimeoutExpired:
        return -1, " (timeout)"


def main():
    for f, old, _ in MUTANTS:  # check every target before touching anything
        n = f.read_text(encoding="utf-8").count(old)
        assert n == 1, f"mutation target found {n} times in {f.name}: {old[:70]}"

    cfg = sh(["cmake", "--preset", "mutation", "-B", BUILD], cwd=ROOT)
    assert cfg.returncode == 0, cfg.stderr
    b = build()
    assert b.returncode == 0, "unmodified code does not build:\n" + b.stdout[-3000:]
    code, _ = tests()
    assert code == 0, "tests fail on the unmodified code; fix them before running the mutation check"

    escaped = 0
    start = time.time()
    for f, old, new in MUTANTS:
        text = f.read_text(encoding="utf-8")
        f.write_text(text.replace(old, new), encoding="utf-8")
        try:
            b = build()
            assert b.returncode == 0, f"mutant does not compile, so it proves nothing: {f.name}: {old[:70]}\n{b.stdout[-2000:]}"
            code, note = tests()
        finally:
            f.write_text(text, encoding="utf-8")
        killed = code != 0
        escaped += not killed
        print(f"{'killed ' if killed else 'ESCAPED'}{note}  {f.name}: {old.strip()[:80]}", flush=True)
    build()  # leave the build tree matching the restored sources
    print(f"{len(MUTANTS) - escaped}/{len(MUTANTS)} mutants killed ({time.time() - start:.0f} s)")
    sys.exit(1 if escaped else 0)


if __name__ == "__main__":
    main()
