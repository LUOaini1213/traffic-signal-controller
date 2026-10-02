"""Mutation check: each deliberate bug below must make at least one unit test fail.

For every mutant: patch one line of the source, rebuild the test and CLI binaries, run the
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
CLI = ROOT / "apps" / "tsc_cli.cpp"
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
    (SRC / "safety_guard.cpp", "enter_failsafe(safe_now, *why);", "enter_failsafe(now, *why);"),
    (SRC / "safety_guard.cpp", "  now = trusted_time(now);\n", ""),
    (SRC / "safety_guard.cpp", "  last_ = safe_now;", "  last_ = now;"),
    # ---- actuated controller logic
    (SRC / "actuated.cpp", "if (e.on && !serving) calls_[p] = true;", "if (e.on && (serving || !serving)) calls_[p] = true;"),
    (SRC / "actuated.cpp", "if (green_elapsed < cfg.min_green) return std::nullopt;",
     "if (green_elapsed < cfg.passage) return std::nullopt;"),
    (SRC / "actuated.cpp", "if (!conflicting_call) return std::nullopt;",
     "if (!conflicting_call && false) return std::nullopt;"),
    (SRC / "actuated.cpp", "if (green_elapsed >= cfg.max_green) return Termination::MaxOut;",
     "if (green_elapsed > cfg.max_green) return Termination::MaxOut;"),
    (SRC / "actuated.cpp", "effective_recall(p) == Recall::Max || gap(p, now) < cfg.passage;", "gap(p, now) < cfg.passage;"),
    (SRC / "actuated.cpp", "gap(p, now) < cfg.passage;", "gap(p, now) <= cfg.passage;"),
    # a green that rested past max green and then ends on a late call is a gap-out, not a max-out
    (SRC / "actuated.cpp",
     "  if (!extended) return Termination::GapOut;\n  if (green_elapsed >= cfg.max_green) return Termination::MaxOut;",
     "  if (green_elapsed >= cfg.max_green) return Termination::MaxOut;\n  if (!extended) return Termination::GapOut;"),
    # a vehicle leaving a loop is not a new vehicle and must not place a call
    (SRC / "actuated.cpp", "if (e.on && !serving) calls_[p] = true;", "if (!serving) calls_[p] = true;"),
    (SRC / "actuated.cpp", "    if (monitor_.on(d)) return 0;\n", ""),
    (SRC / "actuated.cpp", "if (monitor_.faulty(d) || !cfg_.detectors[d].extends) continue;",
     "if (monitor_.faulty(d)) continue;"),
    (SRC / "actuated.cpp", "return std::max(base, cfg_.faults.fallback);", "return base;"),
    (SRC / "actuated.cpp", "return !monitor_.faulty(d) && monitor_.on(d); });", "return (void)d, false; });"),
    (SRC / "actuated.cpp", "void ActuatedController::on_green_start(PhaseId p, TimeMs /*now*/) { calls_[p] = false; }",
     "void ActuatedController::on_green_start(PhaseId /*p*/, TimeMs /*now*/) {}"),
    (SRC / "actuated.cpp", "return broken > cfg_.faults.max_faulty;", "return broken >= cfg_.faults.max_faulty;"),
    # stuck-off loops must not count towards junction fail-safe; lost feeds must
    (SRC / "actuated.cpp", "monitor_.faulty_count(FaultKind::StuckOn) + monitor_.faulty_count(FaultKind::FeedLost);",
     "monitor_.faulty_count();"),
    (SRC / "actuated.cpp", " + monitor_.faulty_count(FaultKind::FeedLost);", ";"),
    # ---- shared phase sequencing
    (SRC / "sequencer.cpp", "if (now - stage_start_ >= p.yellow) {", "if (now - stage_start_ >= p.yellow - 500) {"),
    (SRC / "sequencer.cpp", "if (now - stage_start_ >= p.all_red) try_start_green(now);", "try_start_green(now);"),
    (SRC / "sequencer.cpp", "startup_red_ = std::max(startup_red_, p.all_red);", "(void)p;"),
    # ---- detector fault monitor
    (SRC / "detector_monitor.cpp", "if (s.on && held >= cfg_.stuck_on) {", "if (s.on && held > cfg_.stuck_on) {"),
    (SRC / "detector_monitor.cpp",
     "} else if (!s.on && held >= cfg_.stuck_off && peers_saw >= cfg_.stuck_off_min_others) {",
     "} else if ((void)peers_saw, false) {"),
    (SRC / "detector_monitor.cpp", "peers_saw >= cfg_.stuck_off_min_others", "((void)peers_saw, true)"),
    # silence must be judged against the loop's own approach, not the whole junction
    (SRC / "detector_monitor.cpp", "  if (e.on) ++arrivals_[s.approach];", "  if (e.on) for (auto& a : arrivals_) ++a;"),
    (SRC / "detector_monitor.cpp", "  if (!cfg_.enabled) return fresh;\n", ""),
    (SRC / "detector_monitor.cpp", "  if (s.fault) return false;\n", ""),
    (SRC / "detector_monitor.cpp", "if (e.t < s.last_change) {", "if (false) {"),
    (SRC / "detector_monitor.cpp",
     "if (s.fault && (kind != FaultKind::FeedLost || *s.fault == FaultKind::FeedLost)) return;",
     "if (s.fault) return;"),
    (SRC / "detector_monitor.cpp", " || *s.fault == FaultKind::FeedLost", ""),
    # ---- multi-threaded runtime and queue
    (SRC / "runtime.cpp", "[&](const DetectorEvent& e) { return e.t <= next; });",
     "[&](const DetectorEvent& e) { return e.t < next; });"),
    (SRC / "runtime.cpp", "return a.on && !b.on;", "return !a.on && b.on;"),
    (SRC / "runtime.cpp", "for (DetectorId d : owned_[p]) pipeline_.controller().detector_feed_lost(d, at);", "(void)at;"),
    # the lost-feed fault must be timed from the closed producer's own last watermark ...
    (SRC / "runtime.cpp", "lost_at[msg->producer] = first_tick_after(watermark[msg->producer]);",
     "lost_at[msg->producer] = std::max(next, first_tick_after(start - 1));"),
    # ... and be stamped with that time, not 0
    (SRC / "runtime.cpp", "pipeline_.controller().detector_feed_lost(d, at);", "pipeline_.controller().detector_feed_lost(d, at - at);"),
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
    # ---- real replay entry point, including Windows CSV exports and strict input checks
    (CLI, "if (!line.empty() && line.back() == '\\r') line.pop_back();", "// keep CR"),
    (CLI, 'fields[2] != "0" && fields[2] != "1"', 'false'),
    (CLI, 'csv_fields(line) != std::array<std::string, 3>{"t_s", "detector_id", "on"}', 'false'),
    (CLI, "used != input.size() || ", ""),
    (CLI, " || seconds < 0", ""),
    (CLI, "milliseconds >= exclusive_limit", "(milliseconds >= exclusive_limit && false)"),
    (CLI, "line.find(',', second + 1) != std::string::npos", "false"),
]


def sh(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)


def build():
    return sh(["cmake", "--build", BUILD, "--target", "tsc_tests", "tsc", "-j", "2"])


def tests():
    commands = [
        [BUILD / "tsc_tests", "--gtest_brief=1", "--gtest_fail_fast"],
        [sys.executable, ROOT / "tests" / "test_cli.py", "--binary", BUILD / "tsc",
         "--config", ROOT / "configs" / "intersection.json"],
    ]
    for cmd in commands:
        try:
            r = sh(cmd, timeout=TEST_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return -1, " (timeout)"
        if r.returncode:
            output = r.stdout + r.stderr
            assert "[  FAILED  ]" in output or "FAILED (failures=" in output, (
                "test process failed without an assertion; not a valid mutation kill:\n" + output[-3000:])
            return r.returncode, ""
    return 0, ""


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
        original = f.read_bytes()
        text = original.decode("utf-8")
        f.write_bytes(text.replace(old, new).encode("utf-8"))
        try:
            b = build()
            assert b.returncode == 0, f"mutant does not compile, so it proves nothing: {f.name}: {old[:70]}\n{b.stdout[-2000:]}"
            code, note = tests()
        finally:
            f.write_bytes(original)
            assert f.read_bytes() == original, f"source restoration failed: {f}"
        killed = code != 0
        escaped += not killed
        print(f"{'killed ' if killed else 'ESCAPED'}{note}  {f.name}: {old.strip()[:80]}", flush=True)
    restored = build()  # leave the build tree matching the restored sources
    assert restored.returncode == 0, "restored code does not build:\n" + restored.stdout[-3000:]
    print(f"{len(MUTANTS) - escaped}/{len(MUTANTS)} mutants killed ({time.time() - start:.0f} s)")
    sys.exit(1 if escaped else 0)


if __name__ == "__main__":
    main()
