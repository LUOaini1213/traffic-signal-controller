// Randomised property tests. Fixed seeds, so every run checks the same sequences.
//
// 1. Controller property: for thousands of random configs and random detector histories
//    (bursts, long occupancies, chatter, stuck-on/stuck-off loops), the controllers never
//    propose a command the safety guard refuses, the independent auditor sees no violation in
//    what is displayed, and no called phase waits longer than one full worst-case cycle.
// 2. Guard property: feeding the guard arbitrary garbage commands never makes it display an
//    unsafe sequence, and it never blocks a command that its own check() accepts.
#include <gtest/gtest.h>

#include <cstdlib>
#include <random>

#include "helpers.hpp"
#include "tsc/auditor.hpp"
#include "tsc/fixed_time.hpp"
#include "tsc/safety_guard.hpp"

using namespace tsc;
using namespace tsc::test;
using nlohmann::json;

namespace {

int iterations(int fallback) {
  // Read on the test thread before any other thread exists.
  if (const char* v = std::getenv("TSC_PROPERTY_ITERATIONS")) {  // NOLINT(concurrency-mt-unsafe)
    return std::max(1, std::atoi(v));
  }
  return fallback;
}

double pick(std::mt19937_64& rng, double lo, double hi, double step = 0.5) {
  const auto n = static_cast<long>((hi - lo) / step);
  return lo + step * static_cast<double>(std::uniform_int_distribution<long>(0, n)(rng));
}

json random_config(std::mt19937_64& rng) {
  json j = repo_config_json();
  const char* recalls[] = {"none", "min", "max"};
  for (auto& p : j["phases"]) {
    const double min_g = pick(rng, 3, 15);
    p["min_green_s"] = min_g;
    p["max_green_s"] = min_g + pick(rng, 0, 40);
    p["passage_s"] = pick(rng, 0.5, std::min(4.0, min_g));
    p["yellow_s"] = pick(rng, 3, 4.5);
    p["all_red_s"] = pick(rng, 0.5, 3);
    p["recall"] = recalls[std::uniform_int_distribution<int>(0, 2)(rng)];
  }
  j["faults"] = {{"enabled", std::bernoulli_distribution(0.8)(rng)},
                 {"stuck_on_s", pick(rng, 20, 150, 1)},
                 {"stuck_off_s", pick(rng, 30, 250, 1)},
                 {"stuck_off_min_others", std::uniform_int_distribution<int>(0, 30)(rng)},
                 {"fallback_recall", std::bernoulli_distribution(0.5)(rng) ? "min" : "max"},
                 {"max_faulty", std::uniform_int_distribution<int>(0, 12)(rng)}};
  return j;
}

// Random detector history on the 0.5 s grid, including pathological patterns.
std::vector<DetectorEvent> random_events(std::mt19937_64& rng, std::size_t detectors, TimeMs end) {
  std::vector<DetectorEvent> ev;
  for (DetectorId d = 0; d < detectors; ++d) {
    const double rate = std::uniform_real_distribution<double>(0.0, 0.4)(rng);  // veh/s
    const int mode = std::uniform_int_distribution<int>(0, 9)(rng);  // 0: stuck-on, 1: stuck-off, 2: chatter
    const TimeMs break_at = static_cast<TimeMs>(pick(rng, 0, static_cast<double>(end) / 1000.0)) * 1000;
    TimeMs t = 0;
    std::exponential_distribution<double> gap(rate > 0 ? rate : 1e-9);
    while (true) {
      t += 500 * (1 + static_cast<TimeMs>(gap(rng) * 2.0));
      if (t >= end) break;
      if (mode == 0 && t >= break_at) {  // loop sticks on
        ev.push_back({t, d, true});
        break;
      }
      if (mode == 1 && t >= break_at) break;  // loop goes silent
      ev.push_back({t, d, true});
      if (mode == 2) ev.push_back({t, d, true});  // duplicate edge
      const TimeMs occ = 500 * std::uniform_int_distribution<TimeMs>(0, mode == 2 ? 1 : 12)(rng);
      ev.push_back({t + occ, d, false});
      t += occ;
    }
  }
  std::stable_sort(ev.begin(), ev.end(), [](const auto& a, const auto& b) { return a.t < b.t; });
  return ev;
}

}  // namespace

TEST(Property, ControllersNeverViolateInvariantsOrStarveAPhase) {
  std::mt19937_64 rng(20260927);
  const int n = iterations(2000);
  const TimeMs end = s(300);
  std::size_t faults_seen = 0, failsafes_seen = 0, ticks = 0;
  for (int it = 0; it < n; ++it) {
    const Config cfg = parse_config(random_config(rng));
    const auto events = random_events(rng, cfg.detectors.size(), end);
    const bool fixed = std::bernoulli_distribution(0.2)(rng);

    std::unique_ptr<SignalController> ctl;
    ActuatedController* act = nullptr;
    if (fixed) {
      std::vector<TimeMs> greens;
      greens.reserve(cfg.phases.size());
      for (const auto& p : cfg.phases) greens.push_back(p.min_green + s(pick(rng, 0, 20)));
      ctl = std::make_unique<FixedTimeController>(cfg, 0, greens);
    } else {
      auto a = std::make_unique<ActuatedController>(cfg, 0);
      act = a.get();
      ctl = std::move(a);
    }
    Pipeline pipe(cfg, std::move(ctl), 0);
    InvariantAuditor audit(cfg, 0);

    // Worst-case wait for a called phase: every phase (including the one currently green)
    // runs to max green plus its clearance, once.
    TimeMs bound = 0;
    for (const auto& p : cfg.phases) bound += p.max_green + p.yellow + p.all_red;
    bound += 3000;  // start-up all-red
    std::vector<std::optional<TimeMs>> waiting_since(cfg.phases.size());

    std::size_t next = 0;
    for (TimeMs t = 0; t <= end; t += cfg.tick) {
      while (next < events.size() && events[next].t <= t) pipe.on_event(events[next++]);
      const TickRecord r = pipe.tick(t);
      audit.observe(t, r.displayed, r.failsafe);
      ++ticks;
      if (act == nullptr || r.failsafe) continue;
      for (PhaseId p = 0; p < cfg.phases.size(); ++p) {
        const bool green = r.displayed[cfg.phases[p].groups[0]] == Signal::Green;
        if (green || !act->has_call(p)) {
          waiting_since[p].reset();
        } else if (!waiting_since[p]) {
          waiting_since[p] = t;
        }
        ASSERT_TRUE(!waiting_since[p] || t - *waiting_since[p] <= bound)
            << "iteration " << it << ": phase " << p << " called since " << *waiting_since[p]
            << " ms but not served by " << t << " ms";
      }
    }
    ASSERT_EQ(pipe.guard().refusals(), 0u) << "iteration " << it << ": " << pipe.guard().failsafe_reason();
    ASSERT_EQ(audit.violations(), 0u) << "iteration " << it << ": " << audit.messages().front();
    if (act != nullptr) faults_seen += act->monitor().faulty_count();
    if (pipe.guard().failsafe()) ++failsafes_seen;
  }
  // The generator must actually reach the interesting paths, or the property proves little.
  EXPECT_GT(faults_seen, 0u);
  EXPECT_GT(failsafes_seen, 0u);
  RecordProperty("ticks", static_cast<int>(ticks));
}

TEST(Property, GuardNeverDisplaysAnUnsafeSequenceWhateverItIsFed) {
  std::mt19937_64 rng(7);
  const Config cfg = repo_config();
  const int n = iterations(2000) / 2;
  std::size_t accepted = 0;
  for (int it = 0; it < n; ++it) {
    SafetyGuard guard(cfg, 0);
    InvariantAuditor audit(cfg, 0);
    SignalVector cmd(cfg.groups.size(), Signal::Red);
    for (TimeMs t = 0; t <= s(200); t += cfg.tick) {
      // Mostly small edits of the last command (so some are legal), sometimes pure noise.
      const int edits = std::uniform_int_distribution<int>(0, 2)(rng);
      for (int e = 0; e < edits; ++e) {
        cmd[std::uniform_int_distribution<std::size_t>(0, cmd.size() - 1)(rng)] =
            static_cast<Signal>(std::uniform_int_distribution<int>(0, 2)(rng));
      }
      if (std::bernoulli_distribution(0.02)(rng)) {
        for (auto& c : cmd) c = static_cast<Signal>(std::uniform_int_distribution<int>(0, 2)(rng));
      }
      const bool legal = !guard.check(t, cmd).has_value();
      const bool was_failsafe = guard.failsafe();
      const SignalVector shown = guard.apply(t, cmd);
      if (legal && !was_failsafe) {
        ASSERT_EQ(shown, cmd) << "guard blocked a command its own check() accepts";
        ++accepted;
      }
      audit.observe(t, shown, guard.failsafe());
      if (guard.failsafe()) cmd = shown;  // keep proposing from what is shown
    }
    ASSERT_EQ(audit.violations(), 0u) << "iteration " << it << ": " << audit.messages().front();
  }
  EXPECT_GT(accepted, 500u);  // enough legal commands got through to exercise acceptance
}
