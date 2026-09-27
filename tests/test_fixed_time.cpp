#include <gtest/gtest.h>

#include "helpers.hpp"
#include "tsc/auditor.hpp"
#include "tsc/fixed_time.hpp"

using namespace tsc;
using namespace tsc::test;

namespace {
std::vector<TickRecord> run_fixed(const Config& cfg, std::vector<TimeMs> greens, double until_s,
                                  bool with_events = false) {
  Pipeline pipe(cfg, std::make_unique<FixedTimeController>(cfg, 0, std::move(greens)), 0);
  std::vector<TickRecord> out;
  for (TimeMs t = 0; t <= s(until_s); t += cfg.tick) {
    if (with_events) pipe.on_event({t, static_cast<DetectorId>(t / 500 % 12), t % 1000 == 0});
    out.push_back(pipe.tick(t));
  }
  return out;
}
Signal at(const std::vector<TickRecord>& h, double t_s, GroupId g) {
  return h.at(static_cast<std::size_t>(s(t_s) / 500)).displayed.at(g);
}
}  // namespace

TEST(FixedTime, FollowsThePlanEveryCycleWithoutSkipping) {
  const Config cfg = repo_config();
  // Greens 20 / 8 / 15 / 6; each phase then has 3 s yellow + 2 s all-red: cycle 69 s.
  const auto h = run_fixed(cfg, {s(20), s(8), s(15), s(6)}, 150);
  EXPECT_EQ(at(h, 1.5, G_NT), Signal::Red);
  EXPECT_EQ(at(h, 2.0, G_NT), Signal::Green);
  EXPECT_EQ(at(h, 21.5, G_NT), Signal::Green);
  EXPECT_EQ(at(h, 22.0, G_NT), Signal::Yellow);
  EXPECT_EQ(at(h, 27.0, G_NR), Signal::Green);  // served although no detector ever called it
  EXPECT_EQ(at(h, 35.0, G_NR), Signal::Yellow);
  EXPECT_EQ(at(h, 40.0, G_ET), Signal::Green);
  EXPECT_EQ(at(h, 55.0, G_ET), Signal::Yellow);
  EXPECT_EQ(at(h, 60.0, G_WR), Signal::Green);
  EXPECT_EQ(at(h, 66.0, G_WR), Signal::Yellow);
  EXPECT_EQ(at(h, 71.0, G_NT), Signal::Green);  // second cycle starts one cycle (69 s) later
  EXPECT_EQ(at(h, 70.5, G_NT), Signal::Red);
}

TEST(FixedTime, DetectorsAreIgnoredAndTerminationsAreCounted) {
  const Config cfg = repo_config();
  const auto quiet = run_fixed(cfg, {s(20), s(8), s(15), s(6)}, 300, false);
  const auto busy = run_fixed(cfg, {s(20), s(8), s(15), s(6)}, 300, true);
  for (std::size_t i = 0; i < quiet.size(); ++i) ASSERT_EQ(quiet[i].displayed, busy[i].displayed) << i;

  FixedTimeController ctl(cfg, 0, {s(20), s(8), s(15), s(6)});
  for (TimeMs t = 0; t <= s(300); t += 500) (void)ctl.step(t);
  EXPECT_EQ(ctl.cycle(), s(69));
  EXPECT_EQ(ctl.stats()[NS_R].fixed_ends, 4u);  // right-turn greens end at 35, 104, 173, 242 s
  EXPECT_EQ(ctl.stats()[NS_R].gap_outs + ctl.stats()[NS_R].max_outs, 0u);
  EXPECT_FALSE(ctl.wants_failsafe());
}

TEST(FixedTime, OutputPassesTheIndependentAudit) {
  const Config cfg = repo_config();
  const auto h = run_fixed(cfg, {s(10), s(6), s(10), s(6)}, 600);
  InvariantAuditor audit(cfg, 0);
  for (const auto& r : h) audit.observe(r.t, r.displayed);
  EXPECT_EQ(audit.violations(), 0u);
}

TEST(FixedTime, RejectsPlansBelowMinGreenOrOffTheTick) {
  const Config cfg = repo_config();
  EXPECT_THROW(FixedTimeController(cfg, 0, {s(9.5), s(6), s(10), s(6)}), std::invalid_argument);
  EXPECT_THROW(FixedTimeController(cfg, 0, {s(10), s(6.2), s(10), s(6)}), std::invalid_argument);
  EXPECT_THROW(FixedTimeController(cfg, 0, {s(10), s(6), s(10)}), std::invalid_argument);
  EXPECT_NO_THROW(FixedTimeController(cfg, 0, {s(10), s(6), s(10), s(6)}));
}
