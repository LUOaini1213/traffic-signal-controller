#include <gtest/gtest.h>

#include "helpers.hpp"
#include "tsc/detector_monitor.hpp"

using namespace tsc;
using namespace tsc::test;

namespace {
// Advance and stop-bar loops of the north-south turn-across phase.
const std::vector<std::string> kNsTurnLoops = {"N_2", "S_2", "N_2s", "S_2s"};

// Keeps every detector except the listed ones healthy with a vehicle each 40 s.
void background_traffic(Driver& d, double until_s, const std::vector<std::string>& silent) {
  const char* ids[] = {"N_0", "N_1", "N_2", "S_0", "S_1", "S_2", "E_0", "E_1", "E_2", "W_0", "W_1", "W_2",
                       "N_2s", "S_2s", "E_2s", "W_2s"};
  for (double t = 1; t < until_s; t += 40) {
    for (const char* id : ids) {
      if (std::find(silent.begin(), silent.end(), id) == silent.end()) d.pulse(t, id);
    }
  }
}
}  // namespace

TEST(Faults, StuckOnIsDeclaredAtTheThreshold) {
  Driver d(repo_config());
  background_traffic(d, 400, {"E_1"});
  d.event(20, "E_1", true);  // never clears
  const DetectorId e1 = repo_config().detector_index("E_1");
  d.run_until(319.5);
  EXPECT_FALSE(d.ctl().monitor().faulty(e1));
  EXPECT_EQ(d.ctl().effective_recall(EW_T), Recall::Min);
  d.run_until(320);
  EXPECT_TRUE(d.ctl().monitor().faulty(e1));
  ASSERT_EQ(d.ctl().monitor().faults().size(), 1u);
  EXPECT_EQ(d.ctl().monitor().faults()[0].kind, FaultKind::StuckOn);
  EXPECT_EQ(d.ctl().monitor().faults()[0].at, s(320));
  EXPECT_EQ(d.ctl().effective_recall(EW_T), Recall::Max);  // fallback
  EXPECT_EQ(d.ctl().effective_recall(NS_T), Recall::Min);  // other phases untouched
  EXPECT_FALSE(d.ctl().wants_failsafe());
}

TEST(Faults, StuckOffDetectorFallsBackToRecallSoThePhaseIsServed) {
  Driver d(repo_config());
  background_traffic(d, 1300, kNsTurnLoops);  // right-turn loops on N and S report nothing
  d.run_until(899.5);
  EXPECT_EQ(d.ctl().stats()[NS_R].served, 0u);  // no calls: skipped every cycle
  EXPECT_EQ(d.ctl().monitor().faulty_count(), 0u);
  d.run_until(900);
  EXPECT_EQ(d.ctl().monitor().faulty_count(), 4u);
  EXPECT_EQ(d.ctl().monitor().faults()[0].kind, FaultKind::StuckOff);
  EXPECT_EQ(d.ctl().effective_recall(NS_R), Recall::Max);
  EXPECT_TRUE(d.ctl().has_call(NS_R));
  d.run_until(1300);
  EXPECT_GE(d.ctl().stats()[NS_R].served, 3u);
  // Max recall: the right-turn phase now runs to max green every time.
  EXPECT_EQ(d.ctl().stats()[NS_R].max_outs, d.ctl().stats()[NS_R].served);
  EXPECT_FALSE(d.last().failsafe);
  EXPECT_EQ(d.pipe().guard().refusals(), 0u);
}

TEST(Faults, MinFallbackServesAtMinGreen) {
  Driver d(config_with([](nlohmann::json& j) { j["faults"]["fallback_recall"] = "min"; }));
  background_traffic(d, 1300, kNsTurnLoops);
  d.run_until(1300);
  EXPECT_EQ(d.ctl().effective_recall(NS_R), Recall::Min);
  EXPECT_GE(d.ctl().stats()[NS_R].served, 3u);
  EXPECT_EQ(d.ctl().stats()[NS_R].max_outs, 0u);
  EXPECT_EQ(d.ctl().stats()[NS_R].gap_outs, d.ctl().stats()[NS_R].served);
}

TEST(Faults, WithFaultHandlingDisabledTheStuckPhaseStarves) {
  Driver d(config_with([](nlohmann::json& j) { j["faults"]["enabled"] = false; }));
  background_traffic(d, 2000, kNsTurnLoops);
  d.run_until(2000);
  EXPECT_EQ(d.ctl().monitor().faulty_count(), 0u);
  EXPECT_EQ(d.ctl().stats()[NS_R].served, 0u);
}

TEST(Faults, EventsFromAFaultyDetectorAreIgnored) {
  Driver d(repo_config());
  background_traffic(d, 1200, kNsTurnLoops);
  d.event(1000, "N_2", true);  // the loop "comes back" after being declared stuck-off at 900 s
  const DetectorId n2 = repo_config().detector_index("N_2");
  d.run_until(1000);
  EXPECT_TRUE(d.ctl().monitor().faulty(n2));  // latched until maintenance
  EXPECT_FALSE(d.ctl().monitor().on(n2));     // and its events are no longer applied
}

TEST(Faults, TooManyFaultyDetectorsTriggersFailSafe) {
  for (int stuck : {4, 5}) {
    Driver d(repo_config());
    const std::vector<std::string> ids = {"N_0", "S_0", "E_0", "W_0", "N_1"};
    std::vector<std::string> silent(ids.begin(), ids.begin() + stuck);
    background_traffic(d, 400, silent);
    for (const auto& id : silent) d.event(10, id, true);
    d.run_until(400);
    EXPECT_EQ(d.ctl().monitor().faulty_count(), static_cast<std::size_t>(stuck));
    if (stuck <= 4) {
      EXPECT_FALSE(d.last().failsafe);
    } else {
      EXPECT_TRUE(d.last().failsafe);
      EXPECT_EQ(d.last().displayed, SignalVector(8, Signal::Red));
      EXPECT_EQ(d.pipe().guard().failsafe_reason(), "too many faulty detectors");
    }
  }
}

TEST(Faults, LostFeedIsAFault) {
  Driver d(repo_config());
  const DetectorId w2 = repo_config().detector_index("W_2");
  d.ctl().detector_feed_lost(w2, 0);
  d.ctl().detector_feed_lost(99, 0);  // unknown: ignored
  EXPECT_TRUE(d.ctl().monitor().faulty(w2));
  EXPECT_EQ(d.ctl().monitor().faults()[0].kind, FaultKind::FeedLost);
  EXPECT_EQ(d.ctl().effective_recall(EW_R), Recall::Max);
  EXPECT_TRUE(d.ctl().has_call(EW_R));
}

TEST(DetectorMonitor, TracksEdgesAndCountsStaleEvents) {
  const Config cfg = repo_config();
  DetectorMonitor m(cfg, 0);
  EXPECT_TRUE(m.update({1000, 0, true}));
  EXPECT_TRUE(m.on(0));
  EXPECT_FALSE(m.update({2000, 0, true}));  // no change
  EXPECT_FALSE(m.update({500, 0, false}));  // older than the last change
  EXPECT_EQ(m.stale_events(), 1u);
  EXPECT_TRUE(m.on(0));
  EXPECT_TRUE(m.update({3000, 0, false}));
  EXPECT_EQ(m.last_change(0), 3000);
  EXPECT_FALSE(m.update({s(2001), 0, false}));  // same state again
}

TEST(DetectorMonitor, SilenceIsOnlyAFaultWhileOtherLoopsSeeTraffic) {
  const Config cfg = repo_config();  // stuck_off 900 s, stuck_off_min_others 20
  const std::size_t n = cfg.detectors.size();
  DetectorMonitor m(cfg, 0);
  EXPECT_TRUE(m.check(s(2000)).empty());  // the whole junction quiet for 2000 s: no fault

  // 19 vehicles over loop 1: not enough evidence yet that the other loops are broken.
  TimeMs t = s(2000);
  for (int k = 0; k < 19; ++k, t += 1000) {
    m.update({t, 1, true});
    m.update({t + 500, 1, false});
  }
  EXPECT_TRUE(m.check(t).empty());
  m.update({t, 1, true});  // the 20th
  m.update({t + 500, 1, false});
  const auto f = m.check(t + 500);
  EXPECT_EQ(f.size(), n - 1);  // every loop except the busy one
  EXPECT_FALSE(m.faulty(1));
  EXPECT_EQ(f[0].kind, FaultKind::StuckOff);
  EXPECT_TRUE(m.check(t + 10000).empty());  // latched, not reported twice
  EXPECT_EQ(m.faulty_count(), n - 1);
  EXPECT_FALSE(m.update({t + 20000, 0, true}));  // events from a faulty loop are ignored
}

TEST(DetectorMonitor, StuckOffNeedsTheFullSilentTime) {
  const Config cfg = repo_config();
  DetectorMonitor m(cfg, 0);
  m.update({s(100), 0, true});
  m.update({s(101), 0, false});
  for (TimeMs t = s(200); t < s(1000); t += s(10)) {  // plenty of traffic on loop 1
    m.update({t, 1, true});
    m.update({t + 500, 1, false});
  }
  m.check(s(1000.5));
  EXPECT_FALSE(m.faulty(0));  // silent 899.5 s
  m.check(s(1001));
  EXPECT_TRUE(m.faulty(0));   // silent 900 s
}
