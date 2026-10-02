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

TEST(DetectorMonitor, SilenceIsOnlyAFaultWhileLoopsOfTheSameApproachSeeTraffic) {
  const Config cfg = repo_config();  // stuck_off 900 s, stuck_off_min_others 20
  DetectorMonitor m(cfg, 0);
  EXPECT_TRUE(m.check(s(2000)).empty());  // the whole junction quiet for 2000 s: no fault

  // 19 vehicles over N_1: not enough evidence yet that the other north loops are broken.
  const DetectorId n1 = cfg.detector_index("N_1");
  TimeMs t = s(2000);
  for (int k = 0; k < 19; ++k, t += 1000) {
    m.update({t, n1, true});
    m.update({t + 500, n1, false});
  }
  EXPECT_TRUE(m.check(t).empty());
  m.update({t, n1, true});  // the 20th
  m.update({t + 500, n1, false});
  const auto f = m.check(t + 500);
  // Only the other loops of the north approach are suspected; loops on the quiet approaches
  // are judged against their own neighbours, which saw nothing either.
  ASSERT_EQ(f.size(), 3u);
  for (const char* id : {"N_0", "N_2", "N_2s"}) EXPECT_TRUE(m.faulty(cfg.detector_index(id))) << id;
  for (const char* id : {"N_1", "S_0", "S_1", "S_2", "E_0", "E_2s", "W_1"}) {
    EXPECT_FALSE(m.faulty(cfg.detector_index(id))) << id;
  }
  EXPECT_EQ(f[0].kind, FaultKind::StuckOff);
  EXPECT_TRUE(m.check(t + 10000).empty());  // latched, not reported twice
  EXPECT_EQ(m.faulty_count(), 3u);
  EXPECT_EQ(m.faulty_count(FaultKind::StuckOff), 3u);
  EXPECT_EQ(m.faulty_count(FaultKind::StuckOn), 0u);
  const DetectorId n0 = cfg.detector_index("N_0");
  EXPECT_FALSE(m.update({t + 20000, n0, true}));  // events from a faulty loop are ignored
}

TEST(DetectorMonitor, WithoutApproachLabelsAllLoopsJudgeEachOther) {
  const Config cfg = config_with([](nlohmann::json& j) {
    for (auto& d : j["detectors"]) d.erase("approach");
  });
  DetectorMonitor m(cfg, 0);
  TimeMs t = s(1000);
  for (int k = 0; k < 20; ++k, t += 1000) {
    m.update({t, 1, true});
    m.update({t + 500, 1, false});
  }
  EXPECT_EQ(m.check(t).size(), cfg.detectors.size() - 1);
}

// Found in review: at night only the main road has traffic, one vehicle every 20 s over N_1
// and S_1. Earlier versions judged silence against the whole junction, declared all 14 quiet
// loops stuck-off at 900 s and latched the junction into fail-safe for good.
TEST(Faults, NightTrafficOnOneRoadDoesNotSendTheJunctionToFailSafe) {
  Driver d(repo_config());
  for (double t = 1; t < 3600; t += 20) {
    d.pulse(t, "N_1");
    d.pulse(t + 3, "S_1");
  }
  d.run_until(3600);
  EXPECT_FALSE(d.last().failsafe);
  EXPECT_FALSE(d.ctl().wants_failsafe());
  EXPECT_EQ(d.pipe().guard().refusals(), 0u);
  // Silent loops on the busy approaches are suspected (their neighbour lane counts ~45
  // vehicles in 15 min) and put their phases on recall; the quiet cross road is left alone.
  const auto& m = d.ctl().monitor();
  for (const char* id : {"N_0", "N_2", "N_2s", "S_0", "S_2", "S_2s"}) {
    EXPECT_TRUE(m.faulty(repo_config().detector_index(id))) << id;
  }
  for (const char* id : {"N_1", "S_1", "E_0", "E_1", "E_2", "E_2s", "W_0", "W_1", "W_2", "W_2s"}) {
    EXPECT_FALSE(m.faulty(repo_config().detector_index(id))) << id;
  }
  EXPECT_EQ(m.faulty_count(), m.faulty_count(FaultKind::StuckOff));
  // The junction keeps cycling normally: the cross road is still served on its min recall.
  const auto served_before = d.ctl().stats()[EW_T].served;
  d.run_until(3900);
  EXPECT_GT(d.ctl().stats()[EW_T].served, served_before);
  EXPECT_FALSE(d.last().failsafe);
}

TEST(Faults, StuckOffLoopsNeverCountTowardsFailSafe) {
  Driver d(repo_config());  // max_faulty 4
  // Every loop of the north and south approaches except N_1 / S_1 goes silent (6 loops).
  background_traffic(d, 1200, {"N_0", "N_2", "N_2s", "S_0", "S_2", "S_2s"});
  d.run_until(1200);
  EXPECT_EQ(d.ctl().monitor().faulty_count(FaultKind::StuckOff), 6u);
  EXPECT_FALSE(d.ctl().wants_failsafe());
  EXPECT_FALSE(d.last().failsafe);
  EXPECT_EQ(d.ctl().effective_recall(NS_T), Recall::Max);
  EXPECT_EQ(d.ctl().effective_recall(NS_R), Recall::Max);
  EXPECT_EQ(d.ctl().effective_recall(EW_R), Recall::None);
}

TEST(Faults, LostFeedsCountTowardsFailSafe) {
  Driver d(repo_config());
  for (const char* id : {"N_0", "N_1", "N_2", "N_2s"}) {
    d.ctl().detector_feed_lost(repo_config().detector_index(id), 0);
  }
  EXPECT_FALSE(d.ctl().wants_failsafe());  // 4 = max_faulty
  d.ctl().detector_feed_lost(repo_config().detector_index("S_0"), 0);
  EXPECT_TRUE(d.ctl().wants_failsafe());
}

TEST(Faults, LostFeedsAfterNightTrafficStillCountEveryDisconnectedDetector) {
  const Config cfg = repo_config();
  Driver d(cfg);
  for (double t = 1; t < 900; t += 20) {
    d.pulse(t, "N_1");
    d.pulse(t + 3, "S_1");
  }
  d.run_until(900);
  const auto& monitor = d.ctl().monitor();
  ASSERT_EQ(monitor.faulty_count(FaultKind::StuckOff), 6u);
  ASSERT_EQ(monitor.faults().size(), 6u);
  ASSERT_FALSE(d.last().failsafe);

  // Both busy approaches lose their feeds after six quiet neighbouring loops have
  // already latched stuck-off. All eight disconnected loops must now count.
  for (const char* id : {"N_0", "N_1", "N_2", "N_2s"}) {
    d.ctl().detector_feed_lost(cfg.detector_index(id), s(900.5));
  }
  EXPECT_EQ(monitor.faulty_count(FaultKind::FeedLost), 4u);
  EXPECT_FALSE(d.ctl().wants_failsafe());  // exactly max_faulty
  for (const char* id : {"S_0", "S_1", "S_2", "S_2s"}) {
    d.ctl().detector_feed_lost(cfg.detector_index(id), s(900.5));
  }
  EXPECT_EQ(monitor.faulty_count(), 8u);
  EXPECT_EQ(monitor.faulty_count(FaultKind::StuckOff), 0u);
  EXPECT_EQ(monitor.faulty_count(FaultKind::FeedLost), 8u);
  EXPECT_TRUE(d.ctl().wants_failsafe());
  d.run_until(900.5);
  EXPECT_TRUE(d.last().failsafe);
  d.run_until(904);
  EXPECT_EQ(d.last().displayed, SignalVector(cfg.groups.size(), Signal::Red));
  EXPECT_EQ(d.pipe().guard().failsafe_reason(), "too many faulty detectors");

  ASSERT_EQ(monitor.faults().size(), 14u);  // six initial faults and eight lost feeds
  for (std::size_t i = 0; i < monitor.faults().size(); ++i) {
    EXPECT_EQ(monitor.faults()[i].kind, i < 6 ? FaultKind::StuckOff : FaultKind::FeedLost);
    EXPECT_EQ(monitor.faults()[i].at, i < 6 ? s(900) : s(900.5));
  }
}

TEST(DetectorMonitor, FeedLossUpgradesLatchedFaultWithoutDuplicateOrDowngrade) {
  for (const FaultKind initial : {FaultKind::StuckOff, FaultKind::StuckOn}) {
    SCOPED_TRACE(to_string(initial));
    DetectorMonitor monitor(repo_config(), 0);
    monitor.force_fault(0, initial, s(10));
    monitor.force_fault(0, initial, s(15));
    ASSERT_EQ(monitor.faults().size(), 1u);
    monitor.force_fault(0, FaultKind::FeedLost, s(20));
    monitor.force_fault(0, FaultKind::FeedLost, s(25));
    monitor.force_fault(0, FaultKind::StuckOn, s(30));
    monitor.force_fault(0, FaultKind::StuckOff, s(35));

    EXPECT_EQ(monitor.faulty_count(), 1u);
    EXPECT_EQ(monitor.faulty_count(FaultKind::FeedLost), 1u);
    EXPECT_EQ(monitor.faulty_count(FaultKind::StuckOn), 0u);
    EXPECT_EQ(monitor.faulty_count(FaultKind::StuckOff), 0u);
    EXPECT_TRUE(monitor.faulty(0));
    EXPECT_FALSE(monitor.update({s(40), 0, true}));  // the upgraded fault stays latched
    EXPECT_FALSE(monitor.on(0));
    EXPECT_TRUE(monitor.check(s(1000)).empty());
    ASSERT_EQ(monitor.faults().size(), 2u);
    EXPECT_EQ(monitor.faults()[0].detector, 0u);
    EXPECT_EQ(monitor.faults()[0].kind, initial);
    EXPECT_EQ(monitor.faults()[0].at, s(10));
    EXPECT_EQ(monitor.faults()[1].detector, 0u);
    EXPECT_EQ(monitor.faults()[1].kind, FaultKind::FeedLost);
    EXPECT_EQ(monitor.faults()[1].at, s(20));
  }
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
