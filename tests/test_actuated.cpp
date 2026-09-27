#include <gtest/gtest.h>

#include "helpers.hpp"

using namespace tsc;
using namespace tsc::test;

// Timeline of the repository config with no traffic at all:
//   0.0-2.0 start-up all-red, 2.0 NS_through green (min recall), gaps out at min green 12.0,
//   yellow 12.0-15.0, all-red 15.0-17.0, NS_right skipped (no call), 17.0 EW_through green.

TEST(Actuated, StartUpAllRedThenFirstRecalledPhase) {
  Driver d(repo_config());
  d.run_until(3);
  for (GroupId g = 0; g < 8; ++g) EXPECT_EQ(d.at(1.5, g), Signal::Red) << g;
  EXPECT_EQ(d.at(2.0, G_NT), Signal::Green);
  EXPECT_EQ(d.at(2.0, G_ST), Signal::Green);
  EXPECT_EQ(d.at(2.0, G_NR), Signal::Red);
  EXPECT_EQ(d.at(2.0, G_ET), Signal::Red);
  EXPECT_EQ(d.ctl().stats()[NS_T].served, 1u);
}

TEST(Actuated, MinGreenIsHonouredThenGapsOutWithoutTraffic) {
  Driver d(repo_config());
  d.run_until(40);
  EXPECT_EQ(d.at(11.5, G_NT), Signal::Green);
  EXPECT_EQ(d.at(12.0, G_NT), Signal::Yellow);
  EXPECT_EQ(d.ctl().stats()[NS_T].gap_outs, 1u);
  EXPECT_EQ(d.ctl().stats()[NS_T].max_outs, 0u);
}

TEST(Actuated, ClearanceIsYellowThenAllRedThenNextPhase) {
  Driver d(repo_config());
  d.run_until(40);
  EXPECT_EQ(d.at(14.5, G_NT), Signal::Yellow);
  EXPECT_EQ(d.at(15.0, G_NT), Signal::Red);
  EXPECT_EQ(d.at(16.5, G_ET), Signal::Red);  // all-red interval
  EXPECT_EQ(d.at(16.5, G_NR), Signal::Red);
  EXPECT_EQ(d.at(17.0, G_ET), Signal::Green);
  EXPECT_EQ(d.at(17.0, G_WT), Signal::Green);
  EXPECT_EQ(d.ctl().stats()[NS_R].skipped, 1u);
  EXPECT_EQ(d.ctl().stats()[NS_R].served, 0u);
  EXPECT_EQ(d.first(Signal::Green, G_NT, 17), 32.0);  // EW_right skipped as well
}

TEST(Actuated, VehiclesExtendGreenUntilMaxOut) {
  Driver d(repo_config());
  for (double t = 3; t < 100; t += 2) d.pulse(t, "N_0");
  d.run_until(60);
  EXPECT_EQ(d.at(46.5, G_NT), Signal::Green);
  EXPECT_EQ(d.at(47.0, G_NT), Signal::Yellow);  // 2.0 + max green 45
  EXPECT_EQ(d.ctl().stats()[NS_T].max_outs, 1u);
  EXPECT_EQ(d.ctl().stats()[NS_T].gap_outs, 0u);
}

TEST(Actuated, GapOutExactlyOnePassageTimeAfterLastVehicleLeaves) {
  Driver d(repo_config());
  d.pulse(5, "S_1");
  d.pulse(10, "N_0");
  d.pulse(13, "N_1");  // loop clears at 13.5, passage 3.0
  d.run_until(30);
  EXPECT_EQ(d.at(12.0, G_NT), Signal::Green);  // min green reached, but last gap only 1.5 s
  EXPECT_EQ(d.at(16.0, G_NT), Signal::Green);
  EXPECT_EQ(d.at(16.5, G_NT), Signal::Yellow);
  EXPECT_EQ(d.ctl().stats()[NS_T].gap_outs, 1u);
}

TEST(Actuated, OccupiedLoopHoldsGreenToMax) {
  Driver d(repo_config());
  d.event(20, "E_1", true);  // queued vehicle standing on the loop during EW green
  d.run_until(80);
  // EW_through green at 17.0 cannot gap out while the loop is occupied: max-out at 62.0.
  EXPECT_EQ(d.at(61.5, G_ET), Signal::Green);
  EXPECT_EQ(d.at(62.0, G_ET), Signal::Yellow);
  EXPECT_EQ(d.ctl().stats()[EW_T].max_outs, 1u);
}

TEST(Actuated, CallOnSkippablePhaseIsServedNextAndCleared) {
  Driver d(repo_config());
  d.pulse(5, "N_2");  // right-turner crosses the loop during NS_through green
  d.run_until(80);
  EXPECT_EQ(d.at(16.5, G_NR), Signal::Red);
  EXPECT_EQ(d.at(17.0, G_NR), Signal::Green);
  EXPECT_EQ(d.at(17.0, G_SR), Signal::Green);
  EXPECT_EQ(d.at(22.5, G_NR), Signal::Green);   // min green 6 s
  EXPECT_EQ(d.at(23.0, G_NR), Signal::Yellow);  // then gaps out (no NS_right vehicles)
  EXPECT_EQ(d.at(28.0, G_ET), Signal::Green);
  EXPECT_EQ(d.ctl().stats()[NS_R].served, 1u);
  EXPECT_EQ(d.first(Signal::Green, G_NR, 30), -1);  // call was cleared when served
}

TEST(Actuated, ActuationDuringOwnYellowPlacesANewCall) {
  Driver d(repo_config());
  d.pulse(5, "N_2");
  d.pulse(24, "S_2");  // NS_right is yellow 23.0-26.0
  d.run_until(80);
  // 28 EW_T green, gap-out 38, 43 NS_T green, gap-out 53, NS_right again at 58.
  EXPECT_EQ(d.first(Signal::Green, G_NR, 30), 58.0);
}

TEST(Actuated, VehicleStillOnLoopAtEndOfGreenKeepsCalling) {
  Driver d(repo_config());
  d.event(18, "N_2", true);  // right-turner stops on the loop and stays there
  d.run_until(120);
  // Call at 18 during EW green: EW_T 17-27 green, then EW_R skipped, NS_T 32-42, NS_right 47.
  // The loop stays occupied, so NS_right holds to max (20 s) and yellows at 67.
  EXPECT_EQ(d.at(47.0, G_NR), Signal::Green);
  EXPECT_EQ(d.at(67.0, G_NR), Signal::Yellow);
  // No new "on" edge arrives, but presence keeps the phase called: served again next cycle
  // (72 EW_T green, 82 gap-out, 87 NS_T green, 97 gap-out, 102 NS_right).
  EXPECT_EQ(d.first(Signal::Green, G_NR, 70), 102.0);
}

TEST(Actuated, RestsInGreenWithoutConflictingDemandThenGapsOutOnACall) {
  Driver d(config_with([](nlohmann::json& j) {
    for (auto& p : j["phases"]) p["recall"] = "none";
  }));
  d.pulse(10, "E_0");
  d.pulse(100, "N_0");
  d.run_until(110);
  EXPECT_EQ(d.at(9.5, G_ET), Signal::Red);  // no demand at all: rests in red
  EXPECT_EQ(d.at(10.0, G_ET), Signal::Green);
  EXPECT_EQ(d.at(99.5, G_ET), Signal::Green);  // well past max green, nobody else waiting
  EXPECT_EQ(d.at(100.0, G_ET), Signal::Yellow);
  // The green was no longer being extended (last vehicle at 10.5 s) when the call arrived, so
  // this is a gap-out in NEMA terms, although the green had run past max green while resting.
  EXPECT_EQ(d.ctl().stats()[EW_T].gap_outs, 1u);
  EXPECT_EQ(d.ctl().stats()[EW_T].max_outs, 0u);
  EXPECT_EQ(d.at(105.0, G_NT), Signal::Green);
  EXPECT_EQ(d.ctl().stats()[NS_T].skipped, 1u);
  EXPECT_EQ(d.ctl().stats()[NS_R].skipped, 1u);
}

TEST(Actuated, MaxRecallRunsToMaxGreenWithoutTraffic) {
  Driver d(config_with([](nlohmann::json& j) { j["phases"][0]["recall"] = "max"; }));
  d.run_until(50);
  EXPECT_EQ(d.at(46.5, G_NT), Signal::Green);
  EXPECT_EQ(d.at(47.0, G_NT), Signal::Yellow);
  EXPECT_EQ(d.ctl().stats()[NS_T].max_outs, 1u);
}

TEST(Actuated, UnknownDetectorIndexIsIgnored) {
  Driver d(repo_config());
  d.ctl().on_event({1000, 99, true});
  d.run_until(20);
  EXPECT_EQ(d.pipe().guard().refusals(), 0u);
}

TEST(Actuated, GapIsMeasuredFromLastVehicle) {
  Driver d(repo_config());
  d.pulse(4, "N_0");
  d.run_until(4);
  EXPECT_EQ(d.ctl().gap(NS_T, s(4)), 0);  // loop occupied
  d.run_until(6);
  EXPECT_EQ(d.ctl().gap(NS_T, s(6)), s(1.5));  // cleared at 4.5
}

TEST(Actuated, PipelineReportsEachTermination) {
  Driver d(repo_config());
  for (double t = 3; t < 100; t += 2) d.pulse(t, "N_0");
  d.run_until(80);
  std::vector<std::pair<double, Termination>> seen;
  for (const auto& r : d.history()) {
    if (r.termination) seen.emplace_back(static_cast<double>(r.t) / 1000.0, *r.termination);
  }
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0], std::make_pair(47.0, Termination::MaxOut));  // NS_through
  EXPECT_EQ(seen[1], std::make_pair(62.0, Termination::GapOut));  // EW_through, 52.0 + min green
  EXPECT_EQ(d.history()[static_cast<std::size_t>(s(62) / 500)].terminated_phase, EW_T);
}

TEST(Actuated, ActuationDuringOwnGreenExtendsButDoesNotCallAgain) {
  Driver d(repo_config());
  d.pulse(5, "N_2");
  d.pulse(18, "N_2");  // NS_right is green 17.0-23.0: these extend it, they are not new calls
  d.pulse(20, "S_2");
  d.run_until(80);
  EXPECT_EQ(d.at(22.5, G_NR), Signal::Green);
  EXPECT_EQ(d.at(23.0, G_NR), Signal::Yellow);  // min green 6 s and gap 2.5 s after 20.5
  EXPECT_EQ(d.first(Signal::Green, G_NR, 30), -1);
}

TEST(Actuated, RestingGreenStillExtendedAtACallMaxesOut) {
  Driver d(config_with([](nlohmann::json& j) {
    for (auto& p : j["phases"]) p["recall"] = "none";
  }));
  for (double t = 10; t < 120; t += 2) d.pulse(t, "E_0");  // steady EW traffic keeps extending
  d.pulse(100, "N_0");
  d.run_until(110);
  EXPECT_EQ(d.at(99.5, G_ET), Signal::Green);
  EXPECT_EQ(d.at(100.0, G_ET), Signal::Yellow);  // past max green and still extended: max-out
  EXPECT_EQ(d.ctl().stats()[EW_T].max_outs, 1u);
  EXPECT_EQ(d.ctl().stats()[EW_T].gap_outs, 0u);
}

TEST(Actuated, VehicleLeavingALoopPlacesNoCall) {
  Driver d(repo_config());
  d.pulse(5, "N_2");  // NS_right green 17.0-23.0 (min green), yellow 23.0-26.0
  // A right-turner reaches the call-only stop-bar loop during the green and clears it during
  // the yellow. Its arrival was served, and leaving a loop is not a new vehicle: no call.
  d.event(20, "N_2s", true);
  d.event(25, "N_2s", false);
  d.run_until(120);
  EXPECT_EQ(d.at(22.5, G_NR), Signal::Green);
  EXPECT_EQ(d.at(23.0, G_NR), Signal::Yellow);
  EXPECT_EQ(d.first(Signal::Green, G_NR, 30), -1);  // not served again
  EXPECT_EQ(d.ctl().stats()[NS_R].served, 1u);
}

TEST(Actuated, CallOnlyStopBarLoopCallsButDoesNotExtend) {
  Driver d(repo_config());
  d.event(5, "N_2s", true);  // a right-turner waiting at the stop line, on the loop until 30 s
  d.event(30, "N_2s", false);
  d.run_until(40);
  // Presence on the stop-bar loop calls NS_right (green at 17.0), but a call-only loop does not
  // extend it: it gaps out at min green although the loop is still occupied.
  EXPECT_EQ(d.at(17.0, G_NR), Signal::Green);
  EXPECT_EQ(d.at(22.5, G_NR), Signal::Green);
  EXPECT_EQ(d.at(23.0, G_NR), Signal::Yellow);
  EXPECT_EQ(d.ctl().stats()[NS_R].gap_outs, 1u);
}
