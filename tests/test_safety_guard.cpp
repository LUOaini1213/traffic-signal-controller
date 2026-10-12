#include <gtest/gtest.h>

#include <initializer_list>

#include "helpers.hpp"
#include "tsc/safety_guard.hpp"

using namespace tsc;
using namespace tsc::test;

namespace {
constexpr Signal R = Signal::Red, Y = Signal::Yellow, G = Signal::Green;

// All groups red except those listed, which show `a`.
SignalVector only(std::initializer_list<GroupId> groups, Signal a) {
  SignalVector v(8, R);
  for (GroupId g : groups) v[g] = a;
  return v;
}
const SignalVector kAllRed(8, R);

// Guard that has shown NS_through green from 2.0 s.
struct GuardAtNsGreen : ::testing::Test {
  Config cfg = repo_config();
  SafetyGuard guard{cfg, 0};
  void SetUp() override {
    ASSERT_EQ(guard.apply(0, kAllRed), kAllRed);
    ASSERT_EQ(guard.apply(s(2), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, G));
    ASSERT_FALSE(guard.failsafe());
  }
};
}  // namespace

TEST_F(GuardAtNsGreen, AcceptsALegalCycle) {
  EXPECT_EQ(guard.apply(s(12), only({G_NT, G_ST}, Y)), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(15), kAllRed), kAllRed);
  EXPECT_EQ(guard.apply(s(17), only({G_ET, G_WT}, G)), only({G_ET, G_WT}, G));
  EXPECT_EQ(guard.refusals(), 0u);
  EXPECT_FALSE(guard.failsafe());
}

TEST_F(GuardAtNsGreen, NonConflictingGroupsMayJoinAGreen) {
  // N_R does not conflict with N_T but does with S_T, so it may not join while S_T is green.
  EXPECT_TRUE(guard.check(s(13), only({G_NT, G_ST, G_NR}, G)).has_value());
  // S_T and N_T are compatible: showing one green and one yellow is fine as a vector.
  EXPECT_FALSE(guard.check(s(13), SignalVector{Y, G, R, R, R, R, R, R}).has_value());
}

TEST_F(GuardAtNsGreen, RefusesConflictingGreens) {
  auto cmd = only({G_NT, G_ST}, G);
  cmd[G_ET] = G;
  const auto& shown = guard.apply(s(20), cmd);
  EXPECT_TRUE(guard.failsafe());
  EXPECT_EQ(guard.refusals(), 1u);
  EXPECT_NE(guard.failsafe_reason().find("conflicting"), std::string::npos);
  EXPECT_EQ(shown[G_ET], R);
  EXPECT_EQ(shown[G_NT], Y);  // fail-safe still clears through yellow
}

TEST_F(GuardAtNsGreen, ConflictCheckCoversYellow) {
  auto cmd = only({G_NT, G_ST}, Y);
  cmd[G_WT] = G;
  EXPECT_TRUE(guard.check(s(20), cmd).has_value());
  auto both_yellow = only({G_NT}, Y);
  both_yellow[G_ER] = Y;
  EXPECT_TRUE(guard.check(s(20), both_yellow).has_value());
}

TEST_F(GuardAtNsGreen, RefusesGreenStraightToRed) {
  EXPECT_EQ(guard.apply(s(20), kAllRed)[G_NT], Y);
  EXPECT_TRUE(guard.failsafe());
  EXPECT_NE(guard.failsafe_reason().find("illegal transition G->r"), std::string::npos);
  EXPECT_EQ(guard.apply(s(22.5), kAllRed)[G_NT], Y);  // full yellow even in fail-safe
  EXPECT_EQ(guard.apply(s(23), kAllRed)[G_NT], R);
}

TEST_F(GuardAtNsGreen, RefusesYellowBeforeMinGreen) {
  EXPECT_TRUE(guard.check(s(11.5), only({G_NT, G_ST}, Y)).has_value());
  EXPECT_FALSE(guard.check(s(12), only({G_NT, G_ST}, Y)).has_value());
  guard.apply(s(11.5), only({G_NT, G_ST}, Y));
  EXPECT_TRUE(guard.failsafe());
  EXPECT_NE(guard.failsafe_reason().find("min green"), std::string::npos);
}

TEST_F(GuardAtNsGreen, RefusesShortYellow) {
  guard.apply(s(12), only({G_NT, G_ST}, Y));
  EXPECT_TRUE(guard.check(s(14.5), kAllRed).has_value());
  EXPECT_FALSE(guard.check(s(15), kAllRed).has_value());
  guard.apply(s(14.5), kAllRed);
  EXPECT_TRUE(guard.failsafe());
  EXPECT_NE(guard.failsafe_reason().find("yellow shorter"), std::string::npos);
}

TEST_F(GuardAtNsGreen, RefusesGreenBeforeAllRedClearance) {
  guard.apply(s(12), only({G_NT, G_ST}, Y));
  guard.apply(s(15), kAllRed);
  EXPECT_TRUE(guard.check(s(16.5), only({G_ET, G_WT}, G)).has_value());
  EXPECT_FALSE(guard.check(s(17), only({G_ET, G_WT}, G)).has_value());
  guard.apply(s(16.5), only({G_ET, G_WT}, G));
  EXPECT_TRUE(guard.failsafe());
  EXPECT_NE(guard.failsafe_reason().find("all-red"), std::string::npos);
  EXPECT_EQ(guard.displayed(), kAllRed);
}

TEST_F(GuardAtNsGreen, NonConflictingGroupNeedsNoClearance) {
  // N_R conflicts with S_T but not with N_T. Once S_T has cleared, N_R may go green while
  // N_T is still green.
  guard.apply(s(12), SignalVector{G, Y, R, R, R, R, R, R});
  guard.apply(s(15), SignalVector{G, R, R, R, R, R, R, R});
  EXPECT_TRUE(guard.check(s(16.5), SignalVector{G, R, G, R, R, R, R, R}).has_value());
  EXPECT_FALSE(guard.check(s(17), SignalVector{G, R, G, R, R, R, R, R}).has_value());
}

TEST_F(GuardAtNsGreen, RefusesYellowToGreenAndRedToYellow) {
  guard.apply(s(12), only({G_NT, G_ST}, Y));
  EXPECT_TRUE(guard.check(s(13), only({G_NT, G_ST}, G)).has_value());
  guard.apply(s(15), kAllRed);
  EXPECT_TRUE(guard.check(s(20), only({G_NR}, Y)).has_value());
  EXPECT_FALSE(guard.check(s(20), only({G_NR}, G)).has_value());
  EXPECT_EQ(guard.refusals(), 0u);
}

TEST_F(GuardAtNsGreen, RefusesMalformedCommands) {
  EXPECT_TRUE(guard.check(s(3), SignalVector(7, R)).has_value());
  EXPECT_TRUE(guard.check(s(2), only({G_NT, G_ST}, G)).has_value());   // time did not advance
  EXPECT_TRUE(guard.check(s(1), only({G_NT, G_ST}, G)).has_value());   // time went backwards
  EXPECT_FALSE(guard.check(s(2.5), only({G_NT, G_ST}, G)).has_value());
}

TEST_F(GuardAtNsGreen, CheckHasNoSideEffects) {
  (void)guard.check(s(3), kAllRed);
  EXPECT_EQ(guard.refusals(), 0u);
  EXPECT_FALSE(guard.failsafe());
  EXPECT_EQ(guard.displayed(), only({G_NT, G_ST}, G));
}

TEST_F(GuardAtNsGreen, FailSafeIsLatched) {
  guard.apply(s(5), kAllRed);  // refused
  ASSERT_TRUE(guard.failsafe());
  TimeMs t = s(5);
  for (int i = 0; i < 200; ++i) {
    t += 500;
    // A perfectly legal-looking command no longer gets through.
    const auto& shown = guard.apply(t, only({G_ET, G_WT}, G));
    ASSERT_EQ(shown[G_ET], R);
  }
  EXPECT_EQ(guard.displayed(), kAllRed);
  EXPECT_EQ(guard.refusals(), 1u);
}

TEST_F(GuardAtNsGreen, RequestedFailSafeClearsThroughYellow) {
  guard.request_failsafe(s(4), "test");
  EXPECT_TRUE(guard.failsafe());
  EXPECT_EQ(guard.failsafe_reason(), "test");
  EXPECT_EQ(guard.displayed(), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(6.5), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(7), only({G_NT, G_ST}, G)), kAllRed);
  EXPECT_EQ(guard.refusals(), 0u);  // a request is not a refusal
}

TEST_F(GuardAtNsGreen, BackwardApplyCannotBackdateFailSafeYellow) {
  ASSERT_EQ(guard.apply(s(20), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, G));
  EXPECT_EQ(guard.apply(s(1), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.failsafe_reason(), "time did not advance");
  EXPECT_EQ(guard.refusals(), 1u);
  EXPECT_EQ(guard.apply(s(20.5), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(22.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(23), kAllRed), kAllRed);
}

TEST_F(GuardAtNsGreen, DuplicateApplyStillRequiresAFullFailSafeYellow) {
  guard.apply(s(20), only({G_NT, G_ST}, G));
  EXPECT_EQ(guard.apply(s(20), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.failsafe_reason(), "time did not advance");
  EXPECT_EQ(guard.apply(s(20), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(22.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(23), kAllRed), kAllRed);
  EXPECT_EQ(guard.refusals(), 1u);
}

TEST_F(GuardAtNsGreen, BackwardRequestCannotBackdateFailSafeYellow) {
  guard.apply(s(20), only({G_NT, G_ST}, G));
  guard.request_failsafe(s(1), "lost feed");
  EXPECT_EQ(guard.displayed(), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(20.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(22.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(23), kAllRed), kAllRed);
  EXPECT_EQ(guard.failsafe_reason(), "lost feed");
  EXPECT_EQ(guard.refusals(), 0u);
}

TEST_F(GuardAtNsGreen, FailSafeClockNeverRegressesAfterRequestsOrApplies) {
  guard.request_failsafe(s(20), "lost feed");
  EXPECT_TRUE(guard.check(s(19), only({G_NT, G_ST}, Y)).has_value());
  // Pipeline requests fail-safe and applies its command at the same timestamp.
  EXPECT_EQ(guard.apply(s(20), only({G_NT, G_ST}, G)), only({G_NT, G_ST}, Y));
  guard.request_failsafe(s(21), "later reason");
  guard.request_failsafe(s(21), "duplicate reason");
  guard.request_failsafe(s(1), "stale reason");
  EXPECT_TRUE(guard.check(s(20.5), only({G_NT, G_ST}, Y)).has_value());
  EXPECT_EQ(guard.apply(s(1), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_TRUE(guard.check(s(20.5), only({G_NT, G_ST}, Y)).has_value());
  EXPECT_EQ(guard.apply(s(22.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(23), kAllRed), kAllRed);
  EXPECT_EQ(guard.failsafe_reason(), "lost feed");
  EXPECT_EQ(guard.refusals(), 0u);
}

TEST_F(GuardAtNsGreen, BackwardApplyPreservesAnExistingYellowStart) {
  guard.apply(s(12), only({G_NT, G_ST}, Y));
  guard.apply(s(14), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(1), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(14.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(15), kAllRed), kAllRed);
  EXPECT_EQ(guard.refusals(), 1u);
}

TEST_F(GuardAtNsGreen, BackwardRequestPreservesAnExistingYellowStart) {
  guard.apply(s(12), only({G_NT, G_ST}, Y));
  guard.apply(s(14), only({G_NT, G_ST}, Y));
  guard.request_failsafe(s(1), "lost feed");
  EXPECT_EQ(guard.displayed(), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(14.5), kAllRed), only({G_NT, G_ST}, Y));
  EXPECT_EQ(guard.apply(s(15), kAllRed), kAllRed);
  EXPECT_EQ(guard.refusals(), 0u);
}

TEST(SafetyGuard, ApplyBeforeStartCannotMoveItsClockBeforeStart) {
  SafetyGuard guard(repo_config(), s(10));
  EXPECT_EQ(guard.apply(s(1), kAllRed), kAllRed);
  EXPECT_TRUE(guard.failsafe());
  EXPECT_EQ(guard.failsafe_reason(), "time did not advance");
  EXPECT_TRUE(guard.check(s(9), kAllRed).has_value());
  EXPECT_EQ(guard.apply(s(1), only({G_NT, G_ST}, G)), kAllRed);
  EXPECT_TRUE(guard.check(s(9), kAllRed).has_value());
  EXPECT_EQ(guard.apply(s(20), only({G_NT, G_ST}, G)), kAllRed);
  EXPECT_EQ(guard.refusals(), 1u);
}

TEST(SafetyGuard, RequestBeforeStartCannotMoveItsClockBeforeStart) {
  SafetyGuard guard(repo_config(), s(10));
  guard.request_failsafe(s(1), "lost feed before start");
  EXPECT_EQ(guard.displayed(), kAllRed);
  EXPECT_TRUE(guard.failsafe());
  EXPECT_TRUE(guard.check(s(9), kAllRed).has_value());
  EXPECT_EQ(guard.apply(s(1), only({G_NT, G_ST}, G)), kAllRed);
  EXPECT_TRUE(guard.check(s(9), kAllRed).has_value());
  EXPECT_EQ(guard.apply(s(20), only({G_NT, G_ST}, G)), kAllRed);
  EXPECT_EQ(guard.refusals(), 0u);
}

TEST(SafetyGuard, RefusesConflictingGroupsReleasedInTheSameTick) {
  // Each group on its own is a legal red -> green after the start-up all-red; only the
  // pairwise conflict check can catch this.
  const Config cfg = repo_config();
  SafetyGuard guard(cfg, 0);
  auto cmd = only({G_NT, G_ST}, G);
  cmd[G_ET] = G;
  EXPECT_TRUE(guard.check(s(2), cmd).has_value());
  EXPECT_EQ(guard.apply(s(2), cmd), kAllRed);
  EXPECT_TRUE(guard.failsafe());
}

TEST(SafetyGuard, StartUpRequiresAllRedBeforeFirstGreen) {
  const Config cfg = repo_config();
  SafetyGuard guard(cfg, 0);
  EXPECT_TRUE(guard.check(s(1.5), only({G_NT, G_ST}, G)).has_value());
  EXPECT_FALSE(guard.check(s(2), only({G_NT, G_ST}, G)).has_value());
  EXPECT_TRUE(guard.check(-500, kAllRed).has_value());  // before the guard's start time
}
