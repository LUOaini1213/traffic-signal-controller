#include <gtest/gtest.h>

#include "helpers.hpp"
#include "tsc/auditor.hpp"

using namespace tsc;
using namespace tsc::test;

namespace {
constexpr Signal R = Signal::Red, Y = Signal::Yellow, G = Signal::Green;
SignalVector only(std::initializer_list<GroupId> groups, Signal a) {
  SignalVector v(8, R);
  for (GroupId g : groups) v[g] = a;
  return v;
}

struct Audit : ::testing::Test {
  Config cfg = repo_config();
  InvariantAuditor audit{cfg, 0};
  // A legal start: NS_through green at 2.0 s.
  void SetUp() override {
    audit.observe(0, SignalVector(8, R));
    audit.observe(s(2), only({G_NT, G_ST}, G));
    ASSERT_EQ(audit.violations(), 0u);
  }
};
}  // namespace

TEST_F(Audit, CleanCycleHasNoViolations) {
  audit.observe(s(12), only({G_NT, G_ST}, Y));
  audit.observe(s(15), SignalVector(8, R));
  audit.observe(s(17), only({G_NR, G_SR}, G));
  audit.observe(s(23), only({G_NR, G_SR}, Y));
  audit.observe(s(26), SignalVector(8, R));
  audit.observe(s(28), only({G_ET, G_WT}, G));
  EXPECT_EQ(audit.violations(), 0u) << (audit.messages().empty() ? "" : audit.messages()[0]);
}

TEST_F(Audit, DetectsConflictingReleases) {
  auto v = only({G_NT, G_ST}, G);
  v[G_WT] = G;
  audit.observe(s(20), v);
  EXPECT_GE(audit.violations(), 1u);
  EXPECT_NE(audit.messages()[0].find("conflict"), std::string::npos);
}

TEST_F(Audit, DetectsGreenStraightToRed) {
  audit.observe(s(20), SignalVector(8, R));
  EXPECT_EQ(audit.violations(), 2u);  // N_T and S_T
}

TEST_F(Audit, DetectsShortMinGreenUnlessFailSafe) {
  InvariantAuditor other(cfg, 0);
  other.observe(s(2), only({G_NT, G_ST}, G));
  other.observe(s(8), only({G_NT, G_ST}, Y), /*failsafe=*/true);
  EXPECT_EQ(other.violations(), 0u);
  audit.observe(s(8), only({G_NT, G_ST}, Y));
  EXPECT_EQ(audit.violations(), 2u);
}

TEST_F(Audit, DetectsShortYellow) {
  audit.observe(s(12), only({G_NT, G_ST}, Y));
  audit.observe(s(14.5), SignalVector(8, R));
  EXPECT_EQ(audit.violations(), 2u);
}

TEST_F(Audit, DetectsMissingAllRed) {
  audit.observe(s(12), only({G_NT, G_ST}, Y));
  audit.observe(s(15), SignalVector(8, R));
  audit.observe(s(16.5), only({G_ET}, G));
  EXPECT_EQ(audit.violations(), 2u);  // E_T released before N_T and before S_T cleared
}

TEST_F(Audit, DetectsIllegalChanges) {
  audit.observe(s(12), only({G_NT, G_ST}, Y));
  audit.observe(s(13), only({G_NT, G_ST}, G));  // yellow back to green
  EXPECT_EQ(audit.violations(), 2u);
  InvariantAuditor other(cfg, 0);
  other.observe(s(5), only({G_ER}, Y));  // red to yellow
  EXPECT_EQ(other.violations(), 1u);
  other.observe(s(6), SignalVector(3, R));
  EXPECT_EQ(other.violations(), 2u);
}
