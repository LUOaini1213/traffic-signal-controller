#include <gtest/gtest.h>

#include <cmath>

#include "helpers.hpp"
#include "tsc/webster.hpp"

using namespace tsc;
using namespace tsc::test;

TEST(Webster, TwoPhaseTextbookCase) {
  // y = 0.25 and 0.20, lost time 5 s per phase: L = 10, Y = 0.45,
  // C0 = (1.5*10 + 5) / (1 - 0.45) = 36.36 s, effective green 26.36 s split 0.25 : 0.20.
  WebsterInput in;
  in.flow_ratio = {0.25, 0.20};
  in.lost_time_s = {5, 5};
  in.min_green_s = {5, 5};
  in.min_cycle_s = 30;
  const auto plan = webster(in);
  EXPECT_NEAR(plan.Y, 0.45, 1e-12);
  EXPECT_NEAR(plan.L, 10.0, 1e-12);
  EXPECT_NEAR(plan.optimal_cycle_s, 36.3636, 1e-3);
  EXPECT_FALSE(plan.clamped);
  EXPECT_FALSE(plan.oversaturated);
  EXPECT_DOUBLE_EQ(plan.green_s[0], 14.5);  // 14.65 rounded to the 0.5 s tick
  EXPECT_DOUBLE_EQ(plan.green_s[1], 11.5);  // 11.72
  EXPECT_DOUBLE_EQ(plan.cycle_s, 36.0);
}

TEST(Webster, ClampsToCycleBoundsAndFlagsOversaturation) {
  WebsterInput in;
  in.flow_ratio = {0.05, 0.05};
  in.lost_time_s = {5, 5};
  in.min_green_s = {5, 5};
  in.min_cycle_s = 40;
  auto plan = webster(in);
  EXPECT_TRUE(plan.clamped);
  EXPECT_DOUBLE_EQ(plan.cycle_s, 40.0);
  EXPECT_DOUBLE_EQ(plan.green_s[0], 15.0);

  in.flow_ratio = {0.6, 0.5};
  plan = webster(in);
  EXPECT_TRUE(plan.oversaturated);
  EXPECT_TRUE(std::isinf(plan.optimal_cycle_s));
  EXPECT_DOUBLE_EQ(plan.cycle_s, 150.0);

  in.flow_ratio = {0.45, 0.45};  // Y = 0.9: C0 = 200 s, above the 150 s cap
  plan = webster(in);
  EXPECT_FALSE(plan.oversaturated);
  EXPECT_TRUE(plan.clamped);
  EXPECT_NEAR(plan.optimal_cycle_s, 200.0, 1e-9);
  EXPECT_DOUBLE_EQ(plan.cycle_s, 150.0);
}

TEST(Webster, RaisesGreensToMinGreen) {
  WebsterInput in;
  in.flow_ratio = {0.40, 0.01};
  in.lost_time_s = {5, 5};
  in.min_green_s = {10, 6};
  const auto plan = webster(in);
  EXPECT_DOUBLE_EQ(plan.green_s[1], 6.0);
  EXPECT_DOUBLE_EQ(plan.cycle_s, plan.L + plan.green_s[0] + plan.green_s[1]);
}

TEST(Webster, ZeroDemandSplitsEvenly) {
  WebsterInput in;
  in.flow_ratio = {0, 0, 0, 0};
  in.lost_time_s = {5, 5, 5, 5};
  in.min_green_s = {1, 1, 1, 1};
  const auto plan = webster(in);
  EXPECT_DOUBLE_EQ(plan.green_s[0], plan.green_s[3]);
  EXPECT_DOUBLE_EQ(plan.cycle_s, 40.0);
}

TEST(Webster, RejectsBadInput) {
  WebsterInput in;
  in.flow_ratio = {0.2, 0.2};
  in.lost_time_s = {5};
  in.min_green_s = {5, 5};
  EXPECT_THROW(webster(in), std::invalid_argument);
  in.lost_time_s = {5, -1};
  EXPECT_THROW(webster(in), std::invalid_argument);
  in.lost_time_s = {5, 5};
  in.flow_ratio = {0.2, std::nan("")};
  EXPECT_THROW(webster(in), std::invalid_argument);
}

TEST(Webster, PhaseFlowRatioIsTheCriticalGroup) {
  const Config cfg = repo_config();
  std::vector<GroupDemand> d(8, GroupDemand{100, 1, 1800});
  d[G_NT] = {900, 2, 1800};  // y = 0.25
  d[G_ST] = {720, 2, 1800};  // y = 0.20
  d[G_NR] = {90, 1, 1800};   // y = 0.05
  d[G_SR] = {180, 1, 1800};  // y = 0.10
  const auto y = phase_flow_ratios(cfg, d);
  EXPECT_NEAR(y[NS_T], 0.25, 1e-12);
  EXPECT_NEAR(y[NS_R], 0.10, 1e-12);
  EXPECT_NEAR(y[EW_T], 100.0 / 1800.0, 1e-12);

  const auto plan = webster_for(cfg, d);
  EXPECT_NEAR(plan.L, 20.0, 1e-12);  // 4 phases x (3 s yellow + 2 s all-red)
  const auto g = greens_ms(plan);
  ASSERT_EQ(g.size(), 4u);
  for (std::size_t p = 0; p < 4; ++p) {
    EXPECT_GE(g[p], cfg.phases[p].min_green);
    EXPECT_EQ(g[p] % cfg.tick, 0);
  }
  d.pop_back();
  EXPECT_THROW(phase_flow_ratios(cfg, d), std::invalid_argument);
}

TEST(Webster, ParsesDemandFiles) {
  const Config cfg = repo_config();
  nlohmann::json j;
  for (const auto& g : cfg.groups) j["groups"][g] = {{"flow_vph", 300}, {"lanes", 2}};
  j["groups"]["N_R"] = {{"flow_vph", 60}, {"lanes", 1}, {"saturation_vphpl", 1500}};
  const auto d = parse_demand(cfg, j);
  EXPECT_DOUBLE_EQ(d[G_NR].saturation_vphpl, 1500);
  EXPECT_DOUBLE_EQ(d[G_NT].saturation_vphpl, 1800);
  EXPECT_DOUBLE_EQ(d[G_NT].lanes, 2);

  auto missing = j;
  missing["groups"].erase("W_R");
  EXPECT_THROW(parse_demand(cfg, missing), ConfigError);
  auto unknown = j;
  unknown["groups"]["X"] = {{"flow_vph", 1}};
  EXPECT_THROW(parse_demand(cfg, unknown), ConfigError);
  auto negative = j;
  negative["groups"]["N_T"]["flow_vph"] = -5;
  EXPECT_THROW(parse_demand(cfg, negative), ConfigError);
}
