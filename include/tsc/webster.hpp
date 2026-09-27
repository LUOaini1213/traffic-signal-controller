#pragma once

#include <vector>

#include "tsc/config.hpp"

namespace tsc {

// Webster (1958) optimum cycle for pre-timed control:
//   C0 = (1.5 L + 5) / (1 - Y),   L = sum of lost times,  Y = sum of critical flow ratios y_i
// Effective green is split in proportion to y_i: g_i = (C - L) * y_i / Y.
// Lost time per phase is taken as yellow + all-red (start-up loss assumed equal to the use of
// the yellow), so displayed green = effective green.
struct WebsterInput {
  std::vector<double> flow_ratio;   // critical y_i = q / s for each phase
  std::vector<double> lost_time_s;  // per phase
  std::vector<double> min_green_s;  // per phase; greens are raised to at least this
  double min_cycle_s = 40.0;
  double max_cycle_s = 150.0;
  double tick_s = 0.5;  // greens are rounded to a multiple of this
};

struct WebsterPlan {
  double Y = 0.0;
  double L = 0.0;
  double optimal_cycle_s = 0.0;  // unclamped C0; +inf when Y >= 1
  bool oversaturated = false;    // Y >= 1: no finite cycle can serve the demand
  bool clamped = false;          // C0 fell outside [min_cycle, max_cycle]
  double cycle_s = 0.0;          // cycle actually used, after clamping, rounding and min greens
  std::vector<double> green_s;
};

// Throws std::invalid_argument on inconsistent input sizes or negative values.
WebsterPlan webster(const WebsterInput& in);

// Per-group demand -> per-phase critical flow ratio (max over the phase's groups of
// flow / (lanes * saturation flow)).
struct GroupDemand {
  double flow_vph = 0.0;
  double lanes = 1.0;
  double saturation_vphpl = 1800.0;
};
std::vector<double> phase_flow_ratios(const Config& cfg, const std::vector<GroupDemand>& demand);

// Convenience: plan for a config, in integer milliseconds ready for FixedTimeController.
WebsterPlan webster_for(const Config& cfg, const std::vector<GroupDemand>& demand,
                        double min_cycle_s = 40.0, double max_cycle_s = 150.0);
std::vector<TimeMs> greens_ms(const WebsterPlan& plan);

// Demand file: {"groups": {"<group>": {"flow_vph": 600, "lanes": 2, "saturation_vphpl": 1800}}}.
// Every config group must appear; unknown groups are rejected. Throws ConfigError.
std::vector<GroupDemand> parse_demand(const Config& cfg, const nlohmann::json& j);
nlohmann::json to_json(const WebsterPlan& plan, const Config& cfg);

}  // namespace tsc
