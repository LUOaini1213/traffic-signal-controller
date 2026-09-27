#include "tsc/webster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace tsc {

WebsterPlan webster(const WebsterInput& in) {
  const std::size_t n = in.flow_ratio.size();
  if (n == 0 || in.lost_time_s.size() != n || in.min_green_s.size() != n) {
    throw std::invalid_argument("webster: flow_ratio, lost_time_s and min_green_s must have the same non-zero size");
  }
  if (in.tick_s <= 0.0 || in.min_cycle_s <= 0.0 || in.max_cycle_s < in.min_cycle_s) {
    throw std::invalid_argument("webster: bad cycle bounds or tick");
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!(in.flow_ratio[i] >= 0.0) || !(in.lost_time_s[i] >= 0.0) || !(in.min_green_s[i] >= 0.0)) {
      throw std::invalid_argument("webster: negative or NaN input");
    }
  }

  WebsterPlan plan;
  plan.Y = std::accumulate(in.flow_ratio.begin(), in.flow_ratio.end(), 0.0);
  plan.L = std::accumulate(in.lost_time_s.begin(), in.lost_time_s.end(), 0.0);
  plan.oversaturated = plan.Y >= 1.0;
  plan.optimal_cycle_s = plan.oversaturated ? std::numeric_limits<double>::infinity()
                                            : (1.5 * plan.L + 5.0) / (1.0 - plan.Y);
  double cycle = std::clamp(plan.optimal_cycle_s, in.min_cycle_s, in.max_cycle_s);
  plan.clamped = cycle != plan.optimal_cycle_s;

  const double effective = std::max(0.0, cycle - plan.L);
  plan.green_s.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double share = plan.Y > 0.0 ? in.flow_ratio[i] / plan.Y : 1.0 / static_cast<double>(n);
    const double g = std::round(effective * share / in.tick_s) * in.tick_s;
    plan.green_s[i] = std::max(g, in.min_green_s[i]);
  }
  plan.cycle_s = plan.L + std::accumulate(plan.green_s.begin(), plan.green_s.end(), 0.0);
  return plan;
}

std::vector<double> phase_flow_ratios(const Config& cfg, const std::vector<GroupDemand>& demand) {
  if (demand.size() != cfg.groups.size()) {
    throw std::invalid_argument("phase_flow_ratios: need one demand entry per signal group");
  }
  std::vector<double> y(cfg.phases.size(), 0.0);
  for (std::size_t p = 0; p < cfg.phases.size(); ++p) {
    for (GroupId g : cfg.phases[p].groups) {
      const auto& d = demand[g];
      if (d.lanes <= 0.0 || d.saturation_vphpl <= 0.0 || d.flow_vph < 0.0) {
        throw std::invalid_argument("phase_flow_ratios: lanes and saturation flow must be > 0, flow >= 0");
      }
      y[p] = std::max(y[p], d.flow_vph / (d.lanes * d.saturation_vphpl));
    }
  }
  return y;
}

WebsterPlan webster_for(const Config& cfg, const std::vector<GroupDemand>& demand,
                        double min_cycle_s, double max_cycle_s) {
  WebsterInput in;
  in.flow_ratio = phase_flow_ratios(cfg, demand);
  for (const auto& p : cfg.phases) {
    in.lost_time_s.push_back(static_cast<double>(p.yellow + p.all_red) / 1000.0);
    in.min_green_s.push_back(static_cast<double>(p.min_green) / 1000.0);
  }
  in.min_cycle_s = min_cycle_s;
  in.max_cycle_s = max_cycle_s;
  in.tick_s = static_cast<double>(cfg.tick) / 1000.0;
  return webster(in);
}

std::vector<TimeMs> greens_ms(const WebsterPlan& plan) {
  std::vector<TimeMs> out;
  out.reserve(plan.green_s.size());
  for (double g : plan.green_s) out.push_back(static_cast<TimeMs>(std::llround(g * 1000.0)));
  return out;
}

std::vector<GroupDemand> parse_demand(const Config& cfg, const nlohmann::json& j) {
  std::vector<std::string> errors;
  std::vector<GroupDemand> out(cfg.groups.size());
  std::vector<bool> seen(cfg.groups.size(), false);
  const auto groups = j.find("groups");
  if (!j.is_object() || groups == j.end() || !groups->is_object()) {
    throw ConfigError({"demand: expected {\"groups\": {...}}"});
  }
  for (const auto& [name, v] : groups->items()) {
    const auto it = std::find(cfg.groups.begin(), cfg.groups.end(), name);
    if (it == cfg.groups.end()) {
      errors.push_back("demand: unknown group '" + name + "'");
      continue;
    }
    const auto g = static_cast<std::size_t>(it - cfg.groups.begin());
    seen[g] = true;
    auto num = [&](const char* key, double fallback) {
      const auto f = v.find(key);
      if (f == v.end()) return fallback;
      if (!f->is_number()) {
        errors.push_back("demand." + name + "." + key + ": expected a number");
        return fallback;
      }
      return f->get<double>();
    };
    out[g].flow_vph = num("flow_vph", -1.0);
    out[g].lanes = num("lanes", 1.0);
    out[g].saturation_vphpl = num("saturation_vphpl", 1800.0);
    if (out[g].flow_vph < 0.0) errors.push_back("demand." + name + ".flow_vph: missing or negative");
    if (out[g].lanes <= 0.0 || out[g].saturation_vphpl <= 0.0) {
      errors.push_back("demand." + name + ": lanes and saturation_vphpl must be > 0");
    }
  }
  for (std::size_t g = 0; g < seen.size(); ++g) {
    if (!seen[g]) errors.push_back("demand: no entry for group '" + cfg.groups[g] + "'");
  }
  if (!errors.empty()) throw ConfigError(std::move(errors));
  return out;
}

nlohmann::json to_json(const WebsterPlan& plan, const Config& cfg) {
  nlohmann::json j;
  j["Y"] = plan.Y;
  j["lost_time_s"] = plan.L;
  j["optimal_cycle_s"] = std::isfinite(plan.optimal_cycle_s) ? nlohmann::json(plan.optimal_cycle_s) : nlohmann::json(nullptr);
  j["oversaturated"] = plan.oversaturated;
  j["clamped"] = plan.clamped;
  j["cycle_s"] = plan.cycle_s;
  for (std::size_t p = 0; p < plan.green_s.size(); ++p) j["green_s"][cfg.phases[p].name] = plan.green_s[p];
  return j;
}

}  // namespace tsc
