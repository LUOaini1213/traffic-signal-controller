// tsc: command-line front end to the controller library.
//
//   tsc validate <config.json>
//   tsc webster  <config.json> <demand.json>
//   tsc replay   <config.json> <events.csv> [end_s]
//
// events.csv has a header line and rows "t_s,detector_id,on" (on = 1 or 0). replay runs the
// actuated controller behind the safety guard and prints every change of the displayed
// signals, then a JSON summary.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tsc/actuated.hpp"
#include "tsc/auditor.hpp"
#include "tsc/config.hpp"
#include "tsc/pipeline.hpp"
#include "tsc/webster.hpp"

namespace {

int usage() {
  std::cerr << "usage:\n"
               "  tsc validate <config.json>\n"
               "  tsc webster  <config.json> <demand.json>\n"
               "  tsc replay   <config.json> <events.csv> [end_s]\n";
  return 64;
}

int validate(const std::string& path) {
  const tsc::Config cfg = tsc::load_config(path);
  std::cout << "ok: '" << cfg.name << "': " << cfg.groups.size() << " signal groups, "
            << cfg.phases.size() << " phases, " << cfg.detectors.size() << " detectors\n";
  return 0;
}

int webster(const std::string& cfg_path, const std::string& demand_path) {
  const tsc::Config cfg = tsc::load_config(cfg_path);
  std::ifstream in(demand_path);
  if (!in) throw std::runtime_error("cannot open " + demand_path);
  const auto demand = tsc::parse_demand(cfg, nlohmann::json::parse(in));
  std::cout << tsc::to_json(tsc::webster_for(cfg, demand), cfg).dump(2) << "\n";
  return 0;
}

std::string trim(const std::string& field) {
  const auto first = field.find_first_not_of(" \t");
  if (first == std::string::npos) return {};
  return field.substr(first, field.find_last_not_of(" \t") - first + 1);
}

// This deliberately small format has exactly three unquoted fields. Accept Windows
// exports, but never reinterpret malformed input as an OFF event or a partial number.
std::array<std::string, 3> csv_fields(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const auto first = line.find(',');
  const auto second = first == std::string::npos ? std::string::npos : line.find(',', first + 1);
  if (second == std::string::npos || line.find(',', second + 1) != std::string::npos) {
    throw std::runtime_error("expected exactly three fields: t_s,detector_id,on");
  }
  return {trim(line.substr(0, first)), trim(line.substr(first + 1, second - first - 1)),
          trim(line.substr(second + 1))};
}

tsc::TimeMs parse_time(const std::string& value, const std::string& field) {
  const auto input = trim(value);
  const std::string error = field + ": expected finite, non-negative seconds within the millisecond time range";
  long double seconds = 0;
  std::size_t used = 0;
  try {
    seconds = std::stold(input, &used);
  } catch (const std::exception&) {
    throw std::runtime_error(error);
  }
  if (used != input.size() || !std::isfinite(seconds) || seconds < 0) throw std::runtime_error(error);
  const long double milliseconds = std::round(seconds * 1000.0L);
  // Use the exclusive power-of-two bound: converting INT64_MAX to a floating type can
  // round it up to 2^63 on platforms where long double has no extra precision.
  const long double exclusive_limit = -static_cast<long double>(std::numeric_limits<tsc::TimeMs>::min());
  if (!std::isfinite(milliseconds) || milliseconds >= exclusive_limit) throw std::runtime_error(error);
  return static_cast<tsc::TimeMs>(milliseconds);
}

int replay(const std::string& cfg_path, const std::string& events_path, tsc::TimeMs end) {
  const tsc::Config cfg = tsc::load_config(cfg_path);
  std::ifstream in(events_path);
  if (!in) throw std::runtime_error("cannot open " + events_path);
  std::vector<tsc::DetectorEvent> events;
  std::string line;
  try {
    if (!std::getline(in, line)) throw std::runtime_error("missing t_s,detector_id,on header");
    if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);  // optional UTF-8 BOM
    if (csv_fields(line) != std::array<std::string, 3>{"t_s", "detector_id", "on"}) {
      throw std::runtime_error("expected t_s,detector_id,on header");
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(events_path + ":1: " + e.what());
  }
  for (int row = 2; std::getline(in, line); ++row) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    try {
      const auto fields = csv_fields(line);
      if (fields[2] != "0" && fields[2] != "1") throw std::runtime_error("on must be 0 or 1");
      events.push_back({parse_time(fields[0], "t_s"), cfg.detector_index(fields[1]), fields[2] == "1"});
    } catch (const std::exception& e) {
      throw std::runtime_error(events_path + ":" + std::to_string(row) + ": " + e.what());
    }
  }
  if (in.bad()) throw std::runtime_error("cannot read " + events_path);
  std::stable_sort(events.begin(), events.end(),
                   [](const auto& a, const auto& b) { return a.t < b.t; });

  tsc::Pipeline pipe(cfg, std::make_unique<tsc::ActuatedController>(cfg, 0), 0);
  tsc::InvariantAuditor audit(cfg, 0);
  std::size_t next = 0;
  std::string last;
  std::cout << "t_s,signals,failsafe\n";
  for (tsc::TimeMs t = 0;;) {
    while (next < events.size() && events[next].t <= t) pipe.on_event(events[next++]);
    const auto rec = pipe.tick(t);
    audit.observe(t, rec.displayed, rec.failsafe);
    const std::string s = tsc::to_string(rec.displayed);
    if (s != last) std::cout << static_cast<double>(t) / 1000.0 << "," << s << "," << rec.failsafe << "\n";
    last = s;
    if (end - t < cfg.tick) break;  // do not overflow when the end is near TimeMs' limit
    t += cfg.tick;
  }
  nlohmann::json summary;
  const auto& stats = pipe.controller().stats();
  for (std::size_t p = 0; p < cfg.phases.size(); ++p) {
    summary["phases"][cfg.phases[p].name] = {{"served", stats[p].served},
                                             {"gap_outs", stats[p].gap_outs},
                                             {"max_outs", stats[p].max_outs},
                                             {"skipped", stats[p].skipped}};
  }
  summary["guard_refusals"] = pipe.guard().refusals();
  summary["audit_violations"] = audit.violations();
  summary["failsafe"] = pipe.guard().failsafe();
  std::cerr << summary.dump(2) << "\n";
  return audit.violations() == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  try {
    if (args.size() == 2 && args[0] == "validate") return validate(args[1]);
    if (args.size() == 3 && args[0] == "webster") return webster(args[1], args[2]);
    if ((args.size() == 3 || args.size() == 4) && args[0] == "replay") {
      return replay(args[1], args[2], parse_time(args.size() == 4 ? args[3] : "3600", "end_s"));
    }
  } catch (const tsc::ConfigError& e) {
    std::cerr << e.what() << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return usage();
}
