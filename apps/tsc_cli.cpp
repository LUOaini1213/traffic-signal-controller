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
#include <cmath>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
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

int replay(const std::string& cfg_path, const std::string& events_path, double end_s) {
  const tsc::Config cfg = tsc::load_config(cfg_path);
  std::ifstream in(events_path);
  if (!in) throw std::runtime_error("cannot open " + events_path);
  std::vector<tsc::DetectorEvent> events;
  std::string line;
  std::getline(in, line);  // header
  for (int row = 2; std::getline(in, line); ++row) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string t, det, on;
    if (!std::getline(ss, t, ',') || !std::getline(ss, det, ',') || !std::getline(ss, on, ',')) {
      throw std::runtime_error(events_path + ":" + std::to_string(row) + ": expected t_s,detector_id,on");
    }
    events.push_back({static_cast<tsc::TimeMs>(std::llround(std::stod(t) * 1000.0)),
                      cfg.detector_index(det), on == "1"});
  }
  std::stable_sort(events.begin(), events.end(),
                   [](const auto& a, const auto& b) { return a.t < b.t; });

  tsc::Pipeline pipe(cfg, std::make_unique<tsc::ActuatedController>(cfg, 0), 0);
  tsc::InvariantAuditor audit(cfg, 0);
  const auto end = static_cast<tsc::TimeMs>(std::llround(end_s * 1000.0));
  std::size_t next = 0;
  std::string last;
  std::cout << "t_s,signals,failsafe\n";
  for (tsc::TimeMs t = 0; t <= end; t += cfg.tick) {
    while (next < events.size() && events[next].t <= t) pipe.on_event(events[next++]);
    const auto rec = pipe.tick(t);
    audit.observe(t, rec.displayed, rec.failsafe);
    const std::string s = tsc::to_string(rec.displayed);
    if (s != last) std::cout << static_cast<double>(t) / 1000.0 << "," << s << "," << rec.failsafe << "\n";
    last = s;
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
      return replay(args[1], args[2], args.size() == 4 ? std::stod(args[3]) : 3600.0);
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
