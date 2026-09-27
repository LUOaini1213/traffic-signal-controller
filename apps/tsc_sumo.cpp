// tsc_sumo: runs one SUMO simulation of the junction under one controller.
//
// SUMO is linked in-process through libsumo (the C++ TraCI API shipped with the sumo package),
// so the C++ controller drives the simulation directly with no Python or socket in the loop.
//
//   --controller actuated   this repository's actuated controller (through the safety guard)
//   --controller fixed      this repository's fixed-time controller, Webster plan from --demand
//   --controller sumo       SUMO's built-in "actuated" tlLogic (reference), loaded from --program
//
// Detector samples are read on the simulation thread (libsumo is not thread-safe) and handed
// to one producer thread per approach, which turns them into on/off events for the controller
// thread (tsc::ControllerRuntime). Every displayed state is audited by an InvariantAuditor,
// including SUMO's own programme.
#include <libsumo/libsumo.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tsc/actuated.hpp"
#include "tsc/auditor.hpp"
#include "tsc/blocking_queue.hpp"
#include "tsc/config.hpp"
#include "tsc/fixed_time.hpp"
#include "tsc/runtime.hpp"
#include "tsc/webster.hpp"

namespace {

using tsc::TimeMs;

struct Fault {
  std::string detector;
  bool stuck_on = false;
  TimeMs from = 0;
};

struct Args {
  std::string config, net, routes, detectors, program, demand, tripinfo, summary;
  std::string controller = "actuated";
  int seed = 1;
  double warmup_s = 300, demand_end_s = 3600, end_s = 7200;
  std::vector<Fault> faults;
  bool faults_disabled = false;
};

// Thrown for a bad command line; main() then exits with status 64.
struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

[[noreturn]] void usage(const std::string& why) {
  std::cerr << "tsc_sumo: " << why << "\n"
            << "usage: tsc_sumo --config C --net N --routes R --detectors D --summary OUT.json\n"
               "                [--controller actuated|fixed|sumo] [--demand DEMAND.json] [--program SUMO_TLS.add.xml]\n"
               "                [--seed N] [--tripinfo FILE] [--warmup S] [--demand-end S] [--end S]\n"
               "                [--fault DET:stuck-on|stuck-off:T_S]... [--no-fault-handling]\n";
  throw UsageError(why);
}

Args parse_args(int argc, char** argv) {
  Args a;
  std::vector<std::string> v(argv + 1, argv + argc);
  for (std::size_t i = 0; i < v.size(); ++i) {
    auto need = [&]() -> const std::string& {
      if (i + 1 >= v.size()) usage("missing value after " + v[i]);
      return v[++i];
    };
    const std::string& k = v[i];
    if (k == "--config") a.config = need();
    else if (k == "--net") a.net = need();
    else if (k == "--routes") a.routes = need();
    else if (k == "--detectors") a.detectors = need();
    else if (k == "--program") a.program = need();
    else if (k == "--demand") a.demand = need();
    else if (k == "--tripinfo") a.tripinfo = need();
    else if (k == "--summary") a.summary = need();
    else if (k == "--controller") a.controller = need();
    else if (k == "--seed") a.seed = std::stoi(need());
    else if (k == "--warmup") a.warmup_s = std::stod(need());
    else if (k == "--demand-end") a.demand_end_s = std::stod(need());
    else if (k == "--end") a.end_s = std::stod(need());
    else if (k == "--no-fault-handling") a.faults_disabled = true;
    else if (k == "--fault") {
      std::stringstream ss(need());
      std::string det, kind, t;
      std::getline(ss, det, ':');
      std::getline(ss, kind, ':');
      std::getline(ss, t, ':');
      if (det.empty() || (kind != "stuck-on" && kind != "stuck-off") || t.empty()) usage("bad --fault");
      a.faults.push_back({det, kind == "stuck-on", static_cast<TimeMs>(std::llround(std::stod(t) * 1000))});
    } else {
      usage("unknown argument " + k);
    }
  }
  if (a.config.empty() || a.net.empty() || a.routes.empty() || a.detectors.empty() || a.summary.empty()) {
    usage("--config, --net, --routes, --detectors and --summary are required");
  }
  if (a.controller != "actuated" && a.controller != "fixed" && a.controller != "sumo") usage("bad --controller");
  if (a.controller == "fixed" && a.demand.empty()) usage("--controller fixed needs --demand");
  if (a.controller == "sumo" && a.program.empty()) usage("--controller sumo needs --program");
  return a;
}

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double pos = q * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = static_cast<std::size_t>(std::ceil(pos));
  return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<double>(lo));
}

tsc::Signal aspect(char c) {
  switch (c) {
    case 'G': case 'g': return tsc::Signal::Green;
    case 'y': case 'Y': return tsc::Signal::Yellow;
    default: return tsc::Signal::Red;
  }
}

nlohmann::json json_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  return nlohmann::json::parse(in);
}

struct Sample {
  TimeMs t = 0;
  std::vector<bool> occupied;
};

int run(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  const auto wall_start = std::chrono::steady_clock::now();
  tsc::Config cfg;
  try {
    auto j = json_file(args.config);
    if (args.faults_disabled) j["faults"]["enabled"] = false;
    cfg = tsc::parse_config(j);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
  const auto& sumo = cfg.sumo;
  const std::string tls = sumo.at("tls_id");
  const auto ms = [](double s) { return static_cast<TimeMs>(std::llround(s * 1000.0)); };
  const TimeMs warmup = ms(args.warmup_s), demand_end = ms(args.demand_end_s), end = ms(args.end_s);
  auto in_window = [&](TimeMs t) { return t >= warmup && t < demand_end; };

  std::vector<std::string> cmd = {"sumo", "-n", args.net, "-r", args.routes,
                                  "-a", args.detectors + (args.controller == "sumo" ? "," + args.program : ""),
                                  "--step-length", std::to_string(static_cast<double>(cfg.tick) / 1000.0),
                                  "--seed", std::to_string(args.seed), "--begin", "0",
                                  "--end", std::to_string(args.end_s),
                                  "--time-to-teleport", "-1",  // stuck vehicles stay and keep counting delay
                                  "--no-step-log", "true", "--no-warnings", "true", "--duration-log.disable", "true"};
  if (!args.tripinfo.empty()) {
    cmd.insert(cmd.end(), {"--tripinfo-output", args.tripinfo, "--tripinfo-output.write-unfinished", "true"});
  }
  libsumo::Simulation::start(cmd);

  // Link index -> signal group, from the movements declared in the config.
  std::vector<tsc::GroupId> link_group;
  for (const auto& links : libsumo::TrafficLight::getControlledLinks(tls)) {
    const std::string from = libsumo::Lane::getEdgeID(links.at(0).fromLane);
    const std::string to = libsumo::Lane::getEdgeID(links.at(0).toLane);
    std::optional<tsc::GroupId> g;
    for (const auto& [group, pairs] : sumo.at("movements").items()) {
      for (const auto& pr : pairs) {
        if (pr.at(0) == from && pr.at(1) == to) g = cfg.group_index(group);
      }
    }
    if (!g) {
      std::cerr << "link " << from << "->" << to << " is not mapped to a signal group\n";
      return 2;
    }
    link_group.push_back(*g);
  }
  auto groups_from_state = [&](const std::string& st, bool& consistent) {
    tsc::SignalVector v(cfg.groups.size(), tsc::Signal::Red);
    std::vector<bool> seen(cfg.groups.size(), false);
    consistent = st.size() == link_group.size();
    for (std::size_t i = 0; i < link_group.size() && i < st.size(); ++i) {
      const tsc::Signal a = aspect(st[i]);
      if (seen[link_group[i]] && v[link_group[i]] != a) consistent = false;
      v[link_group[i]] = a;
      seen[link_group[i]] = true;
    }
    return v;
  };
  auto state_from_groups = [&](const tsc::SignalVector& v) {
    std::string st;
    for (tsc::GroupId g : link_group) st.push_back(tsc::to_char(v[g]));
    return st;
  };

  // Detector bookkeeping: SUMO loop id == config detector id; one producer per approach.
  std::vector<std::string> approach_names;
  std::map<std::string, std::string> edge_approach;
  for (const auto& [name, edge] : sumo.at("approaches").items()) {
    approach_names.push_back(name);
    edge_approach[edge.get<std::string>()] = name;
  }
  std::vector<std::vector<tsc::DetectorId>> owned(approach_names.size());
  for (tsc::DetectorId d = 0; d < cfg.detectors.size(); ++d) {
    const std::string lane = sumo.at("detector_lanes").at(cfg.detectors[d].id);
    const std::string approach = edge_approach.at(libsumo::Lane::getEdgeID(lane));
    const auto idx = static_cast<std::size_t>(
        std::find(approach_names.begin(), approach_names.end(), approach) - approach_names.begin());
    owned[idx].push_back(d);
  }
  std::vector<std::optional<Fault>> fault_of(cfg.detectors.size());
  for (const auto& f : args.faults) fault_of.at(cfg.detector_index(f.detector)) = f;
  auto occupied = [&](tsc::DetectorId d, TimeMs t) {
    if (const auto& f = fault_of[d]; f && t >= f->from) return f->stuck_on;
    const std::string& id = cfg.detectors[d].id;
    return libsumo::InductionLoop::getLastStepVehicleNumber(id) > 0 ||
           libsumo::InductionLoop::getLastStepOccupancy(id) > 0.0;
  };

  // Controller (ours) or SUMO's programme.
  nlohmann::json plan_json;
  std::unique_ptr<tsc::SignalController> controller;
  if (args.controller == "actuated") {
    controller = std::make_unique<tsc::ActuatedController>(cfg, 0);
  } else if (args.controller == "fixed") {
    const auto demand = tsc::parse_demand(cfg, json_file(args.demand));
    const auto plan = tsc::webster_for(cfg, demand);
    plan_json = tsc::to_json(plan, cfg);
    controller = std::make_unique<tsc::FixedTimeController>(cfg, 0, tsc::greens_ms(plan));
  } else {
    // Start SUMO's programme in its last phase (an all-red whose successor is phase 0), so it
    // gets the same start-up clearance as our controllers instead of opening with a green.
    libsumo::TrafficLight::setProgram(tls, "sumo_actuated");
    const auto& logic = libsumo::TrafficLight::getAllProgramLogics(tls);
    for (const auto& l : logic) {
      if (l.programID == "sumo_actuated") libsumo::TrafficLight::setPhase(tls, static_cast<int>(l.phases.size()) - 1);
    }
  }

  std::unique_ptr<tsc::ControllerRuntime> rt;
  std::vector<std::unique_ptr<tsc::BlockingQueue<Sample>>> inbox;
  std::vector<std::thread> producers;
  if (controller) {
    rt = std::make_unique<tsc::ControllerRuntime>(cfg, std::move(controller), 0, owned);
    // Every queue exists before the first producer thread starts: the threads read `inbox`,
    // so it must not be resized (reallocated) while they run.
    for (std::size_t p = 0; p < owned.size(); ++p) inbox.push_back(std::make_unique<tsc::BlockingQueue<Sample>>());
    for (std::size_t p = 0; p < owned.size(); ++p) {
      producers.emplace_back([&, p, handle = rt->producer(p)]() mutable {
        std::vector<bool> last(owned[p].size(), false);
        while (auto smp = inbox[p]->pop()) {
          for (std::size_t k = 0; k < owned[p].size(); ++k) {
            if (smp->occupied[k] != last[k]) {
              handle.emit({smp->t, owned[p][k], smp->occupied[k]});
              last[k] = smp->occupied[k];
            }
          }
          handle.advance(smp->t);
        }
        handle.close();
      });
    }
  }
  // Closes the sample queues and joins the producer threads (a joinable std::thread must not be
  // destroyed). Safe to call more than once.
  auto stop_producers = [&]() {
    for (auto& q : inbox) q->close();
    for (auto& th : producers) {
      if (th.joinable()) th.join();
    }
  };
  auto publish_samples = [&](TimeMs t) {
    for (std::size_t p = 0; p < owned.size(); ++p) {
      Sample smp{t, {}};
      for (tsc::DetectorId d : owned[p]) smp.occupied.push_back(occupied(d, t));
      inbox[p]->push(std::move(smp));
    }
  };

  // Measurement state.
  tsc::InvariantAuditor audit(cfg, 0);
  std::size_t inconsistent_states = 0;
  std::vector<double> queue_worst, queue_total;
  std::map<std::string, std::vector<double>> queue_by_approach;
  long arrived_in_window = 0;
  std::vector<nlohmann::json> terminations(cfg.phases.size(), {{"gap_out", 0}, {"max_out", 0}, {"fixed", 0}});
  const TimeMs bin = ms(300);
  std::vector<std::vector<int>> served_by_bin(cfg.phases.size(), std::vector<int>(static_cast<std::size_t>(end / bin) + 1, 0));
  std::optional<tsc::TickRecord> last_rec;

  // SUMO programme bookkeeping (reference controller only).
  std::vector<libsumo::TraCILogic> logics;
  int sumo_phase = -1;
  TimeMs sumo_phase_start = 0;
  if (!rt) {
    for (const auto& l : libsumo::TrafficLight::getAllProgramLogics(tls)) {
      if (l.programID == "sumo_actuated") logics.push_back(l);
    }
    if (logics.empty()) {
      std::cerr << "programme 'sumo_actuated' not found in " << args.program << "\n";
      return 2;
    }
    sumo_phase = libsumo::TrafficLight::getPhase(tls);
  }
  auto phase_of_sumo_green = [&](int idx) -> std::optional<tsc::PhaseId> {
    const auto& st = logics.at(0).phases.at(static_cast<std::size_t>(idx))->state;
    for (std::size_t i = 0; i < st.size(); ++i) {
      if (st[i] == 'G') return cfg.group_phase[link_group[i]];
    }
    return std::nullopt;
  };

  TimeMs t = 0;
  if (rt) publish_samples(0);
  while (true) {
    if (rt) {
      auto rec = rt->next_output();
      if (!rec) {
        std::cerr << "controller runtime stopped unexpectedly\n";
        stop_producers();
        rt->stop();
        rt->join();
        libsumo::Simulation::close();
        return 3;
      }
      libsumo::TrafficLight::setRedYellowGreenState(tls, state_from_groups(rec->displayed));
      if (rec->termination && in_window(t)) {
        const char* key = *rec->termination == tsc::Termination::GapOut   ? "gap_out"
                          : *rec->termination == tsc::Termination::MaxOut ? "max_out"
                                                                          : "fixed";
        terminations[rec->terminated_phase][key] = terminations[rec->terminated_phase][key].get<int>() + 1;
      }
      if (last_rec) {
        for (tsc::PhaseId p = 0; p < cfg.phases.size(); ++p) {
          const tsc::GroupId g = cfg.phases[p].groups[0];
          if (rec->displayed[g] == tsc::Signal::Green && last_rec->displayed[g] != tsc::Signal::Green) {
            ++served_by_bin[p][static_cast<std::size_t>(t / bin)];
          }
        }
      }
      last_rec = std::move(rec);
    }

    libsumo::Simulation::step();
    t += cfg.tick;

    bool consistent = true;
    const auto shown = groups_from_state(libsumo::TrafficLight::getRedYellowGreenState(tls), consistent);
    if (!consistent) ++inconsistent_states;
    audit.observe(t, shown, last_rec && last_rec->failsafe);

    if (!rt) {  // SUMO's own programme: classify each green termination
      const int now_phase = libsumo::TrafficLight::getPhase(tls);
      if (now_phase != sumo_phase) {
        if (const auto p = phase_of_sumo_green(sumo_phase); p && in_window(t)) {
          const double max_dur = logics.at(0).phases.at(static_cast<std::size_t>(sumo_phase))->maxDur;
          const bool maxed = static_cast<double>(t - sumo_phase_start) >= max_dur * 1000.0 - 1.0;
          const char* key = maxed ? "max_out" : "gap_out";
          terminations[*p][key] = terminations[*p][key].get<int>() + 1;
        }
        if (const auto p = phase_of_sumo_green(now_phase)) ++served_by_bin[*p][static_cast<std::size_t>(t / bin)];
        sumo_phase = now_phase;
        sumo_phase_start = t;
      }
    }

    if (in_window(t)) {
      arrived_in_window += libsumo::Simulation::getArrivedNumber();
      double worst = 0, total = 0;
      for (const auto& [name, edge] : sumo.at("approaches").items()) {
        const double q = libsumo::Edge::getLastStepHaltingNumber(edge.get<std::string>());
        queue_by_approach[name].push_back(q);
        worst = std::max(worst, q);
        total += q;
      }
      queue_worst.push_back(worst);
      queue_total.push_back(total);
    }

    const bool drained = t >= demand_end && libsumo::Simulation::getMinExpectedNumber() == 0;
    if (t >= end || drained) break;
    if (rt) publish_samples(t);
  }

  const double sim_end_s = libsumo::Simulation::getTime();
  const auto still_running = libsumo::Simulation::getMinExpectedNumber();
  libsumo::Simulation::close();  // flushes tripinfo

  nlohmann::json out;
  out["controller"] = args.controller;
  out["seed"] = args.seed;
  out["sim_end_s"] = sim_end_s;
  out["unfinished_vehicles"] = still_running;
  out["window_s"] = {args.warmup_s, args.demand_end_s};
  out["throughput_vph"] = static_cast<double>(arrived_in_window) * 3600.0 / (args.demand_end_s - args.warmup_s);
  out["queue_p95_worst_approach_veh"] = percentile(queue_worst, 0.95);
  out["queue_p95_total_veh"] = percentile(queue_total, 0.95);
  for (const auto& [name, q] : queue_by_approach) out["queue_p95_by_approach_veh"][name] = percentile(q, 0.95);
  out["invariant_violations"] = audit.violations();
  out["invariant_messages"] = audit.messages();
  out["inconsistent_group_states"] = inconsistent_states;
  for (tsc::PhaseId p = 0; p < cfg.phases.size(); ++p) {
    out["terminations"][cfg.phases[p].name] = terminations[p];
    out["greens_started_per_300s"][cfg.phases[p].name] = served_by_bin[p];
  }
  if (!plan_json.is_null()) out["webster"] = plan_json;
  out["faults_injected"] = nlohmann::json::array();
  for (const auto& f : args.faults) {
    out["faults_injected"].push_back({{"detector", f.detector}, {"kind", f.stuck_on ? "stuck-on" : "stuck-off"},
                                      {"from_s", static_cast<double>(f.from) / 1000.0}});
  }

  if (rt) {
    stop_producers();
    while (rt->next_output()) {
    }
    const tsc::Pipeline& pipe = rt->join();
    out["guard_refusals"] = pipe.guard().refusals();
    out["failsafe"] = pipe.guard().failsafe();
    out["runtime"] = {{"events", rt->stats().events}, {"late_events", rt->stats().late_events},
                      {"ticks", rt->stats().ticks}};
    out["faults_declared"] = nlohmann::json::array();
    if (const auto* act = dynamic_cast<const tsc::ActuatedController*>(&pipe.controller())) {
      for (const auto& f : act->monitor().faults()) {
        // Feed-lost faults at shutdown are an artefact of closing the producers.
        if (f.kind == tsc::FaultKind::FeedLost) continue;
        out["faults_declared"].push_back({{"detector", cfg.detectors[f.detector].id},
                                          {"kind", tsc::to_string(f.kind)},
                                          {"at_s", static_cast<double>(f.at) / 1000.0}});
      }
    }
  }
  out["wall_time_s"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();

  std::ofstream(args.summary) << out.dump(2) << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const UsageError&) {
    return 64;
  } catch (const std::exception& e) {
    std::cerr << "tsc_sumo: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "tsc_sumo: unknown exception\n";
    return 1;
  }
}
