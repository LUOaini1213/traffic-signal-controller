#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "tsc/types.hpp"

namespace tsc {

struct PhaseConfig {
  std::string name;
  std::vector<GroupId> groups;
  TimeMs min_green = 0;
  TimeMs max_green = 0;
  TimeMs passage = 0;  // gap-out time: green ends when no vehicle has been seen for this long
  TimeMs yellow = 0;
  TimeMs all_red = 0;
  Recall recall = Recall::None;
};

struct DetectorConfig {
  std::string id;
  PhaseId phase = 0;
  // "extend" loops place calls and extend the green; "call" loops (e.g. at the stop line)
  // only place calls, so a vehicle waiting on one is never stranded but cannot hold a green.
  bool extends = true;
  // Loops with the same approach label judge each other for stuck-off faults: a loop is only
  // suspected of being stuck off while the other loops of its approach keep seeing vehicles.
  // Loops without a label all share the empty label.
  std::string approach;
};

struct FaultConfig {
  bool enabled = true;
  TimeMs stuck_on = 300'000;   // continuously occupied this long -> faulty
  TimeMs stuck_off = 600'000;  // continuously empty this long -> faulty ...
  // ... but only if the other loops of the same approach saw at least this many vehicles
  // meanwhile. A loop that is quiet while its approach is quiet (night, a side road without
  // traffic, the end of a run) is not evidence of a fault.
  std::size_t stuck_off_min_others = 20;
  Recall fallback = Recall::Max;
  // More stuck-on or feed-lost detectors than this and the controller asks the safety guard
  // for fail-safe. Stuck-off loops never count towards it: only their own phase goes to recall.
  std::size_t max_faulty = 4;
};

struct Config {
  std::string name;
  TimeMs tick = 500;
  std::vector<std::string> groups;
  std::vector<std::vector<bool>> conflicts;  // groups x groups, symmetric, false diagonal
  std::vector<PhaseConfig> phases;           // served in this cyclic order
  std::vector<DetectorConfig> detectors;
  FaultConfig faults;
  nlohmann::json sumo;  // optional simulator binding; not interpreted by the core library

  // Derived by parse_config: the (single) phase that contains each group.
  std::vector<PhaseId> group_phase;

  [[nodiscard]] bool conflict(GroupId a, GroupId b) const { return conflicts.at(a).at(b); }
  [[nodiscard]] std::size_t group_index(const std::string& name) const;
  [[nodiscard]] std::size_t phase_index(const std::string& name) const;
  [[nodiscard]] std::size_t detector_index(const std::string& id) const;
};

// Thrown when a configuration is rejected. what() lists every problem found, one per line.
class ConfigError : public std::runtime_error {
 public:
  explicit ConfigError(std::vector<std::string> errors);
  [[nodiscard]] const std::vector<std::string>& errors() const noexcept { return errors_; }

 private:
  std::vector<std::string> errors_;
};

// Parse and validate. Never returns an invalid Config: any problem throws ConfigError.
Config parse_config(const nlohmann::json& j);
Config load_config(const std::filesystem::path& path);

// Semantic checks on an already-structured Config. Returns an empty list when valid.
std::vector<std::string> validate(const Config& c);

}  // namespace tsc
