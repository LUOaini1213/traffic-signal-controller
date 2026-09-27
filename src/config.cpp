#include "tsc/config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace tsc {

namespace {

std::string seconds_text(TimeMs ms) {
  std::ostringstream o;
  o << static_cast<double>(ms) / 1000.0;
  return o.str();
}

std::string join(const std::vector<std::string>& lines) {
  std::string out = "invalid configuration:";
  for (const auto& l : lines) out += "\n  - " + l;
  return out;
}

// Small reader that records problems instead of throwing on the first one, so a user sees
// every mistake in a config file at once.
class Reader {
 public:
  explicit Reader(std::vector<std::string>& errors) : errors_(errors) {}

  void error(const std::string& msg) { errors_.push_back(msg); }

  bool is_object(const nlohmann::json& j, const std::string& where) {
    if (j.is_object()) return true;
    error(where + ": expected an object");
    return false;
  }

  void no_unknown_keys(const nlohmann::json& j, const std::string& where,
                       std::initializer_list<const char*> known) {
    for (const auto& [key, _] : j.items()) {
      if (std::none_of(known.begin(), known.end(), [&](const char* k) { return key == k; })) {
        error(where + ": unknown key '" + key + "'");
      }
    }
  }

  const nlohmann::json* field(const nlohmann::json& j, const std::string& where, const char* key,
                              bool required = true) {
    auto it = j.find(key);
    if (it == j.end()) {
      if (required) error(where + ": missing required key '" + key + "'");
      return nullptr;
    }
    return &*it;
  }

  std::string string(const nlohmann::json& j, const std::string& where, const char* key) {
    const auto* v = field(j, where, key);
    if (v == nullptr) return {};
    if (!v->is_string()) {
      error(where + "." + key + ": expected a string");
      return {};
    }
    return v->get<std::string>();
  }

  // Seconds in the file, integer milliseconds in memory.
  TimeMs seconds(const nlohmann::json& j, const std::string& where, const char* key,
                 TimeMs fallback, bool required = true) {
    const auto* v = field(j, where, key, required);
    if (v == nullptr) return fallback;
    if (!v->is_number()) {
      error(where + "." + key + ": expected a number of seconds");
      return fallback;
    }
    const double s = v->get<double>();
    if (!std::isfinite(s) || std::abs(s) > 1e7) {
      error(where + "." + key + ": value out of range");
      return fallback;
    }
    return static_cast<TimeMs>(std::llround(s * 1000.0));
  }

  Recall recall(const nlohmann::json& j, const std::string& where, const char* key,
                Recall fallback) {
    const auto* v = field(j, where, key, false);
    if (v == nullptr) return fallback;
    const std::string s = v->is_string() ? v->get<std::string>() : std::string{};
    if (s == "none") return Recall::None;
    if (s == "min") return Recall::Min;
    if (s == "max") return Recall::Max;
    error(where + "." + key + ": expected one of \"none\", \"min\", \"max\"");
    return fallback;
  }

 private:
  std::vector<std::string>& errors_;
};

template <class T>
std::size_t index_of(const std::vector<T>& items, const std::string& name, const char* what) {
  for (std::size_t i = 0; i < items.size(); ++i) {
    if constexpr (std::is_same_v<T, std::string>) {
      if (items[i] == name) return i;
    } else if constexpr (std::is_same_v<T, PhaseConfig>) {
      if (items[i].name == name) return i;
    } else {
      if (items[i].id == name) return i;
    }
  }
  throw std::out_of_range(std::string("unknown ") + what + " '" + name + "'");
}

}  // namespace

ConfigError::ConfigError(std::vector<std::string> errors)
    : std::runtime_error(join(errors)), errors_(std::move(errors)) {}

std::size_t Config::group_index(const std::string& n) const { return index_of(groups, n, "group"); }
std::size_t Config::phase_index(const std::string& n) const { return index_of(phases, n, "phase"); }
std::size_t Config::detector_index(const std::string& id) const {
  return index_of(detectors, id, "detector");
}

std::vector<std::string> validate(const Config& c) {
  std::vector<std::string> e;
  const std::size_t n = c.groups.size();

  if (c.tick <= 0 || c.tick > 1000) e.push_back("tick_ms must be in (0, 1000]");
  if (n == 0) e.push_back("groups: at least one signal group is required");
  {
    std::set<std::string> seen;
    for (const auto& g : c.groups) {
      if (g.empty()) e.push_back("groups: empty group name");
      if (!seen.insert(g).second) e.push_back("groups: duplicate group '" + g + "'");
    }
  }

  // Conflict matrix: shape, diagonal, symmetry.
  bool matrix_ok = c.conflicts.size() == n;
  if (!matrix_ok) {
    e.push_back("conflicts: expected " + std::to_string(n) + " rows, got " +
                std::to_string(c.conflicts.size()));
  }
  for (std::size_t i = 0; matrix_ok && i < n; ++i) {
    if (c.conflicts[i].size() != n) {
      e.push_back("conflicts: row " + std::to_string(i) + " has " +
                  std::to_string(c.conflicts[i].size()) + " entries, expected " + std::to_string(n));
      matrix_ok = false;
    }
  }
  if (matrix_ok) {
    for (std::size_t i = 0; i < n; ++i) {
      if (c.conflicts[i][i]) e.push_back("conflicts: group '" + c.groups[i] + "' conflicts with itself");
      for (std::size_t k = i + 1; k < n; ++k) {
        if (c.conflicts[i][k] != c.conflicts[k][i]) {
          e.push_back("conflicts: matrix is not symmetric ('" + c.groups[i] + "' vs '" +
                      c.groups[k] + "')");
        }
      }
    }
  }

  if (c.phases.size() < 2) e.push_back("phases: at least two phases are required");
  std::vector<int> group_uses(n, 0);
  std::set<std::string> phase_names;
  for (const auto& p : c.phases) {
    const std::string where = "phase '" + p.name + "'";
    if (p.name.empty()) e.push_back("phases: empty phase name");
    if (!phase_names.insert(p.name).second) e.push_back(where + ": duplicate phase name");
    if (p.groups.empty()) e.push_back(where + ": has no signal groups");
    for (std::size_t a = 0; a < p.groups.size(); ++a) {
      if (p.groups[a] >= n) continue;  // reported by the parser
      ++group_uses[p.groups[a]];
      for (std::size_t b = a + 1; b < p.groups.size() && matrix_ok; ++b) {
        if (p.groups[b] < n && c.conflicts[p.groups[a]][p.groups[b]]) {
          e.push_back(where + ": groups '" + c.groups[p.groups[a]] + "' and '" +
                      c.groups[p.groups[b]] + "' conflict and cannot be green together");
        }
      }
    }
    if (p.min_green <= 0) e.push_back(where + ": min_green_s must be > 0");
    if (p.min_green > p.max_green) {
      e.push_back(where + ": min_green_s (" + seconds_text(p.min_green) +
                  ") is greater than max_green_s (" + seconds_text(p.max_green) + ")");
    }
    if (p.max_green > 300'000) e.push_back(where + ": max_green_s must be <= 300");
    if (p.passage <= 0) e.push_back(where + ": passage_s must be > 0");
    if (p.passage > p.max_green) e.push_back(where + ": passage_s must not exceed max_green_s");
    if (p.yellow < 3000 || p.yellow > 6000) e.push_back(where + ": yellow_s must be in [3, 6]");
    // At least one tick of all-red: a conflicting green may never start in the same tick in
    // which the previous green's yellow ends.
    if (p.all_red < c.tick || p.all_red > 6000) e.push_back(where + ": all_red_s must be in [tick, 6]");
    if (c.tick > 0) {
      for (TimeMs v : {p.min_green, p.max_green, p.passage, p.yellow, p.all_red}) {
        if (v % c.tick != 0) {
          e.push_back(where + ": all times must be multiples of tick_ms (" +
                      std::to_string(c.tick) + " ms)");
          break;
        }
      }
    }
  }
  for (std::size_t g = 0; g < n; ++g) {
    if (group_uses[g] == 0) e.push_back("group '" + c.groups[g] + "' is not served by any phase");
    if (group_uses[g] > 1) {
      e.push_back("group '" + c.groups[g] + "' appears in more than one phase (overlaps are not supported)");
    }
  }

  std::set<std::string> det_ids;
  for (const auto& d : c.detectors) {
    if (d.id.empty()) e.push_back("detectors: empty detector id");
    if (!det_ids.insert(d.id).second) e.push_back("detectors: duplicate id '" + d.id + "'");
    if (d.phase >= c.phases.size()) e.push_back("detector '" + d.id + "': unknown phase");
  }
  // A phase without recall is only ever served when a detector calls it.
  for (std::size_t p = 0; p < c.phases.size(); ++p) {
    const bool has_detector = std::any_of(c.detectors.begin(), c.detectors.end(),
                                          [&](const DetectorConfig& d) { return d.phase == p; });
    if (!has_detector && c.phases[p].recall == Recall::None) {
      e.push_back("phase '" + c.phases[p].name + "': recall is 'none' but no detector calls it, so it would never be served");
    }
  }

  const auto& f = c.faults;
  if (f.stuck_on <= 0 || f.stuck_off <= 0) e.push_back("faults: stuck thresholds must be > 0");
  if (f.fallback == Recall::None) e.push_back("faults.fallback_recall must be 'min' or 'max'");
  return e;
}

Config parse_config(const nlohmann::json& j) {
  std::vector<std::string> errors;
  Reader r(errors);
  Config c;
  if (!r.is_object(j, "config")) throw ConfigError(errors);
  r.no_unknown_keys(j, "config",
                    {"name", "tick_ms", "groups", "conflicts", "phases", "detectors", "faults", "sumo"});

  c.name = r.string(j, "config", "name");
  if (const auto* t = r.field(j, "config", "tick_ms")) {
    if (t->is_number_integer()) {
      c.tick = t->get<TimeMs>();
    } else {
      r.error("config.tick_ms: expected an integer number of milliseconds");
    }
  }

  if (const auto* g = r.field(j, "config", "groups")) {
    if (!g->is_array()) {
      r.error("config.groups: expected an array of names");
    } else {
      for (const auto& x : *g) {
        if (x.is_string()) {
          c.groups.push_back(x.get<std::string>());
        } else {
          r.error("config.groups: every entry must be a string");
        }
      }
    }
  }

  if (const auto* m = r.field(j, "config", "conflicts")) {
    if (!m->is_array()) {
      r.error("config.conflicts: expected a matrix (array of arrays of 0/1)");
    } else {
      for (const auto& row : *m) {
        std::vector<bool> out;
        if (!row.is_array()) {
          r.error("config.conflicts: every row must be an array");
        } else {
          for (const auto& x : row) {
            if (x.is_number_integer() && (x.get<int>() == 0 || x.get<int>() == 1)) {
              out.push_back(x.get<int>() == 1);
            } else if (x.is_boolean()) {
              out.push_back(x.get<bool>());
            } else {
              r.error("config.conflicts: entries must be 0 or 1");
              out.push_back(false);
            }
          }
        }
        c.conflicts.push_back(std::move(out));
      }
    }
  }

  if (const auto* ps = r.field(j, "config", "phases")) {
    if (!ps->is_array()) {
      r.error("config.phases: expected an array");
    } else {
      for (std::size_t i = 0; i < ps->size(); ++i) {
        const auto& pj = (*ps)[i];
        std::string where = "phases[" + std::to_string(i) + "]";
        if (!r.is_object(pj, where)) continue;
        r.no_unknown_keys(pj, where, {"name", "groups", "min_green_s", "max_green_s", "passage_s",
                                      "yellow_s", "all_red_s", "recall"});
        PhaseConfig p;
        p.name = r.string(pj, where, "name");
        if (!p.name.empty()) where = "phase '" + p.name + "'";
        if (const auto* gs = r.field(pj, where, "groups"); gs != nullptr && !gs->is_array()) {
          r.error(where + ".groups: expected an array of group names");
        } else if (gs != nullptr) {
          for (const auto& gname : *gs) {
            const std::string s = gname.is_string() ? gname.get<std::string>() : std::string{};
            auto it = std::find(c.groups.begin(), c.groups.end(), s);
            if (it == c.groups.end()) {
              r.error(where + ": unknown group '" + s + "'");
            } else {
              p.groups.push_back(static_cast<GroupId>(it - c.groups.begin()));
            }
          }
        }
        p.min_green = r.seconds(pj, where, "min_green_s", 0);
        p.max_green = r.seconds(pj, where, "max_green_s", 0);
        p.passage = r.seconds(pj, where, "passage_s", 0);
        p.yellow = r.seconds(pj, where, "yellow_s", 0);
        p.all_red = r.seconds(pj, where, "all_red_s", 0);
        p.recall = r.recall(pj, where, "recall", Recall::None);
        c.phases.push_back(std::move(p));
      }
    }
  }

  if (const auto* ds = r.field(j, "config", "detectors", false)) {
    if (!ds->is_array()) {
      r.error("config.detectors: expected an array");
    } else {
      for (std::size_t i = 0; i < ds->size(); ++i) {
        const auto& dj = (*ds)[i];
        const std::string where = "detectors[" + std::to_string(i) + "]";
        if (!r.is_object(dj, where)) continue;
        r.no_unknown_keys(dj, where, {"id", "phase", "mode"});
        DetectorConfig d;
        d.id = r.string(dj, where, "id");
        const std::string phase = r.string(dj, where, "phase");
        auto it = std::find_if(c.phases.begin(), c.phases.end(),
                               [&](const PhaseConfig& p) { return p.name == phase; });
        if (it == c.phases.end()) {
          r.error(where + ": unknown phase '" + phase + "'");
          d.phase = c.phases.size();
        } else {
          d.phase = static_cast<PhaseId>(it - c.phases.begin());
        }
        if (const auto* mode = r.field(dj, where, "mode", false)) {
          if (*mode == "extend") {
            d.extends = true;
          } else if (*mode == "call") {
            d.extends = false;
          } else {
            r.error(where + ".mode: expected \"extend\" or \"call\"");
          }
        }
        c.detectors.push_back(std::move(d));
      }
    }
  }

  if (const auto* fj = r.field(j, "config", "faults", false)) {
    if (r.is_object(*fj, "faults")) {
      r.no_unknown_keys(*fj, "faults",
                        {"enabled", "stuck_on_s", "stuck_off_s", "stuck_off_min_others", "fallback_recall", "max_faulty"});
      if (const auto* en = r.field(*fj, "faults", "enabled", false)) {
        if (en->is_boolean()) {
          c.faults.enabled = en->get<bool>();
        } else {
          r.error("faults.enabled: expected true or false");
        }
      }
      c.faults.stuck_on = r.seconds(*fj, "faults", "stuck_on_s", c.faults.stuck_on, false);
      c.faults.stuck_off = r.seconds(*fj, "faults", "stuck_off_s", c.faults.stuck_off, false);
      c.faults.fallback = r.recall(*fj, "faults", "fallback_recall", c.faults.fallback);
      for (auto [key, target] : {std::pair{"max_faulty", &c.faults.max_faulty},
                                 std::pair{"stuck_off_min_others", &c.faults.stuck_off_min_others}}) {
        if (const auto* v = r.field(*fj, "faults", key, false)) {
          if (v->is_number_integer() && v->get<long long>() >= 0) {
            *target = v->get<std::size_t>();
          } else {
            r.error(std::string("faults.") + key + ": expected a non-negative integer");
          }
        }
      }
    }
  }

  if (const auto* s = r.field(j, "config", "sumo", false)) c.sumo = *s;

  for (auto& msg : validate(c)) errors.push_back(std::move(msg));
  if (!errors.empty()) throw ConfigError(std::move(errors));

  c.group_phase.assign(c.groups.size(), 0);
  for (PhaseId p = 0; p < c.phases.size(); ++p) {
    for (GroupId g : c.phases[p].groups) c.group_phase[g] = p;
  }
  return c;
}

Config load_config(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw ConfigError({"cannot open '" + path.string() + "'"});
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(in);
  } catch (const nlohmann::json::parse_error& ex) {
    throw ConfigError({"'" + path.string() + "' is not valid JSON: " + ex.what()});
  }
  return parse_config(j);
}

}  // namespace tsc
