#pragma once

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "tsc/actuated.hpp"
#include "tsc/config.hpp"
#include "tsc/pipeline.hpp"

namespace tsc::test {

inline nlohmann::json repo_config_json() {
  std::ifstream in(std::string(TSC_SOURCE_DIR) + "/configs/intersection.json");
  return nlohmann::json::parse(in);
}

inline Config repo_config() { return parse_config(repo_config_json()); }

inline Config config_with(const std::function<void(nlohmann::json&)>& edit) {
  auto j = repo_config_json();
  edit(j);
  return parse_config(j);
}

// Phase and group indices of the repository config.
inline constexpr PhaseId NS_T = 0, NS_R = 1, EW_T = 2, EW_R = 3;
inline constexpr GroupId G_NT = 0, G_ST = 1, G_NR = 2, G_SR = 3, G_ET = 4, G_WT = 5, G_ER = 6, G_WR = 7;

inline constexpr TimeMs s(double seconds) { return static_cast<TimeMs>(seconds * 1000.0); }

// Drives an ActuatedController (behind a Pipeline) tick by tick with scripted events.
class Driver {
 public:
  explicit Driver(Config cfg) : cfg_(std::move(cfg)) {
    auto c = std::make_unique<ActuatedController>(cfg_, 0);
    ctl_ = c.get();
    pipe_ = std::make_unique<Pipeline>(cfg_, std::move(c), 0);
  }

  // Queue a detector event (by config id) for the given time.
  void event(double t_s, const std::string& det, bool on) {
    events_.push_back({s(t_s), cfg_.detector_index(det), on});
  }
  // A vehicle passing a loop: on at t, off 0.5 s later.
  void pulse(double t_s, const std::string& det) {
    event(t_s, det, true);
    event(t_s + 0.5, det, false);
  }

  // Run ticks up to and including t_s.
  void run_until(double t_s) {
    for (; now_ <= s(t_s); now_ += cfg_.tick) {
      for (const auto& e : events_) {
        if (e.t > now_ - cfg_.tick && e.t <= now_) pipe_->on_event(e);
      }
      last_ = pipe_->tick(now_);
      history_.push_back(last_);
    }
  }

  [[nodiscard]] Signal at(double t_s, GroupId g) const {
    for (const auto& r : history_) {
      if (r.t == s(t_s)) return r.displayed.at(g);
    }
    throw std::out_of_range("no tick recorded at that time");
  }
  // First time >= from_s at which group g shows aspect a (-1 if never).
  [[nodiscard]] double first(Signal a, GroupId g, double from_s = 0) const {
    for (const auto& r : history_) {
      if (r.t >= s(from_s) && r.displayed.at(g) == a) return static_cast<double>(r.t) / 1000.0;
    }
    return -1;
  }

  ActuatedController& ctl() { return *ctl_; }
  Pipeline& pipe() { return *pipe_; }
  const std::vector<TickRecord>& history() const { return history_; }
  const TickRecord& last() const { return last_; }

 private:
  Config cfg_;
  ActuatedController* ctl_ = nullptr;  // owned by pipe_
  std::unique_ptr<Pipeline> pipe_;
  std::vector<DetectorEvent> events_;
  std::vector<TickRecord> history_;
  TickRecord last_;
  TimeMs now_ = 0;
};

}  // namespace tsc::test
