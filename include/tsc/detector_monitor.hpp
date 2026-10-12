#pragma once

#include <optional>
#include <vector>

#include "tsc/config.hpp"

namespace tsc {

enum class FaultKind : std::uint8_t { StuckOn, StuckOff, FeedLost };
const char* to_string(FaultKind k);

struct DetectorFault {
  DetectorId detector = 0;
  FaultKind kind = FaultKind::StuckOff;
  TimeMs at = 0;
};

// Tracks each loop's occupancy and flags loops that stop behaving like a real detector:
// occupied continuously for longer than faults.stuck_on, or empty for longer than
// faults.stuck_off while the other loops *of the same approach* (DetectorConfig::approach)
// counted at least faults.stuck_off_min_others vehicles. Silence is judged against related
// loops only, so a quiet side road next to a busy main road does not look broken. A fault is
// latched (a maintenance action would clear it); events from a faulty detector are ignored
// from then on.
class DetectorMonitor {
 public:
  DetectorMonitor(const Config& cfg, TimeMs start);

  // Returns true if the event changed the detector's occupancy and the detector is healthy.
  bool update(const DetectorEvent& e);
  // Runs the stuck-on / stuck-off checks; returns faults declared by this call.
  std::vector<DetectorFault> check(TimeMs now);
  // Declares a fault from outside (e.g. the detector's feed disconnected). FeedLost
  // supersedes a latched occupancy fault; duplicates and other changes are ignored.
  void force_fault(DetectorId d, FaultKind kind, TimeMs now);

  [[nodiscard]] bool on(DetectorId d) const { return state_.at(d).on; }
  [[nodiscard]] TimeMs last_change(DetectorId d) const { return state_.at(d).last_change; }
  [[nodiscard]] bool faulty(DetectorId d) const { return state_.at(d).fault.has_value(); }
  [[nodiscard]] std::size_t faulty_count() const;
  [[nodiscard]] std::size_t faulty_count(FaultKind kind) const;
  // History retains the initial fault and the time of any later feed-loss upgrade.
  [[nodiscard]] const std::vector<DetectorFault>& faults() const { return log_; }
  [[nodiscard]] std::size_t stale_events() const { return stale_; }
  [[nodiscard]] std::size_t size() const { return state_.size(); }

 private:
  struct State {
    bool on = false;
    TimeMs last_change = 0;
    std::size_t approach = 0;              // index into arrivals_
    std::uint64_t arrivals_at_change = 0;  // arrivals_[approach] when this loop last changed
    std::optional<FaultKind> fault;
  };
  FaultConfig cfg_;
  std::vector<State> state_;
  std::vector<DetectorFault> log_;
  std::size_t stale_ = 0;
  std::vector<std::uint64_t> arrivals_;  // "on" edges accepted from healthy loops, per approach
};

}  // namespace tsc
