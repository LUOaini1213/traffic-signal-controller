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
// faults.stuck_off while the other loops counted at least faults.stuck_off_min_others
// vehicles (so a quiet junction does not look like a broken one). A fault is latched (a
// maintenance action would clear it); events from a faulty detector are ignored from then on.
class DetectorMonitor {
 public:
  DetectorMonitor(const Config& cfg, TimeMs start);

  // Returns true if the event changed the detector's occupancy and the detector is healthy.
  bool update(const DetectorEvent& e);
  // Runs the stuck-on / stuck-off checks; returns faults declared by this call.
  std::vector<DetectorFault> check(TimeMs now);
  // Declares a fault from outside (e.g. the detector's feed disconnected).
  void force_fault(DetectorId d, FaultKind kind, TimeMs now);

  [[nodiscard]] bool on(DetectorId d) const { return state_.at(d).on; }
  [[nodiscard]] TimeMs last_change(DetectorId d) const { return state_.at(d).last_change; }
  [[nodiscard]] bool faulty(DetectorId d) const { return state_.at(d).fault.has_value(); }
  [[nodiscard]] std::size_t faulty_count() const;
  [[nodiscard]] const std::vector<DetectorFault>& faults() const { return log_; }
  [[nodiscard]] std::size_t stale_events() const { return stale_; }
  [[nodiscard]] std::size_t size() const { return state_.size(); }

 private:
  struct State {
    bool on = false;
    TimeMs last_change = 0;
    std::uint64_t arrivals_at_change = 0;  // value of arrivals_ when this loop last changed
    std::optional<FaultKind> fault;
  };
  FaultConfig cfg_;
  std::vector<State> state_;
  std::vector<DetectorFault> log_;
  std::size_t stale_ = 0;
  std::uint64_t arrivals_ = 0;  // "on" edges accepted from healthy loops, all loops together
};

}  // namespace tsc
