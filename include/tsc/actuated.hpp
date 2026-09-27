#pragma once

#include "tsc/controller.hpp"
#include "tsc/detector_monitor.hpp"

namespace tsc {

// Single-ring, fully actuated control with locking detector memory.
//
//  * A detector actuation while its phase is not green places a call (latched until served).
//  * Phases with recall "min"/"max" always have a call; a phase with no call is skipped.
//  * While green, the phase is extended by vehicles: it gaps out once min green has elapsed
//    and no healthy detector of the phase has been occupied for `passage`.
//  * It maxes out when green has lasted max green (timed from the start of green).
//  * A green only ends if some other phase is calling; otherwise it rests in green.
//  * A phase with a faulty detector falls back to faults.fallback recall. Only stuck-on and
//    feed-lost detectors count towards faults.max_faulty (junction fail-safe).
class ActuatedController final : public Sequencer {
 public:
  ActuatedController(Config cfg, TimeMs start);

  void on_event(const DetectorEvent& e) override;
  [[nodiscard]] bool wants_failsafe() const override;
  void detector_feed_lost(DetectorId d, TimeMs now) override;

  [[nodiscard]] const DetectorMonitor& monitor() const { return monitor_; }
  [[nodiscard]] bool has_call(PhaseId p) const;
  [[nodiscard]] Recall effective_recall(PhaseId p) const;
  // Time since the phase's extending detectors last saw a vehicle (0 while one is over a loop).
  [[nodiscard]] TimeMs gap(PhaseId p, TimeMs now) const;

 protected:
  void before_step(TimeMs now) override;
  std::optional<Termination> terminate(TimeMs now, TimeMs green_elapsed) override;
  std::optional<PhaseId> next_phase(TimeMs now) override;
  void on_green_start(PhaseId p, TimeMs now) override;

 private:
  [[nodiscard]] bool phase_has_fault(PhaseId p) const;

  DetectorMonitor monitor_;
  std::vector<bool> calls_;
  std::vector<std::vector<DetectorId>> phase_detectors_;
};

}  // namespace tsc
