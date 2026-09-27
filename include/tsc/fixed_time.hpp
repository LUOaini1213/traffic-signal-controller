#pragma once

#include "tsc/controller.hpp"

namespace tsc {

// Pre-timed control: every phase is served every cycle for a fixed green, in config order.
// Detectors are ignored. Greens are typically produced by tsc::webster().
class FixedTimeController final : public Sequencer {
 public:
  // Throws std::invalid_argument if greens.size() != phases, a green is below the phase's
  // min green, or a green is not a multiple of the tick.
  FixedTimeController(Config cfg, TimeMs start, std::vector<TimeMs> greens);

  void on_event(const DetectorEvent& /*e*/) override {}
  [[nodiscard]] bool wants_failsafe() const override { return false; }
  void detector_feed_lost(DetectorId /*d*/, TimeMs /*now*/) override {}

  [[nodiscard]] const std::vector<TimeMs>& greens() const { return greens_; }
  [[nodiscard]] TimeMs cycle() const;

 protected:
  std::optional<Termination> terminate(TimeMs now, TimeMs green_elapsed) override;
  std::optional<PhaseId> next_phase(TimeMs now) override;

 private:
  std::vector<TimeMs> greens_;
};

}  // namespace tsc
