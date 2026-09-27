#pragma once

#include <memory>
#include <optional>

#include "tsc/controller.hpp"
#include "tsc/safety_guard.hpp"

namespace tsc {

struct TickRecord {
  TimeMs t = 0;
  SignalVector commanded;  // what the controller proposed
  SignalVector displayed;  // what the safety guard let through
  bool failsafe = false;
  // Set on the tick at which a green was terminated (gap-out, max-out or fixed end).
  std::optional<Termination> termination;
  PhaseId terminated_phase = 0;
};

// Controller + safety guard, driven from a single thread. ControllerRuntime wraps one of
// these in its own thread; tests and the replay tool use it directly.
class Pipeline {
 public:
  Pipeline(const Config& cfg, std::unique_ptr<SignalController> controller, TimeMs start);

  void on_event(const DetectorEvent& e) { controller_->on_event(e); }
  TickRecord tick(TimeMs now);

  [[nodiscard]] SignalController& controller() { return *controller_; }
  [[nodiscard]] const SignalController& controller() const { return *controller_; }
  [[nodiscard]] const SafetyGuard& guard() const { return guard_; }

 private:
  std::unique_ptr<SignalController> controller_;
  SafetyGuard guard_;
};

}  // namespace tsc
