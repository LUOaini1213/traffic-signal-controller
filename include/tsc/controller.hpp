#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "tsc/config.hpp"

namespace tsc {

struct PhaseStats {
  std::uint64_t served = 0;
  std::uint64_t gap_outs = 0;
  std::uint64_t max_outs = 0;
  std::uint64_t fixed_ends = 0;
  std::uint64_t skipped = 0;  // passed over because it had no call
};

// What the safety pipeline drives. Implementations produce a *proposed* signal vector each
// tick; nothing reaches the street without passing SafetyGuard::apply.
class SignalController {
 public:
  SignalController() = default;
  SignalController(const SignalController&) = delete;
  SignalController& operator=(const SignalController&) = delete;
  SignalController(SignalController&&) = delete;
  SignalController& operator=(SignalController&&) = delete;
  virtual ~SignalController() = default;

  virtual void on_event(const DetectorEvent& e) = 0;
  // Proposed aspects for the interval [now, now + tick).
  virtual SignalVector step(TimeMs now) = 0;
  [[nodiscard]] virtual bool wants_failsafe() const = 0;
  // The feed carrying this detector has gone away (e.g. its producer disconnected).
  virtual void detector_feed_lost(DetectorId d, TimeMs now) = 0;
  [[nodiscard]] virtual const std::vector<PhaseStats>& stats() const = 0;
};

// Shared phase/stage machine: startup all-red, then per phase green -> yellow -> all-red.
// Subclasses decide only *when* a green ends and *which* phase comes next.
class Sequencer : public SignalController {
 public:
  enum class Stage : std::uint8_t { StartupRed, Green, Yellow, AllRed };

  SignalVector step(TimeMs now) final;
  [[nodiscard]] const std::vector<PhaseStats>& stats() const final { return stats_; }

  [[nodiscard]] Stage stage() const { return stage_; }
  [[nodiscard]] PhaseId phase() const { return phase_; }
  [[nodiscard]] TimeMs stage_start() const { return stage_start_; }
  [[nodiscard]] const Config& config() const { return cfg_; }

 protected:
  Sequencer(Config cfg, TimeMs start);

  virtual void before_step(TimeMs /*now*/) {}
  virtual std::optional<Termination> terminate(TimeMs now, TimeMs green_elapsed) = 0;
  virtual std::optional<PhaseId> next_phase(TimeMs now) = 0;
  virtual void on_green_start(PhaseId /*p*/, TimeMs /*now*/) {}

  // Phase served before the one being chosen (for cyclic "next after current" searches).
  [[nodiscard]] PhaseId previous_phase() const { return phase_; }
  std::vector<PhaseStats> stats_;
  Config cfg_;

 private:
  void try_start_green(TimeMs now);
  [[nodiscard]] SignalVector output() const;

  Stage stage_ = Stage::StartupRed;
  PhaseId phase_;
  TimeMs stage_start_;
  TimeMs startup_red_;
};

}  // namespace tsc
