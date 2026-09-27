#include <algorithm>

#include "tsc/controller.hpp"

namespace tsc {

Sequencer::Sequencer(Config cfg, TimeMs start)
    : stats_(cfg.phases.size()),
      cfg_(std::move(cfg)),
      // Pretend the last phase was just served, so the first cyclic search starts at phase 0.
      phase_(cfg_.phases.size() - 1),
      stage_start_(start),
      startup_red_(0) {
  // Start-up clearance: hold all-red for the longest all-red interval before any green.
  for (const auto& p : cfg_.phases) startup_red_ = std::max(startup_red_, p.all_red);
}

void Sequencer::try_start_green(TimeMs now) {
  const auto next = next_phase(now);
  if (!next) return;  // no demand anywhere: rest in red until a call arrives
  phase_ = *next;
  stage_ = Stage::Green;
  stage_start_ = now;
  ++stats_[phase_].served;
  on_green_start(phase_, now);
}

SignalVector Sequencer::step(TimeMs now) {
  before_step(now);
  const PhaseConfig& p = cfg_.phases[phase_];
  switch (stage_) {
    case Stage::StartupRed:
      if (now - stage_start_ >= startup_red_) try_start_green(now);
      break;
    case Stage::Green:
      if (const auto why = terminate(now, now - stage_start_)) {
        auto& s = stats_[phase_];
        switch (*why) {
          case Termination::GapOut: ++s.gap_outs; break;
          case Termination::MaxOut: ++s.max_outs; break;
          case Termination::Fixed: ++s.fixed_ends; break;
          case Termination::FailSafe: break;
        }
        stage_ = Stage::Yellow;
        stage_start_ = now;
      }
      break;
    case Stage::Yellow:
      if (now - stage_start_ >= p.yellow) {
        stage_ = Stage::AllRed;
        stage_start_ = now;
      }
      break;
    case Stage::AllRed:
      if (now - stage_start_ >= p.all_red) try_start_green(now);
      break;
  }
  return output();
}

SignalVector Sequencer::output() const {
  SignalVector out(cfg_.groups.size(), Signal::Red);
  if (stage_ == Stage::Green || stage_ == Stage::Yellow) {
    const Signal s = stage_ == Stage::Green ? Signal::Green : Signal::Yellow;
    for (GroupId g : cfg_.phases[phase_].groups) out[g] = s;
  }
  return out;
}

}  // namespace tsc
