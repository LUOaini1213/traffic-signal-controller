#include "tsc/pipeline.hpp"

#include <stdexcept>

namespace tsc {

Pipeline::Pipeline(const Config& cfg, std::unique_ptr<SignalController> controller, TimeMs start)
    : controller_(std::move(controller)), guard_(cfg, start) {
  if (!controller_) throw std::invalid_argument("Pipeline needs a controller");
}

TickRecord Pipeline::tick(TimeMs now) {
  TickRecord r;
  r.t = now;
  const std::vector<PhaseStats> before = controller_->stats();
  r.commanded = controller_->step(now);
  const auto& after = controller_->stats();
  for (PhaseId p = 0; p < after.size(); ++p) {
    if (after[p].gap_outs != before[p].gap_outs) r.termination = Termination::GapOut;
    else if (after[p].max_outs != before[p].max_outs) r.termination = Termination::MaxOut;
    else if (after[p].fixed_ends != before[p].fixed_ends) r.termination = Termination::Fixed;
    else continue;
    r.terminated_phase = p;
  }
  if (controller_->wants_failsafe()) guard_.request_failsafe(now, "too many faulty detectors");
  r.displayed = guard_.apply(now, r.commanded);
  r.failsafe = guard_.failsafe();
  return r;
}

}  // namespace tsc
