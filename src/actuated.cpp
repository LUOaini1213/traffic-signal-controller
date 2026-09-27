#include "tsc/actuated.hpp"

#include <algorithm>

namespace tsc {

ActuatedController::ActuatedController(Config cfg, TimeMs start)
    : Sequencer(std::move(cfg), start),
      monitor_(cfg_, start),
      calls_(cfg_.phases.size(), false),
      phase_detectors_(cfg_.phases.size()) {
  for (DetectorId d = 0; d < cfg_.detectors.size(); ++d) phase_detectors_[cfg_.detectors[d].phase].push_back(d);
}

void ActuatedController::on_event(const DetectorEvent& e) {
  if (e.detector >= monitor_.size()) return;  // unknown detector: ignore rather than index out of range
  if (!monitor_.update(e)) return;
  const PhaseId p = cfg_.detectors[e.detector].phase;
  const bool serving = stage() == Stage::Green && phase() == p;
  if (e.on && !serving) calls_[p] = true;
}

void ActuatedController::before_step(TimeMs now) { monitor_.check(now); }

void ActuatedController::detector_feed_lost(DetectorId d, TimeMs now) {
  if (d < monitor_.size()) monitor_.force_fault(d, FaultKind::FeedLost, now);
}

bool ActuatedController::wants_failsafe() const {
  // Stuck-off loops only put their own phase on recall. Loops that are stuck on or whose feed
  // is gone point at wiring or equipment trouble; many of them at once means the controller can
  // no longer see the junction.
  const std::size_t broken =
      monitor_.faulty_count(FaultKind::StuckOn) + monitor_.faulty_count(FaultKind::FeedLost);
  return broken > cfg_.faults.max_faulty;
}

bool ActuatedController::phase_has_fault(PhaseId p) const {
  const auto& dets = phase_detectors_[p];
  return std::any_of(dets.begin(), dets.end(), [&](DetectorId d) { return monitor_.faulty(d); });
}

Recall ActuatedController::effective_recall(PhaseId p) const {
  const Recall base = cfg_.phases[p].recall;
  if (!phase_has_fault(p)) return base;
  return std::max(base, cfg_.faults.fallback);
}

bool ActuatedController::has_call(PhaseId p) const {
  if (calls_[p] || effective_recall(p) != Recall::None) return true;
  // Presence: a vehicle standing on a loop keeps calling, e.g. a queue that reaches back over
  // the loop when its green ends must not be skipped just because nobody new arrived.
  const auto& dets = phase_detectors_[p];
  return std::any_of(dets.begin(), dets.end(), [&](DetectorId d) { return !monitor_.faulty(d) && monitor_.on(d); });
}

TimeMs ActuatedController::gap(PhaseId p, TimeMs now) const {
  TimeMs last_seen = stage() == Stage::Green && phase() == p ? stage_start() : now;
  for (DetectorId d : phase_detectors_[p]) {
    if (monitor_.faulty(d) || !cfg_.detectors[d].extends) continue;
    if (monitor_.on(d)) return 0;
    last_seen = std::max(last_seen, monitor_.last_change(d));
  }
  return now - last_seen;
}

std::optional<Termination> ActuatedController::terminate(TimeMs now, TimeMs green_elapsed) {
  const PhaseId p = phase();
  const PhaseConfig& cfg = cfg_.phases[p];
  if (green_elapsed < cfg.min_green) return std::nullopt;

  bool conflicting_call = false;
  for (PhaseId q = 0; q < cfg_.phases.size(); ++q) {
    if (q != p && has_call(q)) conflicting_call = true;
  }
  if (!conflicting_call) return std::nullopt;  // rest in green

  if (green_elapsed >= cfg.max_green) return Termination::MaxOut;
  if (effective_recall(p) == Recall::Max) return std::nullopt;  // held to max green
  if (gap(p, now) >= cfg.passage) return Termination::GapOut;
  return std::nullopt;
}

std::optional<PhaseId> ActuatedController::next_phase(TimeMs /*now*/) {
  const std::size_t n = cfg_.phases.size();
  for (std::size_t k = 1; k <= n; ++k) {
    const PhaseId q = (previous_phase() + k) % n;
    if (has_call(q)) {
      for (std::size_t s = 1; s < k; ++s) ++stats_[(previous_phase() + s) % n].skipped;
      return q;
    }
  }
  return std::nullopt;
}

void ActuatedController::on_green_start(PhaseId p, TimeMs /*now*/) { calls_[p] = false; }

}  // namespace tsc
