#include "tsc/detector_monitor.hpp"

#include <algorithm>

namespace tsc {

const char* to_string(FaultKind k) {
  switch (k) {
    case FaultKind::StuckOn: return "stuck-on";
    case FaultKind::StuckOff: return "stuck-off";
    case FaultKind::FeedLost: return "feed-lost";
  }
  return "?";
}

DetectorMonitor::DetectorMonitor(const Config& cfg, TimeMs start)
    : cfg_(cfg.faults), state_(cfg.detectors.size(), State{false, start, 0, 0, std::nullopt}) {
  // Detectors with the same approach label count each other's vehicles.
  std::vector<std::string> labels;
  for (DetectorId d = 0; d < state_.size(); ++d) {
    const std::string& label = cfg.detectors[d].approach;
    auto it = std::find(labels.begin(), labels.end(), label);
    if (it == labels.end()) it = labels.insert(labels.end(), label);
    state_[d].approach = static_cast<std::size_t>(it - labels.begin());
  }
  arrivals_.assign(labels.size(), 0);
}

bool DetectorMonitor::update(const DetectorEvent& e) {
  State& s = state_.at(e.detector);
  if (s.fault) return false;
  if (e.t < s.last_change) {  // older than what we already know: cannot be applied safely
    ++stale_;
    return false;
  }
  if (e.on == s.on) return false;
  s.on = e.on;
  s.last_change = e.t;
  if (e.on) ++arrivals_[s.approach];
  s.arrivals_at_change = arrivals_[s.approach];
  return true;
}

std::vector<DetectorFault> DetectorMonitor::check(TimeMs now) {
  std::vector<DetectorFault> fresh;
  if (!cfg_.enabled) return fresh;
  for (DetectorId d = 0; d < state_.size(); ++d) {
    State& s = state_[d];
    if (s.fault) continue;
    const TimeMs held = now - s.last_change;
    // Vehicles counted by the other loops of the same approach since this loop last changed.
    const std::uint64_t peers_saw = arrivals_[s.approach] - s.arrivals_at_change;
    if (s.on && held >= cfg_.stuck_on) {
      s.fault = FaultKind::StuckOn;
    } else if (!s.on && held >= cfg_.stuck_off && peers_saw >= cfg_.stuck_off_min_others) {
      s.fault = FaultKind::StuckOff;
    } else {
      continue;
    }
    fresh.push_back({d, *s.fault, now});
    log_.push_back(fresh.back());
  }
  return fresh;
}

void DetectorMonitor::force_fault(DetectorId d, FaultKind kind, TimeMs now) {
  State& s = state_.at(d);
  if (s.fault) return;
  s.fault = kind;
  log_.push_back({d, kind, now});
}

std::size_t DetectorMonitor::faulty_count() const {
  return static_cast<std::size_t>(
      std::count_if(state_.begin(), state_.end(), [](const State& s) { return s.fault.has_value(); }));
}

std::size_t DetectorMonitor::faulty_count(FaultKind kind) const {
  return static_cast<std::size_t>(
      std::count_if(state_.begin(), state_.end(), [kind](const State& s) { return s.fault == kind; }));
}

}  // namespace tsc
