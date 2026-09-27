#include "tsc/auditor.hpp"

namespace tsc {

InvariantAuditor::InvariantAuditor(const Config& cfg, TimeMs start)
    : cfg_(cfg), track_(cfg.groups.size(), Track{Signal::Red, start}) {}

void InvariantAuditor::flag(TimeMs now, const std::string& what) {
  ++count_;
  if (messages_.size() < 50) messages_.push_back("t=" + std::to_string(now) + "ms " + what);
}

void InvariantAuditor::observe(TimeMs now, const SignalVector& shown, bool failsafe) {
  const std::size_t n = track_.size();
  if (shown.size() != n) {
    flag(now, "wrong number of groups");
    return;
  }
  const std::vector<Track> before = track_;
  auto name = [&](std::size_t g) { return cfg_.groups[g]; };
  auto timing = [&](std::size_t g) -> const PhaseConfig& { return cfg_.phases[cfg_.group_phase[g]]; };

  // Rule 1: conflicting movements are never released together.
  for (std::size_t a = 0; a < n; ++a) {
    for (std::size_t b = 0; b < a; ++b) {
      if (cfg_.conflict(a, b) && shown[a] != Signal::Red && shown[b] != Signal::Red) {
        flag(now, "conflict: " + name(a) + " and " + name(b) + " both released");
      }
    }
  }

  // Rules 2-4: every change of aspect must be a legal, sufficiently timed step.
  for (std::size_t g = 0; g < n; ++g) {
    const Signal from = before[g].aspect;
    const Signal to = shown[g];
    if (from == to) continue;
    const TimeMs lasted = now - before[g].entered;
    const PhaseConfig& t = timing(g);

    if (from == Signal::Green && to == Signal::Yellow) {
      if (!failsafe && lasted < t.min_green) flag(now, name(g) + ": min green not honoured");
    } else if (from == Signal::Yellow && to == Signal::Red) {
      if (lasted < t.yellow) flag(now, name(g) + ": yellow cut short");
    } else if (from == Signal::Red && to == Signal::Green) {
      for (std::size_t h = 0; h < n; ++h) {
        if (!cfg_.conflict(g, h)) continue;
        const bool cleared = before[h].aspect == Signal::Red &&
                             now - before[h].entered >= timing(h).all_red;
        if (!cleared) flag(now, name(g) + ": green before " + name(h) + " completed all-red");
      }
    } else if (from == Signal::Green && to == Signal::Red) {
      flag(now, name(g) + ": green went to red without yellow");
    } else {
      flag(now, name(g) + ": illegal change " + std::string(1, to_char(from)) + "->" +
                    std::string(1, to_char(to)));
    }
    track_[g] = Track{to, now};
  }
}

}  // namespace tsc
