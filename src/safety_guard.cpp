#include "tsc/safety_guard.hpp"

#include <algorithm>

namespace tsc {

SafetyGuard::SafetyGuard(const Config& cfg, TimeMs start)
    : conflicts_(cfg.conflicts),
      min_green_(cfg.groups.size()),
      yellow_(cfg.groups.size()),
      all_red_(cfg.groups.size()),
      shown_(cfg.groups.size(), Signal::Red),
      since_(cfg.groups.size(), start),
      start_(start) {
  for (const auto& p : cfg.phases) {
    for (GroupId g : p.groups) {
      min_green_[g] = p.min_green;
      yellow_[g] = p.yellow;
      all_red_[g] = p.all_red;
    }
  }
}

std::optional<std::string> SafetyGuard::check(TimeMs now, const SignalVector& cmd) const {
  const std::size_t n = shown_.size();
  if (cmd.size() != n) return "command has " + std::to_string(cmd.size()) + " groups, expected " + std::to_string(n);
  if (last_ ? now <= *last_ : now < start_) return "time did not advance";

  for (std::size_t a = 0; a < n; ++a) {
    for (std::size_t b = a + 1; b < n; ++b) {
      if (conflicts_[a][b] && cmd[a] != Signal::Red && cmd[b] != Signal::Red) {
        return "conflicting groups " + std::to_string(a) + " and " + std::to_string(b) + " both non-red";
      }
    }
  }

  for (std::size_t g = 0; g < n; ++g) {
    const Signal from = shown_[g];
    const Signal to = cmd[g];
    if (from == to) continue;
    const TimeMs held = now - since_[g];
    const std::string who = "group " + std::to_string(g);
    if (from == Signal::Green && to == Signal::Yellow) {
      if (held < min_green_[g]) return who + ": green ended before min green";
    } else if (from == Signal::Yellow && to == Signal::Red) {
      if (held < yellow_[g]) return who + ": yellow shorter than configured";
    } else if (from == Signal::Red && to == Signal::Green) {
      for (std::size_t h = 0; h < n; ++h) {
        if (!conflicts_[g][h]) continue;
        if (shown_[h] != Signal::Red || now - since_[h] < all_red_[h]) {
          return who + ": green before conflicting group " + std::to_string(h) + " finished all-red";
        }
      }
    } else {
      return who + ": illegal transition " + std::string(1, to_char(from)) + "->" + std::string(1, to_char(to));
    }
  }
  return std::nullopt;
}

const SignalVector& SafetyGuard::apply(TimeMs now, const SignalVector& cmd) {
  const TimeMs safe_now = trusted_time(now);
  if (!failsafe_) {
    if (auto why = check(now, cmd)) {
      ++refusals_;
      enter_failsafe(safe_now, *why);
    } else {
      for (std::size_t g = 0; g < shown_.size(); ++g) {
        if (shown_[g] != cmd[g]) {
          shown_[g] = cmd[g];
          since_[g] = now;
        }
      }
    }
  } else {
    advance_failsafe(safe_now);
  }
  last_ = safe_now;
  return shown_;
}

void SafetyGuard::request_failsafe(TimeMs now, const std::string& reason) {
  now = trusted_time(now);
  if (!failsafe_) enter_failsafe(now, reason);
  last_ = now;
}

TimeMs SafetyGuard::trusted_time(TimeMs now) const {
  // A refused timestamp must neither backdate a new yellow nor rewind the clock used by
  // subsequent calls. Requests and applies share this high-water mark.
  return std::max(now, last_.value_or(start_));
}

void SafetyGuard::enter_failsafe(TimeMs now, const std::string& reason) {
  failsafe_ = true;
  reason_ = reason;
  for (std::size_t g = 0; g < shown_.size(); ++g) {
    if (shown_[g] == Signal::Green) {
      shown_[g] = Signal::Yellow;
      since_[g] = now;
    }
  }
  advance_failsafe(now);
}

void SafetyGuard::advance_failsafe(TimeMs now) {
  for (std::size_t g = 0; g < shown_.size(); ++g) {
    if (shown_[g] == Signal::Yellow && now - since_[g] >= yellow_[g]) {
      shown_[g] = Signal::Red;
      since_[g] = now;
    }
  }
}

}  // namespace tsc
