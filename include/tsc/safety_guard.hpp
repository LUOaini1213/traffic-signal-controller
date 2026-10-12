#pragma once

#include <optional>
#include <string>
#include <vector>

#include "tsc/config.hpp"

namespace tsc {

// The only path from a controller to the signal heads. It keeps its own record of what is
// displayed and since when, and checks every proposed vector *before* it is shown:
//
//   1. no two conflicting groups are non-red at the same time (green or yellow);
//   2. a green lasts at least its min green before turning yellow;
//   3. green is always followed by yellow (never straight to red), yellow by red, and a
//      yellow lasts at least its configured time;
//   4. a group may only turn green if every conflicting group has been red for at least that
//      group's all-red clearance.
//
// A command that breaks any rule is refused and the guard latches into fail-safe: groups that
// are green get their yellow, then everything is held red (flashing red on street hardware,
// i.e. an all-way stop). Fail-safe entry is the one case where a green may be cut short of
// min green; yellow is still never skipped. Only a new SafetyGuard (a restart) clears it.
// Repeated/backward apply timestamps are refused. Fail-safe timing never moves before the
// start time or the latest timestamp observed by apply/request_failsafe, so a stale input
// cannot backdate the start of yellow or shorten its clearance.
class SafetyGuard {
 public:
  SafetyGuard(const Config& cfg, TimeMs start);

  // Returns what is actually displayed for [now, now + tick).
  const SignalVector& apply(TimeMs now, const SignalVector& command);
  // Pure check, no state change. Returns the reason the command would be refused.
  [[nodiscard]] std::optional<std::string> check(TimeMs now, const SignalVector& command) const;
  // Enter fail-safe on request (e.g. the controller lost too many detectors). Calling apply
  // at the same timestamp afterwards is allowed once fail-safe is latched.
  void request_failsafe(TimeMs now, const std::string& reason);

  [[nodiscard]] bool failsafe() const { return failsafe_; }
  [[nodiscard]] const std::string& failsafe_reason() const { return reason_; }
  [[nodiscard]] std::size_t refusals() const { return refusals_; }
  [[nodiscard]] const SignalVector& displayed() const { return shown_; }

 private:
  [[nodiscard]] TimeMs trusted_time(TimeMs now) const;
  void enter_failsafe(TimeMs now, const std::string& reason);
  void advance_failsafe(TimeMs now);

  std::vector<std::vector<bool>> conflicts_;
  std::vector<TimeMs> min_green_, yellow_, all_red_;
  SignalVector shown_;
  std::vector<TimeMs> since_;  // when each group's current aspect started
  TimeMs start_;
  std::optional<TimeMs> last_;  // highest observed time, clamped to start_ after the first call
  bool failsafe_ = false;
  std::string reason_;
  std::size_t refusals_ = 0;
};

}  // namespace tsc
