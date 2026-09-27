#pragma once

#include <string>
#include <vector>

#include "tsc/config.hpp"

namespace tsc {

// After-the-fact checker for a stream of displayed signal vectors. It is deliberately a
// second, separately written implementation of the invariants in SafetyGuard, so that a bug
// in one is caught by the other. The SUMO harness runs it on the states SUMO actually shows,
// for every controller including SUMO's own.
class InvariantAuditor {
 public:
  InvariantAuditor(const Config& cfg, TimeMs start);

  // `failsafe` = the guard was in fail-safe for this sample (a green may then end early).
  void observe(TimeMs now, const SignalVector& shown, bool failsafe = false);

  [[nodiscard]] std::size_t violations() const { return count_; }
  [[nodiscard]] const std::vector<std::string>& messages() const { return messages_; }  // first 50

 private:
  struct Track {
    Signal aspect = Signal::Red;
    TimeMs entered = 0;
  };
  void flag(TimeMs now, const std::string& what);

  Config cfg_;  // own copy: the auditor may outlive the caller's config
  std::vector<Track> track_;
  std::size_t count_ = 0;
  std::vector<std::string> messages_;
};

}  // namespace tsc
