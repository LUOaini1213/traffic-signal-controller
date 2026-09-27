#include "tsc/fixed_time.hpp"

#include <stdexcept>
#include <string>

namespace tsc {

namespace {
std::vector<TimeMs> checked(const Config& cfg, std::vector<TimeMs> greens) {
  if (greens.size() != cfg.phases.size()) {
    throw std::invalid_argument("fixed-time plan has " + std::to_string(greens.size()) +
                                " greens for " + std::to_string(cfg.phases.size()) + " phases");
  }
  for (std::size_t p = 0; p < greens.size(); ++p) {
    if (greens[p] < cfg.phases[p].min_green) {
      throw std::invalid_argument("fixed-time green for phase '" + cfg.phases[p].name +
                                  "' is below its min green");
    }
    if (greens[p] % cfg.tick != 0) {
      throw std::invalid_argument("fixed-time green for phase '" + cfg.phases[p].name +
                                  "' is not a multiple of the tick");
    }
  }
  return greens;
}
}  // namespace

FixedTimeController::FixedTimeController(Config cfg, TimeMs start, std::vector<TimeMs> greens)
    : Sequencer(std::move(cfg), start), greens_(checked(cfg_, std::move(greens))) {}

TimeMs FixedTimeController::cycle() const {
  TimeMs c = 0;
  for (std::size_t p = 0; p < greens_.size(); ++p) {
    c += greens_[p] + cfg_.phases[p].yellow + cfg_.phases[p].all_red;
  }
  return c;
}

std::optional<Termination> FixedTimeController::terminate(TimeMs /*now*/, TimeMs green_elapsed) {
  if (green_elapsed >= greens_[phase()]) return Termination::Fixed;
  return std::nullopt;
}

std::optional<PhaseId> FixedTimeController::next_phase(TimeMs /*now*/) {
  return (previous_phase() + 1) % cfg_.phases.size();
}

}  // namespace tsc
