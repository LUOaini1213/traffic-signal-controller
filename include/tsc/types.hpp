#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tsc {

// All controller time is integer milliseconds. Integer time keeps timer comparisons exact
// (no floating-point drift across a long simulation) and makes runs bit-for-bit repeatable.
using TimeMs = std::int64_t;
using GroupId = std::size_t;
using PhaseId = std::size_t;
using DetectorId = std::size_t;

// Aspect shown to one signal group (a set of movements that always move together).
enum class Signal : std::uint8_t { Red, Yellow, Green };

// Recall: Min = the phase always has a call (served every cycle for at least min green);
// Max = always has a call and is always extended, so it runs to max green.
enum class Recall : std::uint8_t { None, Min, Max };

enum class Termination : std::uint8_t { GapOut, MaxOut, Fixed, FailSafe };

struct DetectorEvent {
  TimeMs t = 0;
  DetectorId detector = 0;
  bool on = false;  // true = vehicle arrived over the loop, false = loop cleared
};

using SignalVector = std::vector<Signal>;

char to_char(Signal s);
std::string to_string(const SignalVector& v);  // e.g. "GGrrrrrr"
const char* to_string(Recall r);
const char* to_string(Termination t);

}  // namespace tsc
