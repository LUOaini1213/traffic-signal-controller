#include "tsc/types.hpp"

namespace tsc {

char to_char(Signal s) {
  switch (s) {
    case Signal::Red: return 'r';
    case Signal::Yellow: return 'y';
    case Signal::Green: return 'G';
  }
  return '?';
}

std::string to_string(const SignalVector& v) {
  std::string out;
  out.reserve(v.size());
  for (Signal s : v) out.push_back(to_char(s));
  return out;
}

const char* to_string(Recall r) {
  switch (r) {
    case Recall::None: return "none";
    case Recall::Min: return "min";
    case Recall::Max: return "max";
  }
  return "?";
}

const char* to_string(Termination t) {
  switch (t) {
    case Termination::GapOut: return "gap-out";
    case Termination::MaxOut: return "max-out";
    case Termination::Fixed: return "fixed";
    case Termination::FailSafe: return "fail-safe";
  }
  return "?";
}

}  // namespace tsc
