#include "tsc/runtime.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace tsc {

bool event_order(const DetectorEvent& a, const DetectorEvent& b) {
  if (a.t != b.t) return a.t < b.t;
  if (a.detector != b.detector) return a.detector < b.detector;
  return a.on && !b.on;  // a zero-length pulse at one instant is "on" then "off"
}

// ------------------------------------------------------------------ Producer

ControllerRuntime::Producer::Producer(std::shared_ptr<Inbox> inbox, std::size_t id,
                                      std::vector<DetectorId> owned, TimeMs start)
    : inbox_(std::move(inbox)), id_(id), owned_(std::move(owned)), watermark_(start - 1), open_(true) {}

ControllerRuntime::Producer::Producer(Producer&& other) noexcept
    : inbox_(std::move(other.inbox_)),
      id_(other.id_),
      owned_(std::move(other.owned_)),
      watermark_(other.watermark_),
      open_(std::exchange(other.open_, false)) {}

ControllerRuntime::Producer& ControllerRuntime::Producer::operator=(Producer&& other) noexcept {
  if (this != &other) {
    close();
    inbox_ = std::move(other.inbox_);
    id_ = other.id_;
    owned_ = std::move(other.owned_);
    watermark_ = other.watermark_;
    open_ = std::exchange(other.open_, false);
  }
  return *this;
}

ControllerRuntime::Producer::~Producer() {
  try {
    close();
  } catch (...) {  // NOLINT(bugprone-empty-catch): a destructor must not throw
  }
}

void ControllerRuntime::Producer::emit(const DetectorEvent& e) {
  if (!open_) throw std::logic_error("emit on a closed producer");
  if (std::find(owned_.begin(), owned_.end(), e.detector) == owned_.end()) {
    throw std::invalid_argument("producer " + std::to_string(id_) + " does not own detector " +
                                std::to_string(e.detector));
  }
  if (e.t <= watermark_) {
    throw std::invalid_argument("event at t=" + std::to_string(e.t) +
                                " is not later than this producer's watermark");
  }
  Message m;
  m.kind = Message::Kind::Event;
  m.producer = id_;
  m.event = e;
  inbox_->push(m);
}

void ControllerRuntime::Producer::advance(TimeMs watermark) {
  if (!open_) throw std::logic_error("advance on a closed producer");
  if (watermark <= watermark_) return;
  watermark_ = watermark;
  Message m;
  m.kind = Message::Kind::Watermark;
  m.producer = id_;
  m.watermark = watermark;
  inbox_->push(m);
}

void ControllerRuntime::Producer::close() {
  if (!open_) return;
  open_ = false;
  Message m;
  m.kind = Message::Kind::Closed;
  m.producer = id_;
  inbox_->push(m);
}

// ------------------------------------------------------------------ Runtime

ControllerRuntime::ControllerRuntime(const Config& cfg, std::unique_ptr<SignalController> controller,
                                     TimeMs start, std::vector<std::vector<DetectorId>> producer_detectors,
                                     std::size_t inbox_capacity)
    : inbox_(std::make_shared<Inbox>(inbox_capacity)),
      pipeline_(cfg, std::move(controller), start),
      tick_(cfg.tick),
      start_(start),
      owned_(std::move(producer_detectors)),
      handed_out_(owned_.size(), false) {
  if (owned_.empty()) throw std::invalid_argument("ControllerRuntime needs at least one producer");
  std::vector<int> owners(cfg.detectors.size(), 0);
  for (const auto& list : owned_) {
    for (DetectorId d : list) {
      if (d >= owners.size()) throw std::invalid_argument("producer owns unknown detector " + std::to_string(d));
      ++owners[d];
    }
  }
  for (std::size_t d = 0; d < owners.size(); ++d) {
    if (owners[d] != 1) {
      throw std::invalid_argument("detector " + std::to_string(d) + " must belong to exactly one producer");
    }
  }
  thread_ = std::thread([this] { run(start_); });
}

ControllerRuntime::~ControllerRuntime() {
  stop();
  if (thread_.joinable()) thread_.join();
}

ControllerRuntime::Producer ControllerRuntime::producer(std::size_t i) {
  if (i >= owned_.size()) throw std::out_of_range("no producer " + std::to_string(i));
  // Producer threads may ask for their handles concurrently. handed_out_ is a vector<bool>,
  // whose elements share machine words, so even different indices need the lock.
  const std::lock_guard lock(handout_mutex_);
  if (handed_out_[i]) throw std::logic_error("producer " + std::to_string(i) + " already handed out");
  handed_out_[i] = true;
  return Producer(inbox_, i, owned_[i], start_);
}

std::optional<TickRecord> ControllerRuntime::next_output() { return outbox_.pop(); }

void ControllerRuntime::stop() { inbox_->close(); }

const Pipeline& ControllerRuntime::join() {
  if (thread_.joinable()) thread_.join();
  return pipeline_;
}

ControllerRuntime::Stats ControllerRuntime::stats() const { return stats_; }

void ControllerRuntime::run(TimeMs start) {
  const std::size_t n = owned_.size();
  std::vector<TimeMs> watermark(n, start - 1);
  std::vector<bool> open(n, true);
  std::vector<DetectorEvent> pending;
  TimeMs next = start;
  std::optional<TimeMs> last_done;

  // Lowest watermark among producers still connected; nullopt when none is left.
  auto frontier = [&]() -> std::optional<TimeMs> {
    std::optional<TimeMs> lo;
    for (std::size_t p = 0; p < n; ++p) {
      if (open[p]) lo = lo ? std::min(*lo, watermark[p]) : watermark[p];
    }
    return lo;
  };

  while (auto msg = inbox_->pop()) {
    switch (msg->kind) {
      case Message::Kind::Event:
        ++stats_.events;
        if (last_done && msg->event.t <= *last_done) {
          ++stats_.late_events;  // cannot be ordered any more; apply now rather than drop it
          pipeline_.on_event(msg->event);
        } else {
          pending.push_back(msg->event);
        }
        break;
      case Message::Kind::Watermark:
        watermark[msg->producer] = std::max(watermark[msg->producer], msg->watermark);
        break;
      case Message::Kind::Closed:
        if (open[msg->producer]) {
          open[msg->producer] = false;
          for (DetectorId d : owned_[msg->producer]) pipeline_.controller().detector_feed_lost(d, next);
        }
        break;
    }

    for (auto lo = frontier(); lo && *lo >= next; lo = frontier()) {
      auto due_end = std::partition(pending.begin(), pending.end(),
                                    [&](const DetectorEvent& e) { return e.t <= next; });
      std::vector<DetectorEvent> due(pending.begin(), due_end);
      pending.erase(pending.begin(), due_end);
      std::sort(due.begin(), due.end(), event_order);
      for (const auto& e : due) pipeline_.on_event(e);
      outbox_.push(pipeline_.tick(next));
      last_done = next;
      next += tick_;
      ++stats_.ticks;
    }
    if (!frontier()) break;  // every producer has gone: nobody can advance time any more
  }
  outbox_.close();
}

}  // namespace tsc
