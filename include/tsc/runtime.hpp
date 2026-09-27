#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "tsc/blocking_queue.hpp"
#include "tsc/pipeline.hpp"

namespace tsc {

// Deterministic order in which simultaneous events are applied: time, then detector id, then
// "on" before "off".
bool event_order(const DetectorEvent& a, const DetectorEvent& b);

// Runs a Pipeline on its own thread, fed by several detector producer threads.
//
// Ordering: each producer sends detector events plus a *watermark* ("I have sent everything
// up to time w"). The controller thread executes tick T only once every open producer's
// watermark has reached T, and applies the pending events with t <= T sorted by
// (time, detector, edge). The signal sequence is therefore identical however the threads are
// scheduled, which the concurrency tests check against a single-threaded Pipeline.
//
// A producer that closes (or is destroyed) is treated as a lost detector feed. Its events up to
// its final watermark are applied as usual; at the first tick after that watermark all of its
// detectors are declared faulty (feed lost), so their phases fall back to recall. The fault
// time depends only on what that producer sent, so the output does not depend on when the
// close message arrives relative to the other producers' messages.
class ControllerRuntime {
 public:
  struct Message {
    enum class Kind : std::uint8_t { Event, Watermark, Closed };
    Kind kind = Kind::Event;
    std::size_t producer = 0;
    DetectorEvent event{};
    TimeMs watermark = 0;
  };
  using Inbox = BlockingQueue<Message>;

  // Handle given to one producer thread. Move-only; closing it (or destroying it) tells the
  // controller that this feed is gone. Safe to use even after the runtime is destroyed
  // (pushes are then dropped), because it co-owns the inbox.
  class Producer {
   public:
    Producer(Producer&& other) noexcept;
    Producer& operator=(Producer&& other) noexcept;
    Producer(const Producer&) = delete;
    Producer& operator=(const Producer&) = delete;
    ~Producer();

    // Throws std::invalid_argument if the detector is not owned by this producer or the event
    // is not later than this producer's last watermark.
    void emit(const DetectorEvent& e);
    void advance(TimeMs watermark);
    void close();

   private:
    friend class ControllerRuntime;
    Producer(std::shared_ptr<Inbox> inbox, std::size_t id, std::vector<DetectorId> owned, TimeMs start);
    std::shared_ptr<Inbox> inbox_;
    std::size_t id_ = 0;
    std::vector<DetectorId> owned_;
    TimeMs watermark_ = 0;
    bool open_ = false;
  };

  struct Stats {
    std::size_t events = 0;
    std::size_t late_events = 0;  // arrived after their tick was already executed (contract breach)
    std::size_t ticks = 0;
  };

  // producer_detectors[i] = detectors fed by producer i. Every detector must belong to exactly
  // one producer.
  ControllerRuntime(const Config& cfg, std::unique_ptr<SignalController> controller, TimeMs start,
                    std::vector<std::vector<DetectorId>> producer_detectors,
                    std::size_t inbox_capacity = 4096);
  ControllerRuntime(const ControllerRuntime&) = delete;
  ControllerRuntime& operator=(const ControllerRuntime&) = delete;
  ControllerRuntime(ControllerRuntime&&) = delete;
  ControllerRuntime& operator=(ControllerRuntime&&) = delete;
  ~ControllerRuntime();

  // Hands out the handle for producer i (once). Thread-safe.
  Producer producer(std::size_t i);
  // Next executed tick, in time order; nullopt once the runtime has finished.
  std::optional<TickRecord> next_output();
  // Aborts: the controller thread exits without waiting for producers.
  void stop();
  // Waits for the controller thread (it exits when all producers have closed, or on stop()).
  // Only after this returns may the pipeline be inspected.
  const Pipeline& join();
  [[nodiscard]] Stats stats() const;  // valid after join()

 private:
  void run(TimeMs start);

  std::shared_ptr<Inbox> inbox_;
  BlockingQueue<TickRecord> outbox_;
  Pipeline pipeline_;
  TimeMs tick_;
  TimeMs start_;
  std::vector<std::vector<DetectorId>> owned_;
  std::mutex handout_mutex_;
  std::vector<bool> handed_out_;  // guarded by handout_mutex_
  Stats stats_;
  std::thread thread_;
};

}  // namespace tsc
