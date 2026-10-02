// Multi-threaded ingestion: several producer threads feed one controller thread.
// Run these under TSan (see CMakePresets.json) to check for data races as well as results.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <latch>
#include <numeric>
#include <random>
#include <thread>

#include "helpers.hpp"
#include "tsc/runtime.hpp"

using namespace tsc;
using namespace tsc::test;

namespace {

// Producer i owns the loops of one approach: three advance loops and one stop-bar loop.
std::vector<std::vector<DetectorId>> by_approach() {
  return {{0, 1, 2, 12}, {3, 4, 5, 13}, {6, 7, 8, 14}, {9, 10, 11, 15}};
}

std::vector<DetectorEvent> traffic(std::uint64_t seed, TimeMs end) {
  std::mt19937_64 rng(seed);
  std::vector<DetectorEvent> ev;
  for (DetectorId d = 0; d < 16; ++d) {
    std::uniform_int_distribution<TimeMs> headway(1, 30), occ(0, 4);
    for (TimeMs t = 500 * headway(rng); t < end; t += 500 * headway(rng)) {
      ev.push_back({t, d, true});
      t += 500 * occ(rng);
      ev.push_back({t, d, false});
    }
  }
  // Drop the few "off" edges that fall after the end of the run.
  std::erase_if(ev, [end](const DetectorEvent& e) { return e.t > end; });
  std::sort(ev.begin(), ev.end(), event_order);
  return ev;
}

// Reference: the same events through a single-threaded Pipeline.
std::vector<TickRecord> single_threaded(const Config& cfg, const std::vector<DetectorEvent>& ev, TimeMs end) {
  Pipeline pipe(cfg, std::make_unique<ActuatedController>(cfg, 0), 0);
  std::vector<TickRecord> out;
  std::size_t next = 0;
  for (TimeMs t = 0; t <= end; t += cfg.tick) {
    while (next < ev.size() && ev[next].t <= t) pipe.on_event(ev[next++]);
    out.push_back(pipe.tick(t));
  }
  return out;
}

struct ThreadedRun {
  std::vector<TickRecord> out;
  ControllerRuntime::Stats stats;
};

ThreadedRun threaded(const Config& cfg, const std::vector<DetectorEvent>& ev, TimeMs end, std::uint64_t jitter_seed) {
  const auto owners = by_approach();
  ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, owners, /*inbox_capacity=*/64);
  std::latch reached_end(static_cast<std::ptrdiff_t>(owners.size()));
  std::vector<std::thread> producers;
  for (std::size_t p = 0; p < owners.size(); ++p) {
    producers.emplace_back([&, p, handle = rt.producer(p)]() mutable {
      std::mt19937_64 rng(jitter_seed * 31 + p);
      std::vector<DetectorEvent> mine;
      for (const auto& e : ev) {
        if (std::find(owners[p].begin(), owners[p].end(), e.detector) != owners[p].end()) mine.push_back(e);
      }
      std::size_t i = 0;
      for (TimeMs t = 0; t <= end; t += cfg.tick) {
        while (i < mine.size() && mine[i].t <= t) handle.emit(mine[i++]);
        // Watermarks at irregular intervals, and random pauses, so that the threads interleave
        // differently on every run.
        if (std::uniform_int_distribution<int>(0, 3)(rng) == 0 || t == end) handle.advance(t);
        if (std::uniform_int_distribution<int>(0, 50)(rng) == 0) std::this_thread::yield();
        if (std::uniform_int_distribution<int>(0, 400)(rng) == 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
      }
      // Close only after everybody has published the final watermark; otherwise an early
      // close would (correctly) be treated as a lost feed and change the result.
      reached_end.arrive_and_wait();
      handle.close();
    });
  }
  ThreadedRun run;
  while (auto r = rt.next_output()) run.out.push_back(std::move(*r));
  for (auto& t : producers) t.join();
  rt.join();
  run.stats = rt.stats();
  return run;
}

}  // namespace

TEST(Concurrency, FourProducerThreadsGiveTheSameSignalsAsOneThread) {
  const Config cfg = repo_config();
  const TimeMs end = s(1800);
  const auto ev = traffic(42, end);
  const auto expected = single_threaded(cfg, ev, end);
  for (std::uint64_t run = 0; run < 5; ++run) {
    const auto got = threaded(cfg, ev, end, run);
    ASSERT_EQ(got.out.size(), expected.size()) << "run " << run;
    for (std::size_t i = 0; i < expected.size(); ++i) {
      ASSERT_EQ(got.out[i].t, expected[i].t);
      ASSERT_EQ(got.out[i].displayed, expected[i].displayed) << "run " << run << " tick " << i;
      ASSERT_EQ(got.out[i].commanded, expected[i].commanded) << "run " << run << " tick " << i;
    }
    EXPECT_EQ(got.stats.events, ev.size());
    EXPECT_EQ(got.stats.late_events, 0u);
    EXPECT_EQ(got.stats.ticks, expected.size());
  }
}

TEST(Concurrency, ClosedProducerIsALostFeedAndOthersCarryOn) {
  const Config cfg = repo_config();
  ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, by_approach());
  std::vector<std::thread> threads;
  {
    auto west = rt.producer(3);
    west.close();  // W approach feed dies before sending anything
  }
  for (std::size_t p = 0; p < 3; ++p) {
    threads.emplace_back([&rt, p]() {
      auto h = rt.producer(p);
      for (TimeMs t = 0; t <= s(120); t += 500) h.advance(t);
    });  // handle destroyed at scope exit = closed
  }
  std::size_t ticks = 0;
  while (rt.next_output()) ++ticks;
  for (auto& t : threads) t.join();
  const auto& pipe = rt.join();
  EXPECT_GE(ticks, 1u);
  EXPECT_LE(ticks, 241u);
  const auto& act = dynamic_cast<const ActuatedController&>(pipe.controller());
  for (DetectorId d : {9u, 10u, 11u, 15u}) {
    EXPECT_TRUE(act.monitor().faulty(d)) << d;
  }
  ASSERT_FALSE(act.monitor().faults().empty());
  EXPECT_EQ(act.monitor().faults()[0].kind, FaultKind::FeedLost);
  EXPECT_EQ(act.effective_recall(EW_R), Recall::Max);
  EXPECT_EQ(pipe.guard().refusals(), 0u);
}

// The same messages from each producer, delivered in two different interleavings: the north
// feed closes either before or after the other feeds have advanced. The lost-feed fault must be
// declared at the first tick after the north feed's last watermark in both cases, and the
// north feed's events before that watermark must still count.
TEST(Concurrency, ClosedProducerGivesTheSameResultWhateverTheArrivalOrder) {
  const Config cfg = repo_config();
  const TimeMs end = s(60), north_last = s(30);
  const std::vector<DetectorEvent> north = {{s(3), 0, true}, {s(3.5), 0, false}, {s(5), 2, true}, {s(5.5), 2, false}};

  auto run = [&](bool north_first) {
    ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, by_approach());
    std::vector<ControllerRuntime::Producer> h;
    for (std::size_t p = 0; p < 4; ++p) h.push_back(rt.producer(p));
    auto north_feed = [&] {
      for (const auto& e : north) h[0].emit(e);
      h[0].advance(north_last);
      h[0].close();
    };
    if (north_first) north_feed();
    for (std::size_t p = 1; p < 4; ++p) h[p].advance(end);
    if (!north_first) north_feed();
    for (std::size_t p = 1; p < 4; ++p) h[p].close();
    std::vector<TickRecord> out;
    while (auto r = rt.next_output()) out.push_back(std::move(*r));
    const auto& act = dynamic_cast<const ActuatedController&>(rt.join().controller());
    return std::make_pair(out, act.monitor().faults());
  };

  // Reference: one thread, the fault declared just before the tick after the last watermark.
  Pipeline ref(cfg, std::make_unique<ActuatedController>(cfg, 0), 0);
  std::vector<TickRecord> expected;
  std::size_t next = 0;
  for (TimeMs t = 0; t <= end; t += cfg.tick) {
    if (t == north_last + cfg.tick) {
      for (DetectorId d : {0u, 1u, 2u, 12u}) ref.controller().detector_feed_lost(d, t);  // north loops
    }
    while (next < north.size() && north[next].t <= t) ref.on_event(north[next++]);
    expected.push_back(ref.tick(t));
  }

  for (bool north_first : {true, false}) {
    const auto [out, faults] = run(north_first);
    ASSERT_EQ(out.size(), expected.size()) << "north_first=" << north_first;
    for (std::size_t i = 0; i < expected.size(); ++i) {
      ASSERT_EQ(out[i].displayed, expected[i].displayed) << "north_first=" << north_first << " tick " << i;
      ASSERT_EQ(out[i].commanded, expected[i].commanded) << "north_first=" << north_first << " tick " << i;
    }
    ASSERT_EQ(faults.size(), 4u);
    for (const auto& f : faults) {
      EXPECT_EQ(f.kind, FaultKind::FeedLost);
      EXPECT_EQ(f.at, north_last + cfg.tick);
    }
  }
  // The north turn-across vehicle at 5 s was seen before the feed died, so NS_right is served
  // at 17.0 s, before the fault at 30.5 s puts it on recall.
  EXPECT_EQ(expected[static_cast<std::size_t>(s(17) / cfg.tick)].displayed[G_NR], Signal::Green);
}

TEST(Concurrency, DisconnectAfterStuckOffEntersFailSafeAtTheNextTick) {
  const Config cfg = repo_config();
  const TimeMs last_feed = s(900), end = s(904);
  for (const bool disconnected_first : {true, false}) {
    SCOPED_TRACE(disconnected_first);
    ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, by_approach());
    std::vector<ControllerRuntime::Producer> h;
    for (std::size_t p = 0; p < 4; ++p) h.push_back(rt.producer(p));
    auto north_south = [&] {
      for (std::size_t p = 0; p < 2; ++p) {
        const DetectorId detector = cfg.detector_index(p == 0 ? "N_1" : "S_1");
        for (TimeMs t = p == 0 ? s(1) : s(4); t < last_feed; t += s(20)) {
          h[p].emit({t, detector, true});
          h[p].emit({t + cfg.tick, detector, false});
        }
        h[p].advance(last_feed);
        h[p].close();
      }
    };
    if (disconnected_first) north_south();
    h[2].advance(end);
    h[3].advance(end);
    if (!disconnected_first) north_south();
    // E/W vouch for the entire run before closing; they must not be marked lost
    // during the N/S failure, regardless of the order those messages arrived.
    h[2].close();
    h[3].close();
    std::vector<TickRecord> out;
    while (auto r = rt.next_output()) out.push_back(std::move(*r));
    const auto& pipe = rt.join();
    const auto& monitor = dynamic_cast<const ActuatedController&>(pipe.controller()).monitor();

    ASSERT_EQ(out.size(), static_cast<std::size_t>(end / cfg.tick) + 1);
    EXPECT_EQ(out.back().t, end);
    for (const auto& r : out) EXPECT_EQ(r.failsafe, r.t > last_feed) << r.t;
    EXPECT_EQ(out.back().displayed, SignalVector(cfg.groups.size(), Signal::Red));
    EXPECT_EQ(pipe.guard().failsafe_reason(), "too many faulty detectors");
    EXPECT_EQ(pipe.guard().refusals(), 0u);
    EXPECT_EQ(rt.stats().late_events, 0u);
    EXPECT_EQ(monitor.faulty_count(), 8u);
    EXPECT_EQ(monitor.faulty_count(FaultKind::FeedLost), 8u);
    ASSERT_EQ(monitor.faults().size(), 14u);
    std::vector<DetectorId> stuck_off, feed_lost;
    for (const auto& fault : monitor.faults()) {
      if (fault.kind == FaultKind::StuckOff) {
        EXPECT_EQ(fault.at, last_feed);
        stuck_off.push_back(fault.detector);
      } else {
        EXPECT_EQ(fault.kind, FaultKind::FeedLost);
        EXPECT_EQ(fault.at, last_feed + cfg.tick);
        feed_lost.push_back(fault.detector);
      }
    }
    std::sort(stuck_off.begin(), stuck_off.end());
    std::sort(feed_lost.begin(), feed_lost.end());
    EXPECT_EQ(stuck_off, (std::vector<DetectorId>{0, 2, 3, 5, 12, 13}));
    EXPECT_EQ(feed_lost, (std::vector<DetectorId>{0, 1, 2, 3, 4, 5, 12, 13}));
  }
}

TEST(Concurrency, StopUnblocksEverything) {
  const Config cfg = repo_config();
  ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, by_approach(), 1);
  std::atomic<bool> go{true};
  std::vector<std::thread> threads;
  for (std::size_t p = 0; p < 4; ++p) {
    threads.emplace_back([&rt, &go, p]() {
      auto h = rt.producer(p);
      // Producer 0 never advances, so the controller can never run a tick; producers 1-3 keep
      // pushing into a one-slot inbox (and block on it) until stop() closes it.
      for (TimeMs t = 1; go.load(); ++t) {
        if (p != 0) h.advance(t);
        std::this_thread::yield();
      }
    });
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  rt.stop();
  go = false;
  for (auto& t : threads) t.join();
  rt.join();
  EXPECT_FALSE(rt.next_output().has_value());  // no tick was ever executed, and none is pending
}

TEST(Concurrency, ProducerContractIsEnforced) {
  const Config cfg = repo_config();
  EXPECT_THROW(ControllerRuntime(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, {{0, 1, 2}}),
               std::invalid_argument);  // detectors 3-11 unowned
  EXPECT_THROW(ControllerRuntime(cfg, std::make_unique<ActuatedController>(cfg, 0), 0,
                                 {{0, 1, 2, 3}, {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}}),
               std::invalid_argument);  // detector 3 owned twice
  ControllerRuntime rt(cfg, std::make_unique<ActuatedController>(cfg, 0), 0, by_approach());
  auto h = rt.producer(1);
  EXPECT_THROW(rt.producer(1), std::logic_error);
  EXPECT_THROW(rt.producer(9), std::out_of_range);
  EXPECT_THROW(h.emit({1000, 0, true}), std::invalid_argument);  // detector 0 belongs to producer 0
  h.advance(2000);
  EXPECT_THROW(h.emit({2000, 3, true}), std::invalid_argument);  // not after the watermark
  EXPECT_NO_THROW(h.emit({2500, 3, true}));
  h.close();
  EXPECT_THROW(h.emit({3000, 3, false}), std::logic_error);
  rt.stop();
}

TEST(BlockingQueue, ManyProducersManyConsumersLoseNothing) {
  BlockingQueue<int> q(16);
  constexpr int kPerProducer = 20000;
  std::atomic<long long> sum{0};
  std::atomic<int> count{0};
  std::vector<std::thread> consumers, producers;
  for (int c = 0; c < 3; ++c) {
    consumers.emplace_back([&] {
      while (auto v = q.pop()) {
        sum += *v;
        ++count;
      }
    });
  }
  for (int p = 0; p < 4; ++p) {
    producers.emplace_back([&q, p] {
      for (int i = 1; i <= kPerProducer; ++i) ASSERT_TRUE(q.push(p * kPerProducer + i));
    });
  }
  for (auto& t : producers) t.join();
  q.close();
  for (auto& t : consumers) t.join();
  const long long n = 4LL * kPerProducer;
  EXPECT_EQ(count.load(), n);
  EXPECT_EQ(sum.load(), n * (n + 1) / 2);
  EXPECT_FALSE(q.push(1));
  EXPECT_TRUE(q.closed());
}

TEST(Concurrency, SimultaneousEventsHaveAFixedOrder) {
  // time first, then detector, then a zero-length pulse is applied as "on" before "off".
  EXPECT_TRUE(event_order({1000, 5, false}, {1500, 0, true}));
  EXPECT_TRUE(event_order({1000, 2, false}, {1000, 3, true}));
  EXPECT_TRUE(event_order({1000, 2, true}, {1000, 2, false}));
  EXPECT_FALSE(event_order({1000, 2, false}, {1000, 2, true}));
  EXPECT_FALSE(event_order({1000, 2, true}, {1000, 2, true}));
}
