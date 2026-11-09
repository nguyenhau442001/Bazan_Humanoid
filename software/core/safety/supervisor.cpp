#include "supervisor.h"

namespace mico {

namespace {
bool after(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) > 0; }   // a later than b, wrap-safe
}  // namespace

void Supervisor::begin(uint32_t now_us) {
  start_us_ = now_us;
  seen_bad_ = bad_.load();
  healthy_.store(1);
}

void Supervisor::control_completed(uint32_t release_us, uint32_t end_us) {
  uint32_t deadline = release_us + cfg_.deadline_us;
  bool late = after(end_us, deadline) || after(release_us, end_us);
  // Releases sit on a 5 ms grid: the step must be a positive whole number of periods (wrap-safe in uint32).
  uint32_t step = release_us - prev_release_;
  bool off_grid = have_prev_ && (step == 0 || step >= 0x80000000u || step % cfg_.period_us != 0);
  prev_release_ = release_us;
  have_prev_ = true;
  if (late || off_grid) {
    bad_deadline_.store(late ? deadline : end_us, std::memory_order_relaxed);
    bad_run_.store(bad_run_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    bad_.store(bad_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
  } else {
    bad_run_.store(0, std::memory_order_relaxed);
  }
  // Publish timestamps first; the sequence number is the commit, read first by step().
  release_.store(release_us, std::memory_order_relaxed);
  end_.store(end_us, std::memory_order_relaxed);
  seq_.store(seq_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
}

void Supervisor::latch(uint32_t now_us, uint32_t violated_deadline_us) {
  if (!latched_.load()) {
    latch_count_.store(latch_count_.load() + 1);
    latch_delay_.store(after(now_us, violated_deadline_us) ? now_us - violated_deadline_us : 0);
  }
  latched_.store(1);
}

Supervisor::Decision Supervisor::step(uint32_t now_us) {
  uint32_t seq = seq_.load(std::memory_order_acquire);
  uint32_t release = release_.load(std::memory_order_relaxed);

  bool alive;
  if (seq == 0) {
    alive = !after(now_us, start_us_ + cfg_.boot_grace_us);          // waiting for the first cycle
    if (!alive) latch(now_us, start_us_ + cfg_.boot_grace_us);
  } else {
    uint32_t bad = bad_.load(std::memory_order_acquire);
    if (bad != seen_bad_) {                                         // a cycle finished late or off-grid
      latch(now_us, bad_deadline_.load(std::memory_order_relaxed));
      seen_bad_ = bad;
    }
    // The next cycle was due at release + period and must be done by then + deadline. A release committed
    // while we read is only newer, which moves this deadline later, never earlier: no false stall.
    uint32_t next_deadline = release + cfg_.period_us + cfg_.deadline_us;
    alive = !after(now_us, next_deadline);
    if (!alive) latch(now_us, next_deadline);                       // stalled: no completion in time
    if (bad_run_.load(std::memory_order_relaxed) >= cfg_.max_bad_run) alive = false;   // persistently invalid
  }
  healthy_.store(alive ? 1 : 0);

  if (clear_req_.load()) {
    if (alive && seq != 0) latched_.store(0);                     // refused while unhealthy: stays latched
    clear_req_.store(0);
  }

  uint32_t req = oe_req_.load();
  bool on = alive && !latched_.load() && seq != 0;
  Decision d{on && (req & 1u), on && (req & 2u), alive, latched_.load() != 0};
  oe_applied_.store((d.oe_legs ? 1u : 0u) | (d.oe_arms ? 2u : 0u));
  return d;
}

}  // namespace mico
