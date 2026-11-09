// Supervisor: checks that the control task really completes every cycle on time, owns the decision to
// power the servos (OE) and gates the hardware watchdog. No hardware access: the supervisor task feeds it
// a monotonic microsecond clock and applies the Decision.
//
// Two writers, each owning its own fields:
//   control task     -> control_completed(), set_oe_request(), request_clear()
//   supervisor task  -> step()
// All shared fields are 32-bit atomics, so neither side ever blocks the other.
//
// What counts as a valid cycle: it finished within the deadline of its own release, and its release follows the
// previous one by a whole number of periods (no repeated, backwards or off-grid releases). control_completed()
// applies that test to every completion as it happens, so a bad cycle is caught even when the next one completes
// before the supervisor's next step. Independently, step() checks on its own clock that the next completion
// arrives in time (stall). A bare heartbeat is never enough: only valid completions keep OE on and the IWDG fed.
//
// Watchdog policy: one invalid cycle latches the fault (OE off) but the MCU keeps running, so it can report and
// recover through ClearFault. The IWDG refresh stops when control is stalled (no completion by the next
// deadline) or when invalid cycles persist (max_bad_run in a row): a reset is then the only way back.
#pragma once
#include <atomic>
#include <stdint.h>

namespace mico {

struct SupervisorConfig {
  uint32_t period_us = 5000;       // control period
  uint32_t deadline_us = 5000;     // each cycle must finish within this of its release
  uint32_t boot_grace_us = 500000; // first control cycle must complete within this after start
  uint32_t max_bad_run = 20;       // this many invalid cycles in a row (100 ms) count as unhealthy: no refresh
};

class Supervisor {
 public:
  struct Decision {
    bool oe_legs;              // drive the PCA9685 #1 outputs
    bool oe_arms;              // drive the PCA9685 #2 outputs
    bool refresh_watchdog;     // the control task is alive: keep the MCU running
    bool latched;              // a deadline fault is latched: outputs stay off until cleared
  };

  explicit Supervisor(const SupervisorConfig& cfg = SupervisorConfig()) : cfg_(cfg) {}

  // Supervisor task, once at start-up.
  void begin(uint32_t now_us);

  // ---- control task side
  // Call at the end of every cycle, after the outputs are written. release_us is when the cycle was due.
  void control_completed(uint32_t release_us, uint32_t end_us);
  void set_oe_request(bool legs, bool arms) { oe_req_.store((legs ? 1u : 0u) | (arms ? 2u : 0u)); }
  // The host cleared the fault on the controller: the supervisor clears its latch if control is healthy.
  void request_clear() { clear_req_.store(1); }

  // ---- supervisor task side
  Decision step(uint32_t now_us);

  // ---- readable from any task
  bool latched() const { return latched_.load() != 0; }
  bool healthy() const { return healthy_.load() != 0; }
  bool clear_pending() const { return clear_req_.load() != 0; }
  uint32_t latch_count() const { return latch_count_.load(); }
  uint32_t completions() const { return seq_.load(); }
  uint32_t bad_completions() const { return bad_.load(); }
  // How long after the violated deadline the latest latch happened (outputs went off in the same step), us.
  uint32_t last_latch_delay_us() const { return latch_delay_.load(); }
  // Outputs as last decided by step(): bit 0 legs, bit 1 arms.
  uint32_t oe_applied() const { return oe_applied_.load(); }

 private:
  void latch(uint32_t now_us, uint32_t violated_deadline_us);

  SupervisorConfig cfg_;
  // written by the control task
  std::atomic<uint32_t> seq_{0};
  std::atomic<uint32_t> release_{0};
  std::atomic<uint32_t> end_{0};
  std::atomic<uint32_t> oe_req_{0};
  std::atomic<uint32_t> clear_req_{0};
  std::atomic<uint32_t> bad_{0};             // invalid completions (late, or off the release grid)
  std::atomic<uint32_t> bad_deadline_{0};    // the deadline the latest invalid completion broke
  std::atomic<uint32_t> bad_run_{0};         // invalid completions in a row, 0 after a valid one
  uint32_t prev_release_ = 0;                // control-task side only
  bool have_prev_ = false;
  // written by the supervisor task
  std::atomic<uint32_t> latched_{0};
  std::atomic<uint32_t> healthy_{0};
  std::atomic<uint32_t> latch_count_{0};
  std::atomic<uint32_t> latch_delay_{0};
  std::atomic<uint32_t> oe_applied_{0};
  uint32_t start_us_ = 0;
  uint32_t seen_bad_ = 0;
};

}  // namespace mico
