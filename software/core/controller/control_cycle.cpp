#include "control_cycle.h"

namespace mico {

void ControlCycle::run(Inputs& in, const uint8_t* rx, size_t n, Outputs& out) {
  in.supervisor_healthy = sup_.healthy();
  // Host messages see the previous cycle's inputs (ClearFault is validated against them).
  ctrl_.on_bytes(rx, n, in.now_ms);
  if (ctrl_.take_clear_request()) sup_.request_clear();

  // Mirror the supervisor's latch into the controller, so the host sees CONTROL_DEADLINE in State and
  // the controller stops commanding. A new latch always raises it; a latch the supervisor refused to clear
  // raises it again once the clear request has been processed.
  uint32_t latches = sup_.latch_count();
  bool reported = ctrl_.faults() & F_CONTROL_DEADLINE;
  if (latches != seen_latches_ || (sup_.latched() && !sup_.clear_pending() && !reported)) {
    seen_latches_ = latches;
    ctrl_.raise_fault(F_CONTROL_DEADLINE, in.now_ms);
  }

  ctrl_.tick(in, out);
  // The controller only requests outputs; the supervisor task owns the OE pins.
  sup_.set_oe_request(out.oe_legs, out.oe_arms);
}

}  // namespace mico
