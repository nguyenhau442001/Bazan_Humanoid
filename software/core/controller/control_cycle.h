// One control cycle: couples the Controller (modes, trajectory, safety) with the Supervisor (deadline,
// watchdog, OE ownership). The firmware's control task reads the inputs, calls run(), writes the PWM, then
// reports completion to the supervisor. Hardware-free, so the coupling is tested on the host.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "controller.h"
#include "supervisor.h"

namespace mico {

class ControlCycle {
 public:
  ControlCycle(Controller& ctrl, Supervisor& sup) : ctrl_(ctrl), sup_(sup) {}

  // rx: bytes received from the host since the previous cycle.
  void run(Inputs& in, const uint8_t* rx, size_t n, Outputs& out);

 private:
  Controller& ctrl_;
  Supervisor& sup_;
  uint32_t seen_latches_ = 0;
};

}  // namespace mico
