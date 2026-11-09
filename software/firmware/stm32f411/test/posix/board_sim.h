// Simulated board for the POSIX integration test: what the firmware drove, and the host side of the UART.
#pragma once
#include <atomic>
#include <stdint.h>

#include "byte_ring.h"

namespace sim {

extern std::atomic<int> oe_legs, oe_arms;          // last level the supervisor set (1 = outputs enabled)
extern std::atomic<int> oe_ever_on;                // OE was enabled at least once
extern std::atomic<uint32_t> refreshes;            // watchdog refreshes
extern std::atomic<int> watchdog_started;
extern bool watchdog_reset;                        // what reset_by_watchdog() reports (set before start)
extern mico::ByteRing<4096> host_to_mcu;           // test driver writes, comm task reads
extern mico::ByteRing<65536> mcu_to_host;          // comm task writes, test driver reads

}  // namespace sim
