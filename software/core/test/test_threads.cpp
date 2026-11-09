// The lock-free ring and the seqlock with two real threads, run under ThreadSanitizer.
#include <stdio.h>

#include <atomic>
#include <thread>

#include "byte_ring.h"
#include "seqlock.h"

using namespace mico;

int main() {
  int failures = 0;

  // Producer writes a counting byte stream in random-sized chunks; the consumer checks every byte.
  static ByteRing<256> ring;
  constexpr uint32_t TOTAL = 2000000;
  std::thread prod([] {
    uint32_t sent = 0, seed = 1;
    uint8_t buf[40];
    while (sent < TOTAL) {
      seed = seed * 1103515245u + 12345u;
      uint32_t n = 1 + (seed >> 16) % 40;
      if (n > TOTAL - sent) n = TOTAL - sent;
      for (uint32_t i = 0; i < n; ++i) buf[i] = static_cast<uint8_t>(sent + i);
      if (ring.write(buf, n)) sent += n;
    }
  });
  uint32_t got = 0;
  uint8_t buf[64];
  while (got < TOTAL) {
    size_t n = ring.read(buf, sizeof(buf));
    for (size_t i = 0; i < n; ++i)
      if (buf[i] != static_cast<uint8_t>(got + i)) ++failures;
    got += n;
  }
  prod.join();

  // Writer publishes {k, 2k}; a reader must never see a torn pair.
  struct Pair {
    uint32_t a, b;
  };
  static Seqlock<Pair> lock;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (uint32_t k = 0; k < 500000; ++k) lock.write({k, 2 * k});
    done = true;
  });
  uint32_t reads = 0;
  while (!done) {
    Pair p;
    if (lock.read(p)) {
      ++reads;
      if (p.b != 2 * p.a) ++failures;
    }
  }
  writer.join();

  printf("threads: %u bytes streamed, %u snapshots read, %d failed\n", TOTAL, reads, failures);
  return failures ? 1 : 0;
}
