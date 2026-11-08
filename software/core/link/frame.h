// Framing for the MCU <-> host link: COBS(type, seq, payload, crc16) followed by a 0x00 delimiter.
// CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, seq and payload, sent little-endian.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "config.h"

namespace mico {

uint16_t crc16(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF);
size_t cobs_encode(const uint8_t* in, size_t n, uint8_t* out);           // out needs n + n/254 + 1
size_t cobs_decode(const uint8_t* in, size_t n, uint8_t* out);           // 0 on malformed input

// Build a complete frame (COBS + trailing 0x00) into out. Returns its length, 0 if cap is too small.
size_t frame_build(uint8_t type, uint8_t seq, const void* payload, size_t n, uint8_t* out, size_t cap);

struct Frame {
  uint8_t type;
  uint8_t seq;
  const uint8_t* payload;
  size_t len;
};

class FrameParser {
 public:
  // Feed one byte. Returns true when a valid frame is ready in `frame` (valid until the next push).
  bool push(uint8_t b, Frame& frame);
  uint16_t errors() const { return errors_; }

 private:
  uint8_t raw_[MAX_FRAME + MAX_FRAME / 254 + 2];
  uint8_t dec_[MAX_FRAME + 4];
  size_t n_ = 0;
  bool overflow_ = false;
  uint16_t errors_ = 0;
};

}  // namespace mico
