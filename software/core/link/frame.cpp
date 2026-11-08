#include "frame.h"

#include <string.h>

namespace mico {

uint16_t crc16(const uint8_t* data, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : crc << 1;
  }
  return crc;
}

size_t cobs_encode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t code_at = 0, o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < n; ++i) {
    if (in[i] == 0) {
      out[code_at] = code;
      code_at = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFF) {
        out[code_at] = code;
        code_at = o++;
        code = 1;
      }
    }
  }
  out[code_at] = code;
  return o;
}

size_t cobs_decode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t i = 0, o = 0;
  while (i < n) {
    uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > n) return 0;
    for (uint8_t k = 1; k < code; ++k) {
      if (in[i] == 0) return 0;
      out[o++] = in[i++];
    }
    if (code != 0xFF && i < n) out[o++] = 0;
  }
  return o;
}

size_t frame_build(uint8_t type, uint8_t seq, const void* payload, size_t n, uint8_t* out, size_t cap) {
  uint8_t raw[MAX_FRAME + 4];
  if (n + 4 > sizeof(raw) || cap < n + 4 + (n + 4) / 254 + 2) return 0;
  raw[0] = type;
  raw[1] = seq;
  if (n) memcpy(raw + 2, payload, n);
  uint16_t c = crc16(raw, n + 2);
  raw[n + 2] = static_cast<uint8_t>(c & 0xFF);
  raw[n + 3] = static_cast<uint8_t>(c >> 8);
  size_t m = cobs_encode(raw, n + 4, out);
  out[m] = 0;
  return m + 1;
}

bool FrameParser::push(uint8_t b, Frame& frame) {
  if (b != 0) {
    if (n_ < sizeof(raw_)) {
      raw_[n_++] = b;
    } else {
      overflow_ = true;
    }
    return false;
  }
  size_t n = n_;
  bool overflow = overflow_;
  n_ = 0;
  overflow_ = false;
  if (n == 0) return false;                       // empty frame: just a delimiter
  size_t m = overflow ? 0 : cobs_decode(raw_, n, dec_);
  if (m < 4) {
    ++errors_;
    return false;
  }
  uint16_t c = static_cast<uint16_t>(dec_[m - 2] | (dec_[m - 1] << 8));
  if (crc16(dec_, m - 2) != c) {
    ++errors_;
    return false;
  }
  frame.type = dec_[0];
  frame.seq = dec_[1];
  frame.payload = dec_ + 2;
  frame.len = m - 4;
  return true;
}

}  // namespace mico
