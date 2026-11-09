// Joint trajectory buffer: timestamped points, linear interpolation at the loop rate.
// Each point says "reach q, dt_ms after the previous point". When the queue runs dry the last point is held.
#pragma once
#include <stdint.h>

#include "config.h"
#include "joint_table.h"

namespace mico {

class Trajectory {
 public:
  // Drop all points and hold q from `now_ms`.
  void reset(const float* q, uint32_t now_ms);
  // Append a point. If the queue had run dry, timing restarts from now_ms (from the held pose).
  // False if the queue is full.
  bool push(uint16_t dt_ms, const float* q, uint32_t now_ms);
  // Commanded joint angles at now_ms (clamped to the joint limits).
  void sample(uint32_t now_ms, float* q_out);
  // True while points remain ahead of now_ms.
  bool active(uint32_t now_ms) const;
  int queued() const { return count_; }

 private:
  struct Point {
    uint32_t t_end;
    float q[NJ];
  };
  Point pts_[TRAJ_QUEUE];
  int head_ = 0, count_ = 0;
  float q_from_[NJ] = {};      // start of the current segment
  uint32_t t_from_ = 0;
  uint32_t t_last_end_ = 0;    // end time of the last queued point
};

}  // namespace mico
