#include "trajectory.h"

#include "conversions.h"

namespace mico {

void Trajectory::reset(const float* q, uint32_t now_ms) {
  head_ = count_ = 0;
  for (int j = 0; j < NJ; ++j) q_from_[j] = clampf(q[j], Q_MIN[j], Q_MAX[j]);
  t_from_ = t_last_end_ = now_ms;
}

bool Trajectory::push(uint16_t dt_ms, const float* q, uint32_t now_ms) {
  if (count_ == TRAJ_QUEUE) return false;
  if (count_ == 0 && static_cast<int32_t>(now_ms - t_last_end_) > 0) t_from_ = t_last_end_ = now_ms;
  Point& p = pts_[(head_ + count_) % TRAJ_QUEUE];
  p.t_end = t_last_end_ + dt_ms;
  for (int j = 0; j < NJ; ++j) p.q[j] = clampf(q[j], Q_MIN[j], Q_MAX[j]);
  t_last_end_ = p.t_end;
  ++count_;
  return true;
}

void Trajectory::sample(uint32_t now_ms, float* q_out) {
  // Retire every point whose end time has passed; its target becomes the next segment's start.
  while (count_ > 0 && static_cast<int32_t>(now_ms - pts_[head_].t_end) >= 0) {
    for (int j = 0; j < NJ; ++j) q_from_[j] = pts_[head_].q[j];
    t_from_ = pts_[head_].t_end;
    head_ = (head_ + 1) % TRAJ_QUEUE;
    --count_;
  }
  if (count_ == 0) {
    for (int j = 0; j < NJ; ++j) q_out[j] = q_from_[j];
    return;
  }
  const Point& p = pts_[head_];
  uint32_t span = p.t_end - t_from_;
  float u = span ? static_cast<float>(static_cast<int32_t>(now_ms - t_from_)) / span : 1.0f;
  u = clampf(u, 0.0f, 1.0f);
  for (int j = 0; j < NJ; ++j) q_out[j] = q_from_[j] + u * (p.q[j] - q_from_[j]);
}

bool Trajectory::active(uint32_t now_ms) const {
  return count_ > 0 && static_cast<int32_t>(pts_[(head_ + count_ - 1) % TRAJ_QUEUE].t_end - now_ms) > 0;
}

}  // namespace mico
