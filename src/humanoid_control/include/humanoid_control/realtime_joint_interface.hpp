#ifndef HUMANOID_CONTROL__REALTIME_JOINT_INTERFACE_HPP_
#define HUMANOID_CONTROL__REALTIME_JOINT_INTERFACE_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace humanoid_control
{

struct JointLimits
{
  std::string name;
  double min_position;
  double max_position;
  double max_velocity;
  double max_effort;
};

/**
 * @brief Safety enforcement wrapper for humanoid joint commands prior to hardware/simulator injection.
 */
class RealtimeJointInterface
{
public:
  RealtimeJointInterface() = default;

  void addJointLimit(const JointLimits & limit)
  {
    limits_.push_back(limit);
  }

  double clampPosition(size_t index, double target_pos) const
  {
    if (index >= limits_.size()) {
      return target_pos;
    }
    return std::clamp(target_pos, limits_[index].min_position, limits_[index].max_position);
  }

  double clampVelocity(size_t index, double current_pos, double target_pos, double dt) const
  {
    if (index >= limits_.size() || dt <= 0.0) {
      return target_pos;
    }
    double max_delta = limits_[index].max_velocity * dt;
    return std::clamp(target_pos, current_pos - max_delta, current_pos + max_delta);
  }

private:
  std::vector<JointLimits> limits_;
};

}  // namespace humanoid_control

#endif  // HUMANOID_CONTROL__REALTIME_JOINT_INTERFACE_HPP_
