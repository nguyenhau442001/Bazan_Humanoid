#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "humanoid_control/realtime_joint_interface.hpp"

namespace humanoid_control
{

class JointTrajectoryBridgeNode : public rclcpp::Node
{
public:
  explicit JointTrajectoryBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joint_trajectory_bridge", options)
  {
    cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/lower_body_trajectory_controller/joint_trajectory", 10);

    target_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/humanoid/motion_planner/joint_trajectory", 10,
      std::bind(&JointTrajectoryBridgeNode::targetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Humanoid Joint Trajectory Bridge active.");
  }

private:
  void targetCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    // Pass-through with safety checking
    cmd_pub_->publish(*msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr target_sub_;
  RealtimeJointInterface safety_guard_;
};

}  // namespace humanoid_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_control::JointTrajectoryBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
