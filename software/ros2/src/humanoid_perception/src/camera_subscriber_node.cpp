#include "humanoid_perception/camera_subscriber_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

namespace humanoid_perception
{

CameraSubscriberNode::CameraSubscriberNode(const rclcpp::NodeOptions & options)
: Node("humanoid_camera_subscriber", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Humanoid RGB-D Camera Subscriber Node...");

  // Subscribe to head RGB and Depth streams
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  rgb_sub_.subscribe(this, "/humanoid/head_camera/image_raw", qos);
  depth_sub_.subscribe(this, "/humanoid/head_camera/depth/image_raw", qos);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), rgb_sub_, depth_sub_);
  sync_->registerCallback(
    std::bind(&CameraSubscriberNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/humanoid/perception/debug_view", 10);

  RCLCPP_INFO(this->get_logger(), "RGB-D Synchronization active on /humanoid/head_camera topics.");
}

void CameraSubscriberNode::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  cv_bridge::CvImageConstPtr cv_rgb_ptr;
  cv_bridge::CvImageConstPtr cv_depth_ptr;

  try {
    cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    cv_depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Push frame into lock-free SPSC buffer for zero-blocking perception ingestion
  frame_queue_.push(cv_rgb_ptr->image.clone());

  // Republish visualization / telemetry
  if (debug_img_pub_->get_subscription_count() > 0) {
    debug_img_pub_->publish(*rgb_msg);
  }
}

}  // namespace humanoid_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_perception::CameraSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
