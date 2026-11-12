#ifndef HUMANOID_PERCEPTION__CAMERA_SUBSCRIBER_NODE_HPP_
#define HUMANOID_PERCEPTION__CAMERA_SUBSCRIBER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "humanoid_infrastructure/lockfree_queue.hpp"

namespace humanoid_perception
{

class CameraSubscriberNode : public rclcpp::Node
{
public:
  explicit CameraSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  humanoid_infrastructure::SPSCQueue<cv::Mat, 64> frame_queue_;
};

}  // namespace humanoid_perception

#endif  // HUMANOID_PERCEPTION__CAMERA_SUBSCRIBER_NODE_HPP_
