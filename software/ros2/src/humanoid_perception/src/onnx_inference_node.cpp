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
#include "humanoid_perception/onnx_inference_pipeline.hpp"
#include "humanoid_interfaces/msg/detection3_d.hpp"
#include "humanoid_interfaces/msg/detection3_d_array.hpp"

namespace humanoid_perception
{

class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("humanoid_onnx_inference", options)
  {
    this->declare_parameter<std::string>("model_path", "models/yolov8_humanoid.onnx");
    this->declare_parameter<float>("confidence_threshold", 0.5f);
    this->declare_parameter<bool>("use_cuda", true);

    OnnxInferencePipeline::Config config;
    config.model_path = this->get_parameter("model_path").as_string();
    config.confidence_threshold = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
    config.use_cuda = this->get_parameter("use_cuda").as_bool();

    pipeline_ = std::make_unique<OnnxInferencePipeline>(config);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    rgb_sub_.subscribe(this, "/humanoid/head_camera/image_raw", qos);
    depth_sub_.subscribe(this, "/humanoid/head_camera/depth/image_raw", qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&OnnxInferenceNode::inferenceCallback, this, std::placeholders::_1, std::placeholders::_2));

    detections_pub_ = this->create_publisher<humanoid_interfaces::msg::Detection3DArray>(
      "/humanoid/perception/detections_3d", 10);

    RCLCPP_INFO(this->get_logger(), "ONNX Runtime / TensorRT Perception Pipeline ready.");
  }

private:
  void inferenceCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    auto start_time = std::chrono::steady_clock::now();

    cv_bridge::CvImageConstPtr cv_rgb_ptr;
    cv_bridge::CvImageConstPtr cv_depth_ptr;

    try {
      cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
      cv_depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    auto boxes = pipeline_->infer(cv_rgb_ptr->image, cv_depth_ptr->image);

    humanoid_interfaces::msg::Detection3DArray out_msg;
    out_msg.header = rgb_msg->header;
    for (const auto & b : boxes) {
      humanoid_interfaces::msg::Detection3D det;
      det.header = rgb_msg->header;
      det.class_name = b.class_name;
      det.confidence = b.confidence;
      det.position = b.position;
      det.size = b.size;
      out_msg.detections.push_back(det);
    }

    detections_pub_->publish(out_msg);

    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_DEBUG(this->get_logger(), "Inference cycle: %ld ms, detections: %zu",
                 elapsed_ms, out_msg.detections.size());
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

  std::unique_ptr<OnnxInferencePipeline> pipeline_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<humanoid_interfaces::msg::Detection3DArray>::SharedPtr detections_pub_;
};

}  // namespace humanoid_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_perception::OnnxInferenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
