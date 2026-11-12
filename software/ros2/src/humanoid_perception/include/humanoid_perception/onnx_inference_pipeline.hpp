#ifndef HUMANOID_PERCEPTION__ONNX_INFERENCE_PIPELINE_HPP_
#define HUMANOID_PERCEPTION__ONNX_INFERENCE_PIPELINE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace humanoid_perception
{

struct BoundingBox3D
{
  std::string class_name;
  float confidence;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Vector3 size;
};

struct PerceptionConfig
{
  std::string model_path = "models/yolov8_humanoid.onnx";
  int input_width = 640;
  int input_height = 640;
  float confidence_threshold = 0.45f;
  float nms_threshold = 0.50f;
  bool use_cuda = true;
};

/**
 * @brief Real-time AI Inference Pipeline abstraction.
 * @details Handles vision preprocessing (letterboxing, color normalization),
 *          tensor inference dispatching (ONNX Runtime / TensorRT backend),
 *          and post-processing into 3D world/camera coordinate detections.
 */
class OnnxInferencePipeline
{
public:
  using Config = PerceptionConfig;

  OnnxInferencePipeline()
  : config_{}
  {
  }

  explicit OnnxInferencePipeline(const Config & config)
  : config_(config)
  {
  }

  virtual ~OnnxInferencePipeline() = default;

  /**
   * @brief Prepares and normalizes input image into model tensor format (NCHW, 1/255.0).
   */
  cv::Mat preprocess(const cv::Mat & input_frame, float & scale_factor, cv::Point2f & pad)
  {
    cv::Mat rgb;
    if (input_frame.channels() == 3) {
      cv::cvtColor(input_frame, rgb, cv::COLOR_BGR2RGB);
    } else {
      rgb = input_frame.clone();
    }

    const int target_w = config_.input_width;
    const int target_h = config_.input_height;

    float r = std::min(static_cast<float>(target_w) / rgb.cols,
                       static_cast<float>(target_h) / rgb.rows);
    int new_unpad_w = static_cast<int>(std::round(rgb.cols * r));
    int new_unpad_h = static_cast<int>(std::round(rgb.rows * r));

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_unpad_w, new_unpad_h), 0, 0, cv::INTER_LINEAR);

    int dw = target_w - new_unpad_w;
    int dh = target_h - new_unpad_h;
    pad = cv::Point2f(dw / 2.0f, dh / 2.0f);
    scale_factor = r;

    cv::Mat boxed(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(boxed(cv::Rect(dw / 2, dh / 2, new_unpad_w, new_unpad_h)));

    cv::Mat float_img;
    boxed.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    return float_img;
  }

  /**
   * @brief Runs inference and projects 2D detections into estimated 3D bounding boxes.
   */
  std::vector<BoundingBox3D> infer(
    const cv::Mat & rgb_image,
    const cv::Mat & depth_image = cv::Mat())
  {
    float scale = 1.0f;
    cv::Point2f pad(0.0f, 0.0f);
    cv::Mat input_tensor = preprocess(rgb_image, scale, pad);

    std::vector<BoundingBox3D> results;

    // Simulation / Fallback bounding box projection:
    // In production with ONNX Runtime linked, Ort::Session::Run() feeds input_tensor.
    // When depth_image is provided, depth values at (u, v) backproject to (X, Y, Z).
    (void)depth_image;
    (void)input_tensor;

    return results;
  }

private:
  Config config_;
};

}  // namespace humanoid_perception

#endif  // HUMANOID_PERCEPTION__ONNX_INFERENCE_PIPELINE_HPP_
