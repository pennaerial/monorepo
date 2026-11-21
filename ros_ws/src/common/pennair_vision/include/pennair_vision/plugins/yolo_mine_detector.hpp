#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// the ncnn CMake target puts <prefix>/include/ncnn on the include path
#include "net.h"
#include "pennair_vision/lazy_camera_publisher.hpp"
#include "pennair_vision/yolo_mine_detector_parameters.hpp"
#include "pennair_vision/yolo_postprocess.hpp"
#include "pennair_vision_msgs/msg/detection_array.hpp"

namespace pennair_vision
{

/// Runs a YOLO26 NCNN detector on camera frames (end-to-end NMS-free head, or one-to-many head with NMS; see the
/// output_format parameter).
/// Detections for every processed frame go to /cv/result. When LazyCameraPublisher requests a debug image (at most
/// debug_image_rate), the same frame is annotated with the detections from that same inference.
/// Inference runs synchronously in process(): the camera subscription keeps only the newest frame, so when inference is
/// slower than the camera, stale frames are dropped rather than queued.
class YoloMineDetector : public LazyCameraPublisher<pennair_vision_msgs::msg::DetectionArray>
{
public:
  /// Loads parameters and the model, then starts the camera subscription. If loading fails, logs an error and stays
  /// inactive so the other plugins keep running.
  void initialize(rclcpp::Node& node, const std::string& camera_topic, std::optional<double> debug_image_rate) override;

  /// See LazyCameraPublisher
  ProcessResult<pennair_vision_msgs::msg::DetectionArray>
  process(const CameraImage& image, const CameraInfo& info, bool generate_debug) override;

private:
  /// Loads the NCNN model; returns false and logs on failure.
  bool load_model(const rclcpp::Logger& logger);
  /// Builds the normalized RGB model input. Frames already at the model input size in a packed 8-bit encoding are
  /// read directly; others are resized/letterboxed. Returns false if the image is malformed or cannot be converted.
  bool preprocess(const CameraImage& image, ncnn::Mat& input, yolo::Letterbox& letterbox);
  /// Applies the model's input normalization in place.
  static void normalize(ncnn::Mat& input);
  /// Runs the network and decodes the output per output_format_ into final detections in model-input pixels
  /// (NMS applied for the one-to-many head only).
  std::optional<std::vector<yolo::Detection>> infer(const ncnn::Mat& input);
  /// Draws detections on a BGR copy of the frame.
  cv::Mat annotate(const CameraImage& image, const std::vector<yolo::Detection>& detections) const;
  /// Logs inference throughput every few seconds.
  void
  record_timing(std::chrono::duration<double, std::milli> inference, std::chrono::duration<double, std::milli> total);

  std::shared_ptr<yolo_mine_detector::ParamListener> param_listener_;
  yolo_mine_detector::Params params_;
  /// Parsed from params_.output_format at startup.
  yolo::OutputFormat output_format_{yolo::OutputFormat::End2End};

  /// Reuses preprocessing buffers between frames instead of allocating new ones.
  ncnn::PoolAllocator input_allocator_;
  ncnn::Net net_;
  /// Which preprocessing path the last frame took; logged when it changes.
  std::optional<bool> fast_path_active_;

  // throughput statistics, reset every log period
  std::chrono::steady_clock::time_point stats_start_{std::chrono::steady_clock::now()};
  int stats_frames_{0};
  double stats_inference_ms_{0.0};
  double stats_total_ms_{0.0};
};

};  // namespace pennair_vision
