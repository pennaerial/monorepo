#include "pennair_vision/plugins/yolo_mine_detector.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <opencv2/imgproc.hpp>

#include "pennair_vision/vision_plugin.hpp"

namespace pennair_vision
{

namespace
{

constexpr auto kStatsPeriod = std::chrono::seconds(5);
/// Ultralytics letterbox padding color (gray 114 on every channel).
constexpr float kPadValue = 114.f;

/// NCNN pixel conversion that reads `encoding` directly as RGB, or nullopt if cv_bridge must convert it first.
std::optional<int> ncnn_pixel_type(const std::string& encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::RGB8) {
    return ncnn::Mat::PIXEL_RGB;
  }
  if (encoding == enc::BGR8) {
    return ncnn::Mat::PIXEL_BGR2RGB;
  }
  if (encoding == enc::RGBA8) {
    return ncnn::Mat::PIXEL_RGBA2RGB;
  }
  if (encoding == enc::BGRA8) {
    return ncnn::Mat::PIXEL_BGRA2RGB;
  }
  if (encoding == enc::MONO8) {
    return ncnn::Mat::PIXEL_GRAY2RGB;
  }
  return std::nullopt;
}

bool has_blob(const std::vector<const char*>& names, const std::string& name)
{
  return std::any_of(names.begin(), names.end(), [&](const char* n) { return name == n; });
}

cv::Scalar class_color(int class_id)
{
  static constexpr std::array<std::array<int, 3>, 6> kPalette{
      {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}, {255, 0, 255}, {255, 255, 0}}
  };
  const auto& c = kPalette[static_cast<std::size_t>(class_id) % kPalette.size()];
  return cv::Scalar(c[0], c[1], c[2]);
}

}  // namespace

void YoloMineDetector::initialize(
    rclcpp::Node& node,
    const std::string& camera_topic,
    std::optional<double> debug_image_rate
)
{
  const auto logger = node.get_logger();
  try {
    // parameters live under yolo_mine_detector.* on the VisionManager node
    param_listener_ = std::make_shared<yolo_mine_detector::ParamListener>(
        node.get_node_parameters_interface(), logger, "yolo_mine_detector."
    );
    params_ = param_listener_->get_params();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(logger, "YoloMineDetector: invalid parameters, plugin disabled: %s", ex.what());
    return;
  }

  const std::optional<yolo::OutputFormat> format = yolo::parse_output_format(params_.output_format);
  if (!format) {
    RCLCPP_ERROR(
        logger, "YoloMineDetector: output_format '%s' must be end2end or one_to_many, plugin disabled",
        params_.output_format.c_str()
    );
    return;
  }
  output_format_ = *format;

  if (!load_model(logger)) {
    RCLCPP_ERROR(logger, "YoloMineDetector: model failed to load, plugin disabled");
    return;
  }

  // sets up /cv/result, /cv/debug and the camera subscription
  LazyCameraPublisher::initialize(node, camera_topic, debug_image_rate);
  RCLCPP_INFO(
      logger, "YoloMineDetector: loaded %s (%s head, input %ld px, %ld threads)", params_.model_param_path.c_str(),
      params_.output_format.c_str(), static_cast<long>(params_.input_size), static_cast<long>(params_.num_threads)
  );
}

bool YoloMineDetector::load_model(const rclcpp::Logger& logger)
{
  for (const std::string& path : {params_.model_param_path, params_.model_bin_path}) {
    if (path.empty() || !std::filesystem::is_regular_file(path)) {
      RCLCPP_ERROR(
          logger,
          "YoloMineDetector: model file '%s' not found; set yolo_mine_detector.model_param_path and model_bin_path",
          path.c_str()
      );
      return false;
    }
  }

  net_.opt.num_threads = static_cast<int>(params_.num_threads);
  net_.opt.use_vulkan_compute = false;  // the Pi 4 GPU is slower than its CPU for this
  if (net_.load_param(params_.model_param_path.c_str()) != 0) {
    RCLCPP_ERROR(logger, "YoloMineDetector: failed to parse %s", params_.model_param_path.c_str());
    return false;
  }
  if (net_.load_model(params_.model_bin_path.c_str()) != 0) {
    RCLCPP_ERROR(logger, "YoloMineDetector: failed to load weights %s", params_.model_bin_path.c_str());
    return false;
  }
  if (!has_blob(net_.input_names(), params_.input_blob) || !has_blob(net_.output_names(), params_.output_blob)) {
    RCLCPP_ERROR(
        logger, "YoloMineDetector: model has no input blob '%s' or output blob '%s'", params_.input_blob.c_str(),
        params_.output_blob.c_str()
    );
    return false;
  }
  return true;
}

ProcessResult<pennair_vision_msgs::msg::DetectionArray>
YoloMineDetector::process(const CameraImage& image, const CameraInfo& /* info */, bool generate_debug)
{
  const auto start = std::chrono::steady_clock::now();
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();  // picks up runtime threshold changes
  }

  ProcessResult<pennair_vision_msgs::msg::DetectionArray> result;
  result.message.header = image->header;  // stamp and frame_id of the exact frame these detections came from
  result.message.image_width = image->width;
  result.message.image_height = image->height;

  ncnn::Mat input;
  yolo::Letterbox letterbox;
  if (!preprocess(image, input, letterbox)) {
    return result;
  }
  const auto inference_start = std::chrono::steady_clock::now();
  std::optional<std::vector<yolo::Detection>> inferred = infer(input);
  const auto inference_end = std::chrono::steady_clock::now();
  if (!inferred) {
    return result;
  }

  std::vector<yolo::Detection>& detections = *inferred;
  for (yolo::Detection& d : detections) {
    d.box = yolo::to_source_pixels(d.box, letterbox, static_cast<int>(image->width), static_cast<int>(image->height));
    pennair_vision_msgs::msg::Detection& msg = result.message.detections.emplace_back();
    msg.class_id = d.class_id;
    msg.score = d.score;
    msg.x_min = d.box.x_min;
    msg.y_min = d.box.y_min;
    msg.x_max = d.box.x_max;
    msg.y_max = d.box.y_max;
  }

  if (generate_debug) {
    result.debug_image = annotate(image, detections);
  }
  record_timing(inference_end - inference_start, std::chrono::steady_clock::now() - start);
  return result;
}

bool YoloMineDetector::preprocess(const CameraImage& image, ncnn::Mat& input, yolo::Letterbox& letterbox)
{
  const int input_size = static_cast<int>(params_.input_size);
  const int width = static_cast<int>(image->width);
  const int height = static_cast<int>(image->height);
  // identity (scale 1, no padding) when the frame already matches the model input
  letterbox = yolo::compute_letterbox(width, height, input_size);

  // Check the message really holds width x height pixels at its declared step before reading it. camera_ros sets
  // step to the libcamera stride, which may include row padding, so it is never assumed to be width * 3.
  std::uint32_t bytes_per_pixel = 0;
  try {
    bytes_per_pixel = static_cast<std::uint32_t>(
        sensor_msgs::image_encodings::numChannels(image->encoding) *
        sensor_msgs::image_encodings::bitDepth(image->encoding) / 8
    );
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "YoloMineDetector: unsupported encoding %s: %s",
        image->encoding.c_str(), ex.what()
    );
    return false;
  }
  if (!yolo::buffer_holds_image(image->data.size(), image->width, image->height, image->step, bytes_per_pixel)) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "YoloMineDetector: malformed %s image: %ux%u, step %u, %zu bytes", image->encoding.c_str(), image->width,
        image->height, image->step, image->data.size()
    );
    return false;
  }

  const unsigned char* pixels = image->data.data();
  int stride = static_cast<int>(image->step);
  std::optional<int> pixel_type = ncnn_pixel_type(image->encoding);

  // Fast path: the camera already delivers model-sized frames in a packed 8-bit encoding (e.g. libcamera RGB888,
  // which camera_ros publishes as bgr8). Build the CHW float tensor straight from the message, stepping rows by
  // `stride`; bgr8 is swapped to RGB inside this same pass. No resize, letterbox or cv_bridge conversion.
  const bool fast_path = pixel_type.has_value() && yolo::matches_model_input(width, height, input_size);
  if (fast_path_active_ != fast_path) {
    fast_path_active_ = fast_path;
    RCLCPP_INFO(
        node_->get_logger(), "YoloMineDetector: %s preprocessing for %ux%u %s frames (model input %d)",
        fast_path ? "direct (no resize)" : "resize/letterbox", image->width, image->height, image->encoding.c_str(),
        input_size
    );
  }
  if (fast_path) {
    input = ncnn::Mat::from_pixels(pixels, *pixel_type, width, height, stride, &input_allocator_);
    normalize(input);
    return !input.empty();
  }

  // Fallback: other resolutions are resized and letterboxed by NCNN. Encodings NCNN cannot read directly (e.g. YUYV)
  // are converted to RGB by cv_bridge first, costing one extra conversion.
  cv_bridge::CvImageConstPtr converted;  // keeps converted pixels alive until resized
  if (!pixel_type) {
    try {
      converted = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8);
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000, "YoloMineDetector: cannot convert %s image: %s",
          image->encoding.c_str(), ex.what()
      );
      return false;
    }
    pixels = converted->image.data;
    stride = static_cast<int>(converted->image.step);
    pixel_type = ncnn::Mat::PIXEL_RGB;
  }

  const ncnn::Mat resized = ncnn::Mat::from_pixels_resize(
      pixels, *pixel_type, width, height, stride, letterbox.resized_width, letterbox.resized_height, &input_allocator_
  );
  ncnn::Option border_opt;
  border_opt.blob_allocator = &input_allocator_;
  ncnn::copy_make_border(
      resized, input, letterbox.pad_top, letterbox.pad_bottom, letterbox.pad_left, letterbox.pad_right,
      ncnn::BORDER_CONSTANT, kPadValue, border_opt
  );
  normalize(input);
  return !input.empty();
}

void YoloMineDetector::normalize(ncnn::Mat& input)
{
  // Ultralytics YOLO26 (same as earlier Ultralytics models): RGB scaled to [0, 1], no mean subtraction
  static constexpr float kNorm[3] = {1.f / 255.f, 1.f / 255.f, 1.f / 255.f};
  input.substract_mean_normalize(nullptr, kNorm);
}

std::optional<std::vector<yolo::Detection>> YoloMineDetector::infer(const ncnn::Mat& input)
{
  // One extractor per frame; the executor never runs this callback concurrently, so net_ is never used in parallel.
  ncnn::Extractor ex = net_.create_extractor();
  ncnn::Mat output;
  if (ex.input(params_.input_blob.c_str(), input) != 0 || ex.extract(params_.output_blob.c_str(), output) != 0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "YoloMineDetector: inference failed");
    return std::nullopt;
  }

  // Ultralytics exports give a 2D blob; accept a single-channel 3D blob too. NCNN's h is rows, w is columns.
  const ncnn::Mat out2d = (output.dims == 3 && output.c == 1) ? output.channel(0) : output;
  const auto* data = static_cast<const float*>(out2d.data);
  const auto conf = static_cast<float>(params_.conf_threshold);
  const auto max_detections = static_cast<std::size_t>(params_.max_detections);
  std::optional<int> num_classes;
  if (!params_.class_labels.empty()) {
    num_classes = static_cast<int>(params_.class_labels.size());
  }

  if (output_format_ == yolo::OutputFormat::End2End) {
    if (out2d.dims == 2 && yolo::is_end2end_shape(out2d.h, out2d.w)) {
      // NMS-free head: rows are already final detections
      return yolo::keep_top(yolo::decode_end2end(data, out2d.h, conf, num_classes), max_detections);
    }
  } else {
    const std::optional<int> classes =
        out2d.dims == 2 ? yolo::one_to_many_classes(out2d.h, out2d.w, num_classes) : std::nullopt;
    if (classes) {
      return yolo::nms(
          yolo::decode_one_to_many(data, out2d.h, out2d.w, conf), static_cast<float>(params_.iou_threshold),
          max_detections
      );
    }
  }

  RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "YoloMineDetector: output shape (dims=%d h=%d w=%d c=%d) does not match output_format '%s' with %zu class "
      "labels. end2end expects [max_det, 6]; one_to_many expects [4 + classes, anchors]",
      output.dims, output.h, output.w, output.c, params_.output_format.c_str(), params_.class_labels.size()
  );
  return std::nullopt;
}

cv::Mat YoloMineDetector::annotate(const CameraImage& image, const std::vector<yolo::Detection>& detections) const
{
  cv_bridge::CvImagePtr canvas;
  try {
    // Copy needed: the camera buffer is shared with other subscribers and must not be drawn on.
    canvas = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& ex) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "YoloMineDetector: cannot draw on %s image: %s",
        image->encoding.c_str(), ex.what()
    );
    return {};
  }

  for (const yolo::Detection& d : detections) {
    const cv::Scalar color = class_color(d.class_id);
    const cv::Point top_left(static_cast<int>(d.box.x_min), static_cast<int>(d.box.y_min));
    const cv::Point bottom_right(static_cast<int>(d.box.x_max), static_cast<int>(d.box.y_max));
    cv::rectangle(canvas->image, top_left, bottom_right, color, 2);

    const std::string name = d.class_id >= 0 && static_cast<std::size_t>(d.class_id) < params_.class_labels.size()
                                 ? params_.class_labels[static_cast<std::size_t>(d.class_id)]
                                 : std::to_string(d.class_id);
    char text[128];
    std::snprintf(text, sizeof(text), "%s %.2f", name.c_str(), d.score);
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const cv::Point label_origin(top_left.x, std::max(top_left.y, size.height + baseline));
    cv::rectangle(
        canvas->image, label_origin + cv::Point(0, baseline), label_origin + cv::Point(size.width, -size.height), color,
        cv::FILLED
    );
    cv::putText(canvas->image, text, label_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
  return canvas->image;
}

void YoloMineDetector::record_timing(
    std::chrono::duration<double, std::milli> inference,
    std::chrono::duration<double, std::milli> total
)
{
  ++stats_frames_;
  stats_inference_ms_ += inference.count();
  stats_total_ms_ += total.count();
  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = now - stats_start_;
  if (elapsed < kStatsPeriod) {
    return;
  }
  RCLCPP_INFO(
      node_->get_logger(), "YoloMineDetector: %.1f frames/s processed, inference %.1f ms, total %.1f ms per frame",
      stats_frames_ / elapsed.count(), stats_inference_ms_ / stats_frames_, stats_total_ms_ / stats_frames_
  );
  stats_start_ = now;
  stats_frames_ = 0;
  stats_inference_ms_ = 0.0;
  stats_total_ms_ = 0.0;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::YoloMineDetector, pennair_vision::VisionPlugin)
