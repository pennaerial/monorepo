#pragma once

/// Pure YOLO pre/post-processing math with no ROS, NCNN or OpenCV dependency, so it can be unit tested anywhere.
/// Targets Ultralytics YOLO26 NCNN exports, which can have either detection head:
///  - End2End (YOLO26's NMS-free one-to-one head): output [max_det, 6], each row x1, y1, x2, y2, score, class_id in
///    model-input pixels, already deduplicated, so no NMS is applied.
///  - OneToMany (YOLO26 one-to-many head, same layout as YOLOv8/YOLO11): output [4 + num_classes, num_anchors];
///    rows 0-3 are box cx, cy, w, h in model-input pixels, the rest are per-class scores (already sigmoided), with no
///    objectness score. Needs NMS.
/// The head is chosen explicitly by parameter rather than guessed from the tensor shape, because the shapes can
/// coincide (e.g. a 2-class one-to-many output has 6 channels, like an end-to-end row).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pennair_vision::yolo
{

/// Box in some pixel space, (x_min, y_min) top-left, (x_max, y_max) bottom-right.
struct Box {
  float x_min{0.f};
  float y_min{0.f};
  float x_max{0.f};
  float y_max{0.f};
};

struct Detection {
  int class_id{0};
  float score{0.f};
  Box box;
};

/// Which detection head the exported model ends with.
enum class OutputFormat {
  End2End,    // [max_det, 6] rows of x1, y1, x2, y2, score, class_id; NMS-free
  OneToMany,  // [4 + num_classes, num_anchors]; needs NMS
};

/// Parses the output_format parameter ("end2end" or "one_to_many").
inline std::optional<OutputFormat> parse_output_format(const std::string& name)
{
  if (name == "end2end") {
    return OutputFormat::End2End;
  }
  if (name == "one_to_many") {
    return OutputFormat::OneToMany;
  }
  return std::nullopt;
}

/// Number of values per row of an end-to-end output.
inline constexpr int kEnd2EndColumns = 6;

/// True if a buffer with `step` bytes per row (row padding allowed) really contains height rows of
/// width * bytes_per_pixel pixel bytes. Guards against reading past the end of a malformed message.
inline bool buffer_holds_image(
    std::size_t data_size,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t step,
    std::uint32_t bytes_per_pixel
)
{
  if (width == 0 || height == 0 || bytes_per_pixel == 0) {
    return false;
  }
  const std::uint64_t row_bytes = static_cast<std::uint64_t>(width) * bytes_per_pixel;
  if (step < row_bytes) {
    return false;
  }
  // the last row does not need trailing padding
  return data_size >= static_cast<std::uint64_t>(step) * (height - 1) + row_bytes;
}

/// True if a frame already has the model's square input size, so it needs no resize or letterbox.
inline bool matches_model_input(int width, int height, int input_size)
{
  return width == input_size && height == input_size;
}

/// How an image was resized and padded into the square model input (Ultralytics-style centered letterbox).
struct Letterbox {
  float scale{1.f};  // model-input pixels per source pixel
  int resized_width{0};
  int resized_height{0};
  int pad_left{0};
  int pad_top{0};
  int pad_right{0};
  int pad_bottom{0};
};

/// Computes a letterbox that fits a src_width x src_height image into input_size x input_size, keeping aspect ratio.
inline Letterbox compute_letterbox(int src_width, int src_height, int input_size)
{
  Letterbox lb;
  lb.scale = std::min(
      static_cast<float>(input_size) / static_cast<float>(src_width),
      static_cast<float>(input_size) / static_cast<float>(src_height)
  );
  lb.resized_width = std::clamp(static_cast<int>(std::lround(src_width * lb.scale)), 1, input_size);
  lb.resized_height = std::clamp(static_cast<int>(std::lround(src_height * lb.scale)), 1, input_size);
  const int pad_w = input_size - lb.resized_width;
  const int pad_h = input_size - lb.resized_height;
  lb.pad_left = pad_w / 2;
  lb.pad_right = pad_w - lb.pad_left;
  lb.pad_top = pad_h / 2;
  lb.pad_bottom = pad_h - lb.pad_top;
  return lb;
}

/// Maps a box from model-input pixels back to source-image pixels, clamped to the image.
inline Box to_source_pixels(const Box& input_box, const Letterbox& lb, int src_width, int src_height)
{
  auto map_x = [&](float x) {
    return std::clamp((x - static_cast<float>(lb.pad_left)) / lb.scale, 0.f, static_cast<float>(src_width));
  };
  auto map_y = [&](float y) {
    return std::clamp((y - static_cast<float>(lb.pad_top)) / lb.scale, 0.f, static_cast<float>(src_height));
  };
  return Box{map_x(input_box.x_min), map_y(input_box.y_min), map_x(input_box.x_max), map_y(input_box.y_max)};
}

inline float iou(const Box& a, const Box& b)
{
  const float ix = std::max(0.f, std::min(a.x_max, b.x_max) - std::max(a.x_min, b.x_min));
  const float iy = std::max(0.f, std::min(a.y_max, b.y_max) - std::max(a.y_min, b.y_min));
  const float inter = ix * iy;
  const float area_a = std::max(0.f, a.x_max - a.x_min) * std::max(0.f, a.y_max - a.y_min);
  const float area_b = std::max(0.f, b.x_max - b.x_min) * std::max(0.f, b.y_max - b.y_min);
  const float uni = area_a + area_b - inter;
  return uni > 0.f ? inter / uni : 0.f;
}

/// True if a rows x cols output (row-major) is an end-to-end [max_det, 6] tensor.
inline bool is_end2end_shape(int rows, int cols)
{
  return rows > 0 && cols == kEnd2EndColumns;
}

/// Number of classes in a one-to-many [4 + num_classes, num_anchors] output, or nullopt if the shape does not fit.
/// When num_classes is known (from class_labels) the channel count must match it exactly.
inline std::optional<int> one_to_many_classes(int rows, int cols, std::optional<int> num_classes)
{
  if (rows <= 4 || cols <= 0) {
    return std::nullopt;
  }
  if (num_classes) {
    return rows == 4 + *num_classes ? num_classes : std::nullopt;
  }
  // without labels, there are always far more anchors than channels
  return rows < cols ? std::optional{rows - 4} : std::nullopt;
}

/// Decodes an end-to-end [rows, 6] output (x1, y1, x2, y2, score, class_id per row) into detections in model-input
/// pixels, keeping rows with score >= conf_threshold. Rows whose class is outside [0, num_classes) are dropped when
/// num_classes is known.
inline std::vector<Detection>
decode_end2end(const float* data, int rows, float conf_threshold, std::optional<int> num_classes)
{
  std::vector<Detection> out;
  for (int r = 0; r < rows; ++r) {
    const float* row = data + static_cast<std::size_t>(r) * kEnd2EndColumns;
    const float score = row[4];
    if (!(score >= conf_threshold)) {  // also rejects NaN
      continue;
    }
    const int class_id = static_cast<int>(std::lround(row[5]));
    if (class_id < 0 || (num_classes && class_id >= *num_classes)) {
      continue;
    }
    out.push_back(Detection{class_id, score, Box{row[0], row[1], row[2], row[3]}});
  }
  return out;
}

/// Decodes a one-to-many [4 + num_classes, num_anchors] output into candidate detections in model-input pixels.
/// Keeps, per anchor, only the best class, and only if its score is >= conf_threshold. Needs NMS afterwards.
inline std::vector<Detection> decode_one_to_many(const float* data, int rows, int cols, float conf_threshold)
{
  const int num_anchors = cols;
  const int num_classes = rows - 4;
  auto at = [&](int c, int a) { return data[static_cast<std::size_t>(c) * cols + a]; };

  std::vector<Detection> out;
  for (int a = 0; a < num_anchors; ++a) {
    int best_class = 0;
    float best_score = at(4, a);
    for (int k = 1; k < num_classes; ++k) {
      const float s = at(4 + k, a);
      if (s > best_score) {
        best_score = s;
        best_class = k;
      }
    }
    if (!(best_score >= conf_threshold)) {
      continue;
    }
    const float cx = at(0, a);
    const float cy = at(1, a);
    const float w = at(2, a);
    const float h = at(3, a);
    out.push_back(Detection{best_class, best_score, Box{cx - w / 2.f, cy - h / 2.f, cx + w / 2.f, cy + h / 2.f}});
  }
  return out;
}

/// Keeps the max_detections highest-scoring detections, highest first. Used for end-to-end output, which needs no NMS.
inline std::vector<Detection> keep_top(std::vector<Detection> dets, std::size_t max_detections)
{
  std::stable_sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) { return a.score > b.score; });
  if (dets.size() > max_detections) {
    dets.resize(max_detections);
  }
  return dets;
}

/// Greedy per-class non-maximum suppression (Ultralytics default: boxes of different classes never suppress each
/// other). Returns at most max_detections boxes, highest score first. Used for one-to-many output only.
inline std::vector<Detection> nms(std::vector<Detection> dets, float iou_threshold, std::size_t max_detections)
{
  std::stable_sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) { return a.score > b.score; });
  std::vector<Detection> kept;
  for (const Detection& d : dets) {
    if (kept.size() >= max_detections) {
      break;
    }
    const bool suppressed = std::any_of(kept.begin(), kept.end(), [&](const Detection& k) {
      return k.class_id == d.class_id && iou(k.box, d.box) > iou_threshold;
    });
    if (!suppressed) {
      kept.push_back(d);
    }
  }
  return kept;
}

}  // namespace pennair_vision::yolo
