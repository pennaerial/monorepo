#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pennair_vision/yolo_postprocess.hpp"

using namespace pennair_vision::yolo;

namespace
{

/// Builds a channels-first [4 + num_classes][num_anchors] output tensor from per-anchor rows.
struct AnchorRow {
  float cx, cy, w, h;
  std::vector<float> scores;
};

std::vector<float> channels_first(const std::vector<AnchorRow>& anchors, int num_classes)
{
  const int n = static_cast<int>(anchors.size());
  std::vector<float> data(static_cast<std::size_t>((4 + num_classes) * n), 0.f);
  for (int a = 0; a < n; ++a) {
    const AnchorRow& r = anchors[static_cast<std::size_t>(a)];
    data[static_cast<std::size_t>(0 * n + a)] = r.cx;
    data[static_cast<std::size_t>(1 * n + a)] = r.cy;
    data[static_cast<std::size_t>(2 * n + a)] = r.w;
    data[static_cast<std::size_t>(3 * n + a)] = r.h;
    for (int k = 0; k < num_classes; ++k) {
      data[static_cast<std::size_t>((4 + k) * n + a)] = r.scores[static_cast<std::size_t>(k)];
    }
  }
  return data;
}

}  // namespace

TEST(Buffer, AcceptsTightlyPackedRgb)
{
  EXPECT_TRUE(buffer_holds_image(640u * 3u * 640u, 640, 640, 640 * 3, 3));
}

TEST(Buffer, AcceptsPaddedRowsWithAndWithoutTrailingPadding)
{
  // stride padded from 1920 to 2048 bytes, as libcamera may do
  EXPECT_TRUE(buffer_holds_image(2048u * 640u, 640, 640, 2048, 3));
  // last row only needs its pixel bytes, not the padding after them
  EXPECT_TRUE(buffer_holds_image(2048u * 639u + 1920u, 640, 640, 2048, 3));
}

TEST(Buffer, RejectsMalformedMessages)
{
  EXPECT_FALSE(buffer_holds_image(2048u * 639u + 1919u, 640, 640, 2048, 3));  // truncated by one byte
  EXPECT_FALSE(buffer_holds_image(1919u * 640u, 640, 640, 1919, 3));          // step shorter than a row
  EXPECT_FALSE(buffer_holds_image(1000u, 0, 640, 1920, 3));                   // zero width
  EXPECT_FALSE(buffer_holds_image(1000u, 640, 0, 1920, 3));                   // zero height
  EXPECT_FALSE(buffer_holds_image(1000u, 640, 640, 1920, 0));                 // unknown pixel size
}

TEST(Buffer, NoOverflowOnHugeDimensions)
{
  EXPECT_FALSE(buffer_holds_image(1024u, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 4));
}

TEST(FastPath, OnlyExactModelSize)
{
  EXPECT_TRUE(matches_model_input(640, 640, 640));
  EXPECT_FALSE(matches_model_input(640, 480, 640));
  EXPECT_FALSE(matches_model_input(1280, 1280, 640));
  EXPECT_FALSE(matches_model_input(320, 320, 640));
}

TEST(FastPath, ModelSizedFrameHasIdentityLetterbox)
{
  const Letterbox lb = compute_letterbox(640, 640, 640);
  EXPECT_FLOAT_EQ(lb.scale, 1.f);
  EXPECT_EQ(lb.resized_width, 640);
  EXPECT_EQ(lb.resized_height, 640);
  EXPECT_EQ(lb.pad_left + lb.pad_right + lb.pad_top + lb.pad_bottom, 0);
  const Box src = to_source_pixels(Box{12.5f, 30.f, 200.f, 639.f}, lb, 640, 640);
  EXPECT_FLOAT_EQ(src.x_min, 12.5f);
  EXPECT_FLOAT_EQ(src.y_min, 30.f);
  EXPECT_FLOAT_EQ(src.x_max, 200.f);
  EXPECT_FLOAT_EQ(src.y_max, 639.f);
}

TEST(Letterbox, LandscapeImagePadsTopAndBottom)
{
  // 1280x720 into 640: scale 0.5 -> 640x360, 280 rows of padding split evenly
  const Letterbox lb = compute_letterbox(1280, 720, 640);
  EXPECT_FLOAT_EQ(lb.scale, 0.5f);
  EXPECT_EQ(lb.resized_width, 640);
  EXPECT_EQ(lb.resized_height, 360);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_right, 0);
  EXPECT_EQ(lb.pad_top, 140);
  EXPECT_EQ(lb.pad_bottom, 140);
}

TEST(Letterbox, OddPaddingPutsRemainderBottomRight)
{
  // 640x481 into 320: scale 320/640 = 0.5 -> 320x241 (240.5 rounds up), 79 rows of padding
  const Letterbox lb = compute_letterbox(640, 481, 320);
  EXPECT_EQ(lb.resized_width, 320);
  EXPECT_EQ(lb.resized_height, 241);
  EXPECT_EQ(lb.pad_top + lb.pad_bottom, 79);
  EXPECT_EQ(lb.pad_top, 39);
  EXPECT_EQ(lb.pad_bottom, 40);
}

TEST(Letterbox, PortraitImagePadsLeftAndRight)
{
  const Letterbox lb = compute_letterbox(480, 640, 640);
  EXPECT_EQ(lb.resized_width, 480);
  EXPECT_EQ(lb.resized_height, 640);
  EXPECT_EQ(lb.pad_left, 80);
  EXPECT_EQ(lb.pad_right, 80);
  EXPECT_EQ(lb.pad_top, 0);
}

TEST(Letterbox, MapsInputBoxBackToSourcePixels)
{
  const Letterbox lb = compute_letterbox(1280, 720, 640);
  // source box (100, 200)-(300, 400) appears in the input at scale 0.5 shifted down by pad_top
  const Box input_box{50.f, 100.f + 140.f, 150.f, 200.f + 140.f};
  const Box src = to_source_pixels(input_box, lb, 1280, 720);
  EXPECT_NEAR(src.x_min, 100.f, 1e-4);
  EXPECT_NEAR(src.y_min, 200.f, 1e-4);
  EXPECT_NEAR(src.x_max, 300.f, 1e-4);
  EXPECT_NEAR(src.y_max, 400.f, 1e-4);
}

TEST(Letterbox, ClampsBoxesInPaddingToImage)
{
  const Letterbox lb = compute_letterbox(1280, 720, 640);
  const Box src = to_source_pixels(Box{-10.f, 0.f, 700.f, 640.f}, lb, 1280, 720);
  EXPECT_FLOAT_EQ(src.x_min, 0.f);
  EXPECT_FLOAT_EQ(src.y_min, 0.f);
  EXPECT_FLOAT_EQ(src.x_max, 1280.f);
  EXPECT_FLOAT_EQ(src.y_max, 720.f);
}

TEST(Format, ParsesParameterValues)
{
  EXPECT_EQ(parse_output_format("end2end"), OutputFormat::End2End);
  EXPECT_EQ(parse_output_format("one_to_many"), OutputFormat::OneToMany);
  EXPECT_FALSE(parse_output_format("yolov8").has_value());
}

TEST(Shape, End2EndNeedsSixColumns)
{
  EXPECT_TRUE(is_end2end_shape(300, 6));
  EXPECT_FALSE(is_end2end_shape(6, 8400));  // a 2-class one-to-many output is not mistaken for end-to-end
  EXPECT_FALSE(is_end2end_shape(0, 6));
}

TEST(Shape, OneToManyClassCount)
{
  EXPECT_EQ(one_to_many_classes(6, 8400, 2), 2);
  EXPECT_EQ(one_to_many_classes(84, 8400, std::nullopt), 80);
  EXPECT_FALSE(one_to_many_classes(84, 8400, 2).has_value());           // model has 80 classes, labels say 2
  EXPECT_FALSE(one_to_many_classes(300, 6, 2).has_value());             // end-to-end output with the wrong format
  EXPECT_FALSE(one_to_many_classes(300, 6, std::nullopt).has_value());  // same, without labels
  EXPECT_FALSE(one_to_many_classes(4, 8400, std::nullopt).has_value());
}

TEST(End2End, DecodesRowsAboveThreshold)
{
  // rows of x1, y1, x2, y2, score, class_id; the tail is padding the model fills with low scores
  const std::vector<float> data{
      10.f,  20.f,  110.f, 220.f, 0.92f, 1.f,  //
      300.f, 300.f, 350.f, 380.f, 0.40f, 0.f,  //
      0.f,   0.f,   0.f,   0.f,   0.01f, 0.f,
  };
  const std::vector<Detection> dets = decode_end2end(data.data(), 3, 0.25f, 2);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].class_id, 1);
  EXPECT_FLOAT_EQ(dets[0].score, 0.92f);
  EXPECT_FLOAT_EQ(dets[0].box.x_min, 10.f);
  EXPECT_FLOAT_EQ(dets[0].box.y_min, 20.f);
  EXPECT_FLOAT_EQ(dets[0].box.x_max, 110.f);
  EXPECT_FLOAT_EQ(dets[0].box.y_max, 220.f);
  EXPECT_EQ(dets[1].class_id, 0);
}

TEST(End2End, DropsUnknownClassesAndNan)
{
  const std::vector<float> data{
      0.f, 0.f, 10.f, 10.f, 0.9f,          5.f,      // class 5 with only 2 labels
      0.f, 0.f, 10.f, 10.f, 0.9f,          -1.f,     // negative class
      0.f, 0.f, 10.f, 10.f, std::nanf(""), 0.f,      // NaN score
      0.f, 0.f, 10.f, 10.f, 0.9f,          0.9999f,  // float class rounds to 1
  };
  const std::vector<Detection> dets = decode_end2end(data.data(), 4, 0.25f, 2);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].class_id, 1);
  // without labels, any non-negative class is kept
  EXPECT_EQ(decode_end2end(data.data(), 4, 0.25f, std::nullopt).size(), 2u);
}

TEST(End2End, KeepTopSortsAndCaps)
{
  const std::vector<Detection> dets{
      {0, 0.5f, {}},
      {1, 0.9f, {}},
      {0, 0.7f, {}},
  };
  const std::vector<Detection> kept = keep_top(dets, 2);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_FLOAT_EQ(kept[0].score, 0.9f);
  EXPECT_FLOAT_EQ(kept[1].score, 0.7f);
}

TEST(OneToMany, KeepsBestClassAboveThreshold)
{
  const std::vector<AnchorRow> anchors{
      {100.f, 100.f, 20.f, 40.f, {0.1f, 0.9f}},  // class 1
      {300.f, 50.f, 10.f, 10.f, {0.2f, 0.1f}},   // below threshold
      {200.f, 200.f, 50.f, 50.f, {0.6f, 0.3f}},  // class 0
  };
  const std::vector<float> data = channels_first(anchors, 2);
  const std::vector<Detection> dets = decode_one_to_many(data.data(), 6, 3, 0.25f);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].class_id, 1);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9f);
  EXPECT_FLOAT_EQ(dets[0].box.x_min, 90.f);
  EXPECT_FLOAT_EQ(dets[0].box.y_min, 80.f);
  EXPECT_FLOAT_EQ(dets[0].box.x_max, 110.f);
  EXPECT_FLOAT_EQ(dets[0].box.y_max, 120.f);
  EXPECT_EQ(dets[1].class_id, 0);
  EXPECT_FLOAT_EQ(dets[1].score, 0.6f);
}

TEST(OneToMany, NoDetectionsGivesEmptyResult)
{
  const std::vector<float> data = channels_first({{10.f, 10.f, 5.f, 5.f, {0.01f, 0.02f}}}, 2);
  EXPECT_TRUE(decode_one_to_many(data.data(), 6, 1, 0.25f).empty());
}

TEST(Nms, SuppressesOverlappingBoxesOfSameClassOnly)
{
  const std::vector<Detection> dets{
      {0, 0.8f, {0.f, 0.f, 100.f, 100.f}},
      {0, 0.9f, {5.f, 5.f, 105.f, 105.f}},      // overlaps the first, higher score -> kept
      {1, 0.7f, {0.f, 0.f, 100.f, 100.f}},      // same place, different class -> kept
      {0, 0.5f, {300.f, 300.f, 350.f, 350.f}},  // separate -> kept
  };
  const std::vector<Detection> kept = nms(dets, 0.45f, 100);
  ASSERT_EQ(kept.size(), 3u);
  EXPECT_FLOAT_EQ(kept[0].score, 0.9f);
  EXPECT_EQ(kept[1].class_id, 1);
  EXPECT_FLOAT_EQ(kept[2].score, 0.5f);
}

TEST(Nms, RespectsMaxDetections)
{
  std::vector<Detection> dets;
  for (int i = 0; i < 10; ++i) {
    const float x = 100.f * static_cast<float>(i);
    dets.push_back({0, 0.1f * static_cast<float>(i), {x, 0.f, x + 50.f, 50.f}});
  }
  const std::vector<Detection> kept = nms(dets, 0.45f, 3);
  ASSERT_EQ(kept.size(), 3u);
  EXPECT_FLOAT_EQ(kept[0].score, 0.9f);
  EXPECT_FLOAT_EQ(kept[2].score, 0.7f);
}

TEST(Iou, KnownValues)
{
  EXPECT_FLOAT_EQ(iou({0.f, 0.f, 10.f, 10.f}, {0.f, 0.f, 10.f, 10.f}), 1.f);
  EXPECT_FLOAT_EQ(iou({0.f, 0.f, 10.f, 10.f}, {20.f, 20.f, 30.f, 30.f}), 0.f);
  EXPECT_NEAR(iou({0.f, 0.f, 10.f, 10.f}, {5.f, 0.f, 15.f, 10.f}), 50.f / 150.f, 1e-6);
}
