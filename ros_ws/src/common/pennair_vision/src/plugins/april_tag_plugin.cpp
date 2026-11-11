#include "pennair_vision/plugins/april_tag_plugin.hpp"

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "pennair_vision/vision_plugin.hpp"
#include <sensor_msgs/image_encodings.hpp>

#include "pennair_vision/msg/april_tag.hpp"
#include "pennair_vision/msg/april_tag_array.hpp"

#include <cmath>

using namespace cv;

namespace pennair_vision
{

AprilTagPlugin::AprilTagPlugin()
{
  // TODO: make detector configurable
  tf_ = tag36h11_create();
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);

  td_->quad_decimate = 2.0;
  td_->quad_sigma = 0.0;
  td_->nthreads = 2; 
  td_->refine_edges = 1; 
}

AprilTagPlugin::~AprilTagPlugin()
{
  if (td_)
    apriltag_detector_destroy(td_);
  if (tf_)
    tag36h11_destroy(tf_);
}

ProcessResult<pennair_vision::msg::AprilTagArray>
AprilTagPlugin::process(const CameraImage& image, const CameraInfo& info, bool generate_debug)
{
  ProcessResult<pennair_vision::msg::AprilTagArray> result;

  const auto frame = cv_bridge::toCvShare(image, "bgr8");
  cv::Mat gray;
  cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);

  image_u8_t apriltag_img = {.width = gray.cols, .height = gray.rows, .stride = static_cast<int>(gray.step), .buf = gray.data};

  zarray_t* detections = apriltag_detector_detect(td_, &apriltag_img);
  int num_tags = zarray_size(detections);
  result.message.header = image->header;

  cv::Mat debug_image;
  if (generate_debug) {
    debug_image = frame->image.clone();
  }
  
  for (int i = 0; i < num_tags; i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    pennair_vision::msg::AprilTag tag;
    tag.family = det->family->name;
    tag.id = det->id;
    tag.hamming = det->hamming;
    tag.decision_margin = det->decision_margin;

    tag.centre.x = det->c[0];
    tag.centre.y = det->c[1];
    tag.centre.z = 0.0;

    for (int j = 0; j < 4; j++) {
        tag.corners[j].x = det->p[j][0];
        tag.corners[j].y = det->p[j][1];
        tag.corners[j].z = 0.0;
    }

    for (int j = 0; j < 9; j++) {
      tag.homography[j] = det->H->data[j];
    }

    double tag_size = 0.165; //need to input actual size of apriltag in meters
    apriltag_detection_info_t det_info;
    det_info.det = det;
    det_info.tagsize = tag_size;

    det_info.fx = info->k[0]; 
    det_info.fy = info->k[4];
    det_info.cx = info->k[2];
    det_info.cy = info->k[5];

    apriltag_pose_t pose{};
    estimate_tag_pose(&det_info, &pose);
    tag.pose_valid = (pose.R != nullptr && pose.t != nullptr);
    if (tag.pose_valid) {
      tag.pose.position.x = pose.t->data[0];
      tag.pose.position.y = pose.t->data[1];
      tag.pose.position.z = pose.t->data[2];
      double r00 = pose.R->data[0];
      double r01 = pose.R->data[1];
      double r02 = pose.R->data[2];
      double r10 = pose.R->data[3];
      double r11 = pose.R->data[4];
      double r12 = pose.R->data[5];
      double r20 = pose.R->data[6];
      double r21 = pose.R->data[7];
      double r22 = pose.R->data[8];

      double trace = r00 + r11 + r22;
      double qx, qy, qz, qw;

      if (trace > 0.0) {
          double s = 2.0 * std::sqrt(trace + 1.0);
          qw = 0.25 * s;
          qx = (r21 - r12) / s;
          qy = (r02 - r20) / s;
          qz = (r10 - r01) / s;
      } else if (r00 > r11 && r00 > r22) {
          double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
          qw = (r21 - r12) / s;
          qx = 0.25 * s;
          qy = (r01 + r10) / s;
          qz = (r02 + r20) / s;
      } else if (r11 > r22) {
          double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
          qw = (r02 - r20) / s;
          qx = (r01 + r10) / s;
          qy = 0.25 * s;
          qz = (r12 + r21) / s;
      } else {
          double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
          qw = (r10 - r01) / s;
          qx = (r02 + r20) / s;
          qy = (r12 + r21) / s;
          qz = 0.25 * s;
      }

      double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);

      if (norm > 0.0) {
          qx /= norm;
          qy /= norm;
          qz /= norm;
          qw /= norm;
      }

      tag.pose.orientation.x = qx;
      tag.pose.orientation.y = qy;
      tag.pose.orientation.z = qz;
      tag.pose.orientation.w = qw;
      }

    if (generate_debug) {
      cv::Point corners[4];
      for (int l = 0; l < 4; l++) {
        corners[l] = cv::Point(det->p[l][0], det->p[l][1]);
      }

      for(int j = 0; j < 4; j++) { //adds bounding box around april tag
        cv::line(debug_image, corners[j], corners[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }

      //adds april tag id text on top of frame
      cv::Point center(
      static_cast<int>(det->c[0]),
      static_cast<int>(det->c[1]));
      cv::putText(debug_image, "ID: " + std::to_string(det->id), cv::Point(center.x,center.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 2); 

      if(tag.pose_valid){
        auto project_point = [&](double x, double y, double z) -> cv::Point{
        double X = pose.R->data[0]*x + pose.R->data[1]*y + pose.R->data[2]*z + pose.t->data[0];
        double Y = pose.R->data[3]*x + pose.R->data[4]*y + pose.R->data[5]*z + pose.t->data[1];
        double Z = pose.R->data[6]*x + pose.R->data[7]*y + pose.R->data[8]*z + pose.t->data[2];
        
        if(Z<=0) {
          return cv::Point(-1, -1); // point behind camera
        }

        int px = static_cast<int>((det_info.fx * X / Z) + det_info.cx);
        int py = static_cast<int>((det_info.fy * Y / Z) + det_info.cy);
        return cv::Point(px, py);
        };

        double axis_length = 0.1; 
        cv::Point origin = project_point(0, 0, 0);
        cv::Point x_axis = project_point(axis_length, 0, 0);
        cv::Point y_axis = project_point(0, axis_length, 0);
        cv::Point z_axis = project_point(0, 0, axis_length);

        if (origin.x >= 0 && x_axis.x >= 0) {
          cv::line(debug_image, origin, x_axis, cv::Scalar(0, 0, 255), 3);
        } if (origin.x >= 0 && y_axis.x >= 0) {
          cv::line(debug_image, origin, y_axis, cv::Scalar(0, 255, 0), 3);
        } if (origin.x >= 0 && z_axis.x >= 0) {
          cv::line(debug_image, origin, z_axis, cv::Scalar(255, 0, 0), 3);
        }
      }
    }
    
    if (pose.R) {
      matd_destroy(pose.R);
    }
    if (pose.t) {
      matd_destroy(pose.t);
    }
    
    result.message.detections.push_back(tag);
  } 
  
  if (generate_debug) {
    result.debug_image = debug_image;
    result.debug_encoding = sensor_msgs::image_encodings::BGR8;
  }

  apriltag_detections_destroy(detections);

  return result;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pennair_vision::AprilTagPlugin, pennair_vision::VisionPlugin)