from __future__ import annotations

from typing import override

from rclpy.node import Node
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from sim_interfaces.srv import GetSearchLocations
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge

from uav.vehicles.UAV import UAV

import pupil_apriltags

import math
import cv2 as cv


def pixel_to_offset(u, v, altitude, fx=331, fy=331, cx=392, cy=309):
    x_m = (u - cx) / fx * altitude
    y_m = (v - cy) / fy * altitude
    return x_m, y_m

def fwd_right_to_local(forward, right, yaw):
    north = (math.cos(yaw) * forward) - (math.sin(yaw) * right)
    east = (math.sin(yaw) * forward) + (math.cos(yaw) * right)

    return north, east

def detect_stars(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, (20, 150, 150), (35, 255, 255))
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    print("blobs found:", len(contours))

    centroids = []

    for c in contours:
        m = cv.moments(c)

        area = cv.contourArea(c)
        hull = cv.convexHull(c)
        hull_area = cv.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = area / hull_area

        if solidity > 0.8:
            continue

        if m["m00"] != 0:
            cx = int(m["m10"]/m["m00"])
            cy = int(m["m01"]/m["m00"])    
            centroids.append((cx, cy))
    
    return centroids

class SearchParams(ParamsBase):
    """
    altitude: How high to fly while searching (meters)
    """
    altitude: float = 5.0

@register_mode(
    id="uav.SearchMode",
    params_cls=SearchParams,
    targets=[UAV],
    transition_labels=["complete"],    
)

class SearchMode(Mode[UAV, SearchParams]):
    """
    A mode for searching for a target.
    """


    def check_stability(self, time_delta: float) -> bool:
        lp = self.vehicle.local_position
        roll, pitch = self.vehicle.roll, self.vehicle.pitch
        if roll is None or pitch is None:
            return False

        speed = math.sqrt(lp.vx**2 + lp.vy**2 + lp.vz**2)
        tilt = max(abs(roll), abs(pitch))

        if speed < 0.5 and tilt < 0.05:
            self.stable_time += time_delta
        else:
            self.stable_time = 0.0

        if self.stable_time < 0.5:
            return False

        return True


    def image_callback(self, msg: Image) -> None:
        self.latest_frame = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.cam_k is not None:
            return
        self.cam_k = msg.k
        self.cam_fx = self.cam_k[0]
        self.cam_fy = self.cam_k[4]
        self.cx = self.cam_k[2]
        self.cy = self.cam_k[5]

    @override
    def initialize(self, node: Node, vehicle: UAV, params: SearchParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.search_locations = None
        self.future = None
        self.search_locations_goal = None
        self.star_targets = []
        self.index = 0
        self.target_tag_id = None
        self.tag_detector = pupil_apriltags.Detector(families="tag36h11")
        self.target_found = False

        self.cv_bridge = CvBridge()
        self.cam_k = None
        self.cam_fx = None
        self.cam_fy = None
        self.cx = None
        self.cy = None
        self.latest_frame = None
        self.debug_msg = None
        self.dist = None

        self.stable_time = 0.0
        self.state = "patch"
        self.star_index = 0

        self.client = self.node.create_client(GetSearchLocations, "/get_search_patches")
        self.client.wait_for_service(timeout_sec=5.0)

        self.request = GetSearchLocations.Request()
        self.future = self.client.call_async(self.request)

        self.debug_pub = self.node.create_publisher(Image, "/search_debug", 10)
        self.img_sub = self.node.create_subscription(Image, self.vehicle.image_topic, self.image_callback, 10)
        self.cam_info_sub = self.node.create_subscription(CameraInfo, self.vehicle.camera_info_topic, self.camera_info_callback, 10)

    @override
    def on_update(self, time_delta: float) -> None:
        if self.target_found:
            return
        if self.debug_msg is not None:
            self.debug_pub.publish(self.debug_msg)
        if self.search_locations is None: 
            if self.future.done():
                response = self.future.result()
                if response.ready and len(response.search_locations) > 0:
                    self.search_locations = response.search_locations
                    self.target_tag_id = response.target_tag_id
                    self.log(f"Received search locations and target tag ID: {self.search_locations}, {self.target_tag_id}")
                else:
                    self.log("Search locations not ready yet.")
                    self.future = self.client.call_async(self.request)
            return

        if self.state == "stars":
            self.vehicle.publish_position_setpoint(self.star_targets[self.star_index], lock_yaw=False) 
            target = self.star_targets[self.star_index]
            dx = target[0] - self.vehicle.local_position.x
            dy = target[1] - self.vehicle.local_position.y
            horizontal_error = math.sqrt(dx**2 + dy**2)
            vertical_error = abs(target[2] - self.vehicle.local_position.z)
            if horizontal_error < 0.15 and vertical_error < 0.14:
                if not self.check_stability(time_delta):
                    return
                self.log(f"\n\nhorizontal={horizontal_error:.2f}, vertical={vertical_error:.2f}\n\n")
                self.log(f"\n\n*******************Arrived at star {self.star_index} at {self.star_targets[self.star_index]}*******************")
                if self.latest_frame is not None:
                    gray = self.cv_bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='mono8')
                    tags = self.tag_detector.detect(gray)
                    self.log(f"\nDetected {len(tags)} AprilTags at star {self.star_index}., current height: {self.vehicle.local_position.z:.2f}m)\n\n")
                    if not tags:
                        self.debug_msg = self.cv_bridge.cv2_to_imgmsg(gray, encoding="mono8")
                    for tag in tags:
                        if tag.tag_id == self.target_tag_id:
                            self.log(f"Target tag {self.target_tag_id} found at star {self.star_index}!")
                            self.target_found = True
                            return
                self.star_index += 1
                self.stable_time = 0.0

                if self.star_index >= len(self.star_targets):
                    self.state = "patch"
                    self.index += 1
                    self.star_targets = []
                    self.stable_time = 0.0
            return

        if self.index >= len(self.search_locations):
            return

        waypoint = self.search_locations[self.index]
        self.search_locations_goal = (waypoint.latitude_deg, waypoint.longitude_deg, self.p.altitude)
        local = self.vehicle.gps_to_local(self.search_locations_goal)
        self.vehicle.publish_position_setpoint(local, lock_yaw=False)

        dist = self.vehicle.distance_to_waypoint("GPS", self.search_locations_goal)
        if dist < 1.0:
            if not self.check_stability(time_delta):
                return
            self.star_targets = []
            if self.latest_frame is not None:
                cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')

                centroids = detect_stars(cv_image)
                
                debug_frame = cv_image.copy()
                for i, (px, py) in enumerate(centroids):
                    cv.circle(debug_frame, (px, py), 8, (0, 0, 255), 2)
                    cv.putText(debug_frame, str(i), (px + 10, py - 10),
                    cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                self.debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                if centroids:
                    for cx, cy in centroids:
                        x_m, y_m = pixel_to_offset(cx, cy, self.p.altitude, self.cam_fx, self.cam_fy, self.cx, self.cy)

                        right, forward = x_m, -y_m
                        north, east = fwd_right_to_local(forward, right, self.vehicle.yaw)
                        north = north + self.vehicle.local_position.x
                        east = east + self.vehicle.local_position.y

                        self.star_targets.append((north, east, -0.5))

                self.log(f"\n\n--------------------Arrived at waypoint {self.index}, found {len(centroids)} gold stars.--------------------")
           
            self.stable_time = 0.0
            if len(self.star_targets) > 0:
                self.state = "stars"
                self.star_index = 0
            else:
                self.index += 1

    @override
    def check_status(self) -> str | None:
        if self.target_found:
            return "complete"
        if self.search_locations is not None and self.index >= len(self.search_locations):
            return "complete"
        return "continue"
