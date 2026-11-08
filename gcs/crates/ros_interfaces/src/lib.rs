// Generated raw Rust structs. No ROS runtime is required.
// Regenerate with gcs/generate_rs.py; do not edit by hand.

// Service structs follow ROS naming (e.g. AddTwoInts_Request).
#![allow(non_camel_case_types)]

/// A ROS message that can be decoded from encapsulated CDR bytes.
pub trait RosMessage: Sized + Send + 'static {
    fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error>;
}

/// A ROS service type, tying its request and response messages together.
pub trait RosService {
    type Request: RosMessage;
    type Response: RosMessage;
    /// Full ROS type name, e.g. `example_interfaces/srv/AddTwoInts`.
    const TYPE_NAME: &'static str;
}

pub mod builtin_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Time {
            pub sec: i32,
            pub nanosec: u32,
        }

        impl crate::RosMessage for Time {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod geometry_msgs {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Point {
            pub x: f64,
            pub y: f64,
            pub z: f64,
        }

        impl crate::RosMessage for Point {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Pose {
            pub position: crate::geometry_msgs::msg::Point,
            pub orientation: crate::geometry_msgs::msg::Quaternion,
        }

        impl crate::RosMessage for Pose {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PoseStamped {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::Pose,
        }

        impl crate::RosMessage for PoseStamped {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Quaternion {
            pub x: f64,
            pub y: f64,
            pub z: f64,
            pub w: f64,
        }

        impl crate::RosMessage for Quaternion {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod nav_msgs {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Path {
            pub header: crate::std_msgs::msg::Header,
            pub poses: Vec<crate::geometry_msgs::msg::PoseStamped>,
        }

        impl crate::RosMessage for Path {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod payload_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct DriveCommand {
            pub linear: f64,
            pub angular: f64,
        }

        impl crate::RosMessage for DriveCommand {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct MotorState {
            pub linear_setpoint_mps: f64,
            pub angular_setpoint_rad_s: f64,
            pub left_setpoint_rpm: f64,
            pub right_setpoint_rpm: f64,
            pub left_measured_rpm: f64,
            pub right_measured_rpm: f64,
            pub left_pid_error_rpm: f64,
            pub right_pid_error_rpm: f64,
            pub left_pid_p: f64,
            pub left_pid_i: f64,
            pub left_pid_d: f64,
            pub right_pid_p: f64,
            pub right_pid_i: f64,
            pub right_pid_d: f64,
            pub left_output_norm: f64,
            pub right_output_norm: f64,
            pub left_pwm_percent: f64,
            pub right_pwm_percent: f64,
            pub left_encoder_count: i64,
            pub right_encoder_count: i64,
        }

        impl crate::RosMessage for MotorState {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadHeartbeat {
            pub stamp: crate::builtin_interfaces::msg::Time,
            pub vehicle_name: std::string::String,
            pub seq: u32,
            pub active_mode: std::string::String,
            pub mission_started: bool,
            pub is_error: bool,
            pub connected_peers: Vec<std::string::String>,
            pub disconnected_peers: Vec<std::string::String>,
            pub tag_visible: bool,
            pub tag_distance_m: f32,
            pub tag_bearing_deg: f32,
            pub drive_linear_mps: f32,
            pub drive_angular_radps: f32,
        }

        impl crate::RosMessage for PayloadHeartbeat {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ServoCommand {
            pub degree: f64,
        }

        impl crate::RosMessage for ServoCommand {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

    pub mod srv {
        /// Marker type for the `payload_interfaces/srv/ComputePidZieglerNichols` service.
        pub struct ComputePidZieglerNichols;

        impl crate::RosService for ComputePidZieglerNichols {
            type Request = ComputePidZieglerNichols_Request;
            type Response = ComputePidZieglerNichols_Response;
            const TYPE_NAME: &'static str = "payload_interfaces/srv/ComputePidZieglerNichols";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ComputePidZieglerNichols_Request {
            pub ku: f64,
            pub pu: f64,
        }

        impl crate::RosMessage for ComputePidZieglerNichols_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ComputePidZieglerNichols_Response {
            pub success: bool,
            pub message: std::string::String,
            pub kp: f64,
            pub ki: f64,
            pub kd: f64,
        }

        impl crate::RosMessage for ComputePidZieglerNichols_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `payload_interfaces/srv/DeadReckon` service.
        pub struct DeadReckon;

        impl crate::RosService for DeadReckon {
            type Request = DeadReckon_Request;
            type Response = DeadReckon_Response;
            const TYPE_NAME: &'static str = "payload_interfaces/srv/DeadReckon";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct DeadReckon_Request {
            pub linear: f64,
            pub angular: f64,
            pub speed: f64,
            pub timeout_sec: f64,
        }

        impl crate::RosMessage for DeadReckon_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct DeadReckon_Response {
            pub success: bool,
            pub message: std::string::String,
        }

        impl crate::RosMessage for DeadReckon_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `payload_interfaces/srv/TimedDrive` service.
        pub struct TimedDrive;

        impl crate::RosService for TimedDrive {
            type Request = TimedDrive_Request;
            type Response = TimedDrive_Response;
            const TYPE_NAME: &'static str = "payload_interfaces/srv/TimedDrive";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct TimedDrive_Request {
            pub linear: f64,
            pub angular: f64,
            pub duration_sec: f64,
        }

        impl crate::RosMessage for TimedDrive_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct TimedDrive_Response {
            pub success: bool,
            pub message: std::string::String,
        }

        impl crate::RosMessage for TimedDrive_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod pennair_planner {

    pub mod srv {
        /// Marker type for the `pennair_planner/srv/PlanPath` service.
        pub struct PlanPath;

        impl crate::RosService for PlanPath {
            type Request = PlanPath_Request;
            type Response = PlanPath_Response;
            const TYPE_NAME: &'static str = "pennair_planner/srv/PlanPath";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PlanPath_Request {
            pub start_pose: crate::geometry_msgs::msg::Pose,
            pub goal_pose: crate::geometry_msgs::msg::Pose,
        }

        impl crate::RosMessage for PlanPath_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PlanPath_Response {
            pub path: crate::nav_msgs::msg::Path,
        }

        impl crate::RosMessage for PlanPath_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod rcl_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Log {
            pub stamp: crate::builtin_interfaces::msg::Time,
            pub level: u8,
            pub name: std::string::String,
            pub msg: std::string::String,
            pub file: std::string::String,
            pub function: std::string::String,
            pub line: u32,
        }

        impl Log {
            pub const DEBUG: u8 = 10;
            pub const INFO: u8 = 20;
            pub const WARN: u8 = 30;
            pub const ERROR: u8 = 40;
            pub const FATAL: u8 = 50;
        }

        impl crate::RosMessage for Log {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod ros_gz_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct EntityFactory {
            pub name: std::string::String,
            pub allow_renaming: bool,
            pub sdf: std::string::String,
            pub sdf_filename: std::string::String,
            pub clone_name: std::string::String,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub relative_to: std::string::String,
        }

        impl crate::RosMessage for EntityFactory {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod sensor_msgs {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CameraInfo {
            pub header: crate::std_msgs::msg::Header,
            pub height: u32,
            pub width: u32,
            pub distortion_model: std::string::String,
            pub d: Vec<f64>,
            pub k: [f64; 9],
            pub r: [f64; 9],
            pub p: [f64; 12],
            pub binning_x: u32,
            pub binning_y: u32,
            pub roi: crate::sensor_msgs::msg::RegionOfInterest,
        }

        impl crate::RosMessage for CameraInfo {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CompressedImage {
            pub header: crate::std_msgs::msg::Header,
            pub format: std::string::String,
            pub data: Vec<u8>,
        }

        impl crate::RosMessage for CompressedImage {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Image {
            pub header: crate::std_msgs::msg::Header,
            pub height: u32,
            pub width: u32,
            pub encoding: std::string::String,
            pub is_bigendian: u8,
            pub step: u32,
            pub data: Vec<u8>,
        }

        impl crate::RosMessage for Image {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct RegionOfInterest {
            pub x_offset: u32,
            pub y_offset: u32,
            pub height: u32,
            pub width: u32,
            pub do_rectify: bool,
        }

        impl crate::RosMessage for RegionOfInterest {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod sim_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct HoopPose {
            pub x: f64,
            pub y: f64,
            pub z: f64,
            pub roll: f64,
            pub pitch: f64,
            pub yaw: f64,
        }

        impl crate::RosMessage for HoopPose {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ObjectState {
            pub name: std::string::String,
            pub shape: std::string::String,
            pub model: std::string::String,
            pub color: std::string::String,
            pub scale: f64,
            pub tag_id: i32,
            pub x: f64,
            pub y: f64,
            pub yaw: f64,
        }

        impl crate::RosMessage for ObjectState {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct SearchLocation {
            pub latitude_deg: f64,
            pub longitude_deg: f64,
            pub radius_m: f64,
        }

        impl crate::RosMessage for SearchLocation {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

    pub mod srv {
        /// Marker type for the `sim_interfaces/srv/AttachDetach` service.
        pub struct AttachDetach;

        impl crate::RosService for AttachDetach {
            type Request = AttachDetach_Request;
            type Response = AttachDetach_Response;
            const TYPE_NAME: &'static str = "sim_interfaces/srv/AttachDetach";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct AttachDetach_Request {
            pub child_model_name: std::string::String,
            pub child_link_name: std::string::String,
            pub command: std::string::String,
        }

        impl crate::RosMessage for AttachDetach_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct AttachDetach_Response {
            pub success: bool,
            pub message: std::string::String,
        }

        impl crate::RosMessage for AttachDetach_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `sim_interfaces/srv/CreateMultiple` service.
        pub struct CreateMultiple;

        impl crate::RosService for CreateMultiple {
            type Request = CreateMultiple_Request;
            type Response = CreateMultiple_Response;
            const TYPE_NAME: &'static str = "sim_interfaces/srv/CreateMultiple";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CreateMultiple_Request {
            pub entities: Vec<crate::ros_gz_interfaces::msg::EntityFactory>,
        }

        impl crate::RosMessage for CreateMultiple_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CreateMultiple_Response {
            pub success: bool,
        }

        impl crate::RosMessage for CreateMultiple_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `sim_interfaces/srv/GetSearchLocations` service.
        pub struct GetSearchLocations;

        impl crate::RosService for GetSearchLocations {
            type Request = GetSearchLocations_Request;
            type Response = GetSearchLocations_Response;
            const TYPE_NAME: &'static str = "sim_interfaces/srv/GetSearchLocations";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct GetSearchLocations_Request {
            pub structure_needs_at_least_one_member: u8,
        }

        impl crate::RosMessage for GetSearchLocations_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct GetSearchLocations_Response {
            pub ready: bool,
            pub target_tag_id: i32,
            pub search_locations: Vec<crate::sim_interfaces::msg::SearchLocation>,
        }

        impl crate::RosMessage for GetSearchLocations_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `sim_interfaces/srv/HoopList` service.
        pub struct HoopList;

        impl crate::RosService for HoopList {
            type Request = HoopList_Request;
            type Response = HoopList_Response;
            const TYPE_NAME: &'static str = "sim_interfaces/srv/HoopList";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct HoopList_Request {
            pub structure_needs_at_least_one_member: u8,
        }

        impl crate::RosMessage for HoopList_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct HoopList_Response {
            pub hoop_positions: Vec<crate::sim_interfaces::msg::HoopPose>,
        }

        impl crate::RosMessage for HoopList_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `sim_interfaces/srv/ObjectStateList` service.
        pub struct ObjectStateList;

        impl crate::RosService for ObjectStateList {
            type Request = ObjectStateList_Request;
            type Response = ObjectStateList_Response;
            const TYPE_NAME: &'static str = "sim_interfaces/srv/ObjectStateList";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ObjectStateList_Request {
            pub structure_needs_at_least_one_member: u8,
        }

        impl crate::RosMessage for ObjectStateList_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct ObjectStateList_Response {
            pub world: std::string::String,
            pub stage: std::string::String,
            pub generated_at: std::string::String,
            pub tag_family: std::string::String,
            pub tag_size_m: f64,
            pub objects: Vec<crate::sim_interfaces::msg::ObjectState>,
        }

        impl crate::RosMessage for ObjectStateList_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod std_msgs {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Float64 {
            pub data: f64,
        }

        impl crate::RosMessage for Float64 {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Header {
            pub stamp: crate::builtin_interfaces::msg::Time,
            pub frame_id: std::string::String,
        }

        impl crate::RosMessage for Header {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct Int32 {
            pub data: i32,
        }

        impl crate::RosMessage for Int32 {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct String {
            pub data: std::string::String,
        }

        impl crate::RosMessage for String {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}

pub mod uav_interfaces {

    pub mod msg {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct HelloWorld {
            pub msg: std::string::String,
        }

        impl crate::RosMessage for HelloWorld {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct UdpTest {
            pub data: std::string::String,
        }

        impl crate::RosMessage for UdpTest {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

    pub mod srv {
        /// Marker type for the `uav_interfaces/srv/CameraData` service.
        pub struct CameraData;

        impl crate::RosService for CameraData {
            type Request = CameraData_Request;
            type Response = CameraData_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/CameraData";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CameraData_Request {
            pub cam_info: bool,
            pub cam_image_raw: bool,
            pub cam_image_compressed: bool,
        }

        impl crate::RosMessage for CameraData_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct CameraData_Response {
            pub image_raw: crate::sensor_msgs::msg::Image,
            pub image_compressed: crate::sensor_msgs::msg::CompressedImage,
            pub camera_info: crate::sensor_msgs::msg::CameraInfo,
        }

        impl crate::RosMessage for CameraData_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `uav_interfaces/srv/PayloadAprilTagState` service.
        pub struct PayloadAprilTagState;

        impl crate::RosService for PayloadAprilTagState {
            type Request = PayloadAprilTagState_Request;
            type Response = PayloadAprilTagState_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/PayloadAprilTagState";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadAprilTagState_Request {
            pub tag_size_m: f64,
            pub tag_family: std::string::String,
        }

        impl crate::RosMessage for PayloadAprilTagState_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadAprilTagState_Response {
            pub detector_available: bool,
            pub has_image: bool,
            pub has_camera_info: bool,
            pub tag_ids: Vec<i32>,
            pub pose_x: Vec<f64>,
            pub pose_y: Vec<f64>,
            pub pose_yaw: Vec<f64>,
            pub tvec_x: Vec<f64>,
            pub tvec_y: Vec<f64>,
            pub tvec_z: Vec<f64>,
            pub center_x: Vec<f64>,
            pub center_y: Vec<f64>,
            pub yaw_error: Vec<f64>,
            pub all_tag_ids: Vec<i32>,
            pub all_tvec_x: Vec<f64>,
            pub all_tvec_y: Vec<f64>,
            pub all_tvec_z: Vec<f64>,
            pub all_center_x: Vec<f64>,
            pub all_center_y: Vec<f64>,
            pub all_yaw_error: Vec<f64>,
            pub all_tag_area: Vec<f64>,
            pub image_width: u32,
        }

        impl crate::RosMessage for PayloadAprilTagState_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `uav_interfaces/srv/PayloadColorOrbitState` service.
        pub struct PayloadColorOrbitState;

        impl crate::RosService for PayloadColorOrbitState {
            type Request = PayloadColorOrbitState_Request;
            type Response = PayloadColorOrbitState_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/PayloadColorOrbitState";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadColorOrbitState_Request {
            pub tag_size_m: f64,
            pub tag_family: std::string::String,
            pub dock_target_tag_id: i32,
        }

        impl crate::RosMessage for PayloadColorOrbitState_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadColorOrbitState_Response {
            pub detector_available: bool,
            pub has_image: bool,
            pub has_camera_info: bool,
            pub tag_ids: Vec<i32>,
            pub pose_x: Vec<f64>,
            pub pose_y: Vec<f64>,
            pub pose_yaw: Vec<f64>,
            pub tvec_x: Vec<f64>,
            pub tvec_y: Vec<f64>,
            pub tvec_z: Vec<f64>,
            pub pink_ratio: f64,
            pub pink_count: i32,
            pub green_count: i32,
            pub boundary_detected: bool,
            pub lateral_error_px: f64,
            pub boundary_angle: f64,
            pub dock_target_visible: bool,
            pub dock_target_view_angle_deg: f64,
        }

        impl crate::RosMessage for PayloadColorOrbitState_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `uav_interfaces/srv/PayloadColorSquareState` service.
        pub struct PayloadColorSquareState;

        impl crate::RosService for PayloadColorSquareState {
            type Request = PayloadColorSquareState_Request;
            type Response = PayloadColorSquareState_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/PayloadColorSquareState";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadColorSquareState_Request {
            pub structure_needs_at_least_one_member: u8,
        }

        impl crate::RosMessage for PayloadColorSquareState_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadColorSquareState_Response {
            pub has_image: bool,
            pub current_color: std::string::String,
            pub boundary_detected: bool,
            pub lateral_error_px: f64,
            pub boundary_angle: f64,
        }

        impl crate::RosMessage for PayloadColorSquareState_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `uav_interfaces/srv/PayloadDriveOutState` service.
        pub struct PayloadDriveOutState;

        impl crate::RosService for PayloadDriveOutState {
            type Request = PayloadDriveOutState_Request;
            type Response = PayloadDriveOutState_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/PayloadDriveOutState";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadDriveOutState_Request {
            pub lower_hsv: [i32; 3],
            pub upper_hsv: [i32; 3],
        }

        impl crate::RosMessage for PayloadDriveOutState_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadDriveOutState_Response {
            pub has_image: bool,
            pub can_drive_out: bool,
            pub clear_ratio: f64,
        }

        impl crate::RosMessage for PayloadDriveOutState_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        /// Marker type for the `uav_interfaces/srv/PayloadTracking` service.
        pub struct PayloadTracking;

        impl crate::RosService for PayloadTracking {
            type Request = PayloadTracking_Request;
            type Response = PayloadTracking_Response;
            const TYPE_NAME: &'static str = "uav_interfaces/srv/PayloadTracking";
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadTracking_Request {
            pub altitude: f64,
            pub yaw: f64,
            pub payload_color: std::string::String,
        }

        impl crate::RosMessage for PayloadTracking_Request {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        pub struct PayloadTracking_Response {
            pub x: f64,
            pub y: f64,
            pub direction: [f64; 3],
            pub dlz_empty: bool,
        }

        impl crate::RosMessage for PayloadTracking_Response {
            fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error> {
                cdr::deserialize(bytes)
            }
        }

    }

}
