// Generated raw Rust structs. No ROS runtime is required.

/// A ROS message that can be decoded from encapsulated CDR bytes.
pub trait RosMessage: Sized + Send + 'static {
    fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error>;
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
