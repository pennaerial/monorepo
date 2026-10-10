// Generated raw Rust structs. No ROS runtime is required.

/// A ROS message that can be decoded from encapsulated CDR bytes.
pub trait RosMessage: Sized {
    fn deserialize(bytes: &[u8]) -> Result<Self, cdr::Error>;
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
