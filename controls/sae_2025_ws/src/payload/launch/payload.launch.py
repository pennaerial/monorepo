from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_param_file(path):
    with open(path, "r", encoding="utf-8") as param_file:
        return yaml.safe_load(param_file) or {}


def launch_setup(context):
    payload_share_dir = get_package_share_directory("payload")
    payload_params_path = os.path.join(
        payload_share_dir, "config", "payload_params.yaml"
    )

    payload_name = LaunchConfiguration("payload_name").perform(context)
    use_camera_bool = (
        LaunchConfiguration("use_camera").perform(context).lower() == "true"
    )
    controller_override = LaunchConfiguration("controller").perform(context)

    payload_params = load_param_file(payload_params_path)
    ros_params = payload_params.get("/**", {}).get("ros__parameters", {})
    controller = (
        controller_override
        if controller_override
        else ros_params.get("controller", "SimController")
    )
    world_name = ros_params.get("sim", {}).get("world_name", "default")

    parameters = [payload_params_path]
    if controller_override:
        parameters.append({"controller": controller_override})

    payload = Node(
        package="payload",
        executable="payload",
        parameters=parameters,
        output="screen",
        name=payload_name,
    )

    actions = [payload]

    if controller == "SimController":
        gz_camera_topic = (
            f"/world/{world_name}/model/{payload_name}"
            "/link/camera_link/sensor/camera/image"
        )
        gz_camera_info_topic = (
            f"/world/{world_name}/model/{payload_name}"
            "/link/camera_link/sensor/camera/camera_info"
        )
        ros_camera_topic = f"/{payload_name}/camera"
        ros_camera_info_topic = f"/{payload_name}/camera_info"

        camera_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                gz_camera_topic + "@sensor_msgs/msg/Image[gz.msgs.Image",
                gz_camera_info_topic + "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                (gz_camera_topic, ros_camera_topic),
                (gz_camera_info_topic, ros_camera_info_topic),
            ],
            output="screen",
        )
        actions.append(camera_bridge)

    if use_camera_bool and controller != "SimController":
        camera_info_path = os.path.join(
            payload_share_dir, "config", f"{payload_name}_camera_info.yaml"
        )
        v4l2_cam = Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            parameters=[
                {
                    "image_size": [640, 480],
                    "camera_info_url": f"file://{camera_info_path}",
                }
            ],
            remappings=[
                ("/image_raw", f"/{payload_name}/image_raw"),
                ("/image_raw/compressed", f"/{payload_name}/image_raw/compressed"),
                ("/camera_info", f"/{payload_name}/camera_info"),
            ],
            output="screen",
        )
        image_rotate = Node(
            package="tools",
            executable="image_rotate",
            parameters=[
                {
                    "input_topic": f"/{payload_name}/image_raw/compressed",
                    "output_topic": f"/{payload_name}/camera",
                    "compressed": True,
                    "degrees": 180.0,
                }
            ],
            output="screen",
        )
        actions.extend([v4l2_cam, image_rotate])

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("payload_name", default_value="payload_0"),
            DeclareLaunchArgument("controller", default_value=""),
            DeclareLaunchArgument("use_camera", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
