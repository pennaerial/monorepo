from __future__ import annotations

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace
import types
from typing import Any


def import_module_if_available(name: str) -> Any | None:
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError:
        return None


def ensure_basic_rclpy_stubs() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy: Any = types.ModuleType("rclpy")
    node_module: Any = types.ModuleType("rclpy.node")

    class Node:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

    node_module.Node = Node
    rclpy.node = node_module
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node_module})


def ensure_runtime_rclpy_stubs() -> None:
    rclpy: Any = sys.modules.get("rclpy") or import_module_if_available("rclpy")
    if rclpy is None:
        rclpy = types.ModuleType("rclpy")
        sys.modules["rclpy"] = rclpy
    if not hasattr(rclpy, "init"):
        rclpy.init = lambda *args, **kwargs: None
    if not hasattr(rclpy, "shutdown"):
        rclpy.shutdown = lambda: None
    if not hasattr(rclpy, "ok"):
        rclpy.ok = lambda: True

    node_mod: Any = sys.modules.get("rclpy.node") or import_module_if_available(
        "rclpy.node"
    )
    if node_mod is None:
        node_mod = types.ModuleType("rclpy.node")
        sys.modules["rclpy.node"] = node_mod
    if not hasattr(node_mod, "Node"):

        class Node:
            def __init__(self, *_args, **_kwargs) -> None:
                pass

        node_mod.Node = Node

    executors_mod: Any = sys.modules.get(
        "rclpy.executors"
    ) or import_module_if_available("rclpy.executors")
    if executors_mod is None:
        executors_mod = types.ModuleType("rclpy.executors")
        sys.modules["rclpy.executors"] = executors_mod
    if not hasattr(executors_mod, "ExternalShutdownException"):

        class ExternalShutdownException(Exception):
            pass

        executors_mod.ExternalShutdownException = ExternalShutdownException

    for module_name, attr_name in (
        ("rclpy.clock", "Clock"),
        ("rclpy.parameter", "Parameter"),
    ):
        module: Any = sys.modules.get(module_name) or import_module_if_available(
            module_name
        )
        if module is None:
            module = types.ModuleType(module_name)
            sys.modules[module_name] = module
        if not hasattr(module, attr_name):
            setattr(module, attr_name, type(attr_name, (), {}))

    validate_namespace_mod: Any = sys.modules.get(
        "rclpy.validate_namespace"
    ) or import_module_if_available("rclpy.validate_namespace")
    if validate_namespace_mod is None:
        validate_namespace_mod = types.ModuleType("rclpy.validate_namespace")
        sys.modules["rclpy.validate_namespace"] = validate_namespace_mod
    if not hasattr(validate_namespace_mod, "validate_namespace"):
        validate_namespace_mod.validate_namespace = lambda namespace: None

    validate_node_name_mod: Any = sys.modules.get(
        "rclpy.validate_node_name"
    ) or import_module_if_available("rclpy.validate_node_name")
    if validate_node_name_mod is None:
        validate_node_name_mod = types.ModuleType("rclpy.validate_node_name")
        sys.modules["rclpy.validate_node_name"] = validate_node_name_mod
    if not hasattr(validate_node_name_mod, "validate_node_name"):
        validate_node_name_mod.validate_node_name = lambda node_name: None

    qos_mod: Any = sys.modules.get("rclpy.qos") or import_module_if_available(
        "rclpy.qos"
    )
    if qos_mod is None:
        qos_mod = types.ModuleType("rclpy.qos")
        sys.modules["rclpy.qos"] = qos_mod
    for name in (
        "QoSProfile",
        "QoSReliabilityPolicy",
        "QoSHistoryPolicy",
        "QoSDurabilityPolicy",
    ):
        if not hasattr(qos_mod, name):
            setattr(qos_mod, name, type(name, (), {}))

    rclpy.node = node_mod
    rclpy.executors = executors_mod
    rclpy.clock = sys.modules["rclpy.clock"]
    rclpy.parameter = sys.modules["rclpy.parameter"]
    rclpy.validate_namespace = validate_namespace_mod
    rclpy.validate_node_name = validate_node_name_mod
    rclpy.qos = qos_mod


def purge_fake_rclpy_modules() -> None:
    fake_rclpy = sys.modules.get("rclpy")
    if fake_rclpy is not None and not hasattr(fake_rclpy, "__path__"):
        for module_name in list(sys.modules):
            if module_name == "rclpy" or module_name.startswith("rclpy."):
                del sys.modules[module_name]


def ensure_std_msgs_stub() -> None:
    std_msgs: Any = sys.modules.get("std_msgs") or import_module_if_available(
        "std_msgs"
    )
    if std_msgs is not None:
        return
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg: Any = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Empty = type("Empty", (), {})
    std_msgs.msg = std_msgs_msg
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg})


def ensure_std_srvs_stub() -> None:
    std_srvs: Any = sys.modules.get("std_srvs") or import_module_if_available(
        "std_srvs"
    )
    if std_srvs is not None:
        return
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv: Any = types.ModuleType("std_srvs.srv")

    class Trigger:
        Request = type("Request", (), {})
        Response = type("Response", (), {})

    std_srvs_srv.Trigger = Trigger
    std_srvs.srv = std_srvs_srv
    sys.modules.update({"std_srvs": std_srvs, "std_srvs.srv": std_srvs_srv})


def ensure_ament_index_stub(package_root: Path) -> None:
    ament_index_python: Any = sys.modules.get(
        "ament_index_python"
    ) or import_module_if_available("ament_index_python")
    if ament_index_python is None:
        ament_index_python = types.ModuleType("ament_index_python")
        sys.modules["ament_index_python"] = ament_index_python

    ament_index_packages: Any = sys.modules.get(
        "ament_index_python.packages"
    ) or import_module_if_available("ament_index_python.packages")
    if ament_index_packages is None:
        ament_index_packages = types.ModuleType("ament_index_python.packages")
        sys.modules["ament_index_python.packages"] = ament_index_packages

    if not hasattr(ament_index_packages, "PackageNotFoundError"):

        class PackageNotFoundError(Exception):
            pass

        ament_index_packages.PackageNotFoundError = PackageNotFoundError
    if not hasattr(ament_index_packages, "get_package_share_directory"):
        ament_index_packages.get_package_share_directory = lambda _name: str(
            package_root
        )
    ament_index_python.packages = ament_index_packages


def ensure_launch_import_stubs(
    package_root: Path, *, include_launch_ros: bool = False
) -> None:
    ensure_ament_index_stub(package_root)

    launch_module: Any = sys.modules.get("launch") or import_module_if_available(
        "launch"
    )
    if launch_module is None:
        launch_module = types.ModuleType("launch")
        sys.modules["launch"] = launch_module
    if not hasattr(launch_module, "LaunchDescription"):
        launch_module.LaunchDescription = type("LaunchDescription", (), {})

    launch_actions: Any = sys.modules.get(
        "launch.actions"
    ) or import_module_if_available("launch.actions")
    if launch_actions is None:
        launch_actions = types.ModuleType("launch.actions")
        sys.modules["launch.actions"] = launch_actions
    for name in (
        "DeclareLaunchArgument",
        "ExecuteProcess",
        "IncludeLaunchDescription",
        "OpaqueFunction",
    ):
        if not hasattr(launch_actions, name):
            setattr(launch_actions, name, type(name, (), {}))

    launch_sources: Any = sys.modules.get(
        "launch.launch_description_sources"
    ) or import_module_if_available("launch.launch_description_sources")
    if launch_sources is None:
        launch_sources = types.ModuleType("launch.launch_description_sources")
        sys.modules["launch.launch_description_sources"] = launch_sources
    if not hasattr(launch_sources, "PythonLaunchDescriptionSource"):
        launch_sources.PythonLaunchDescriptionSource = type(
            "PythonLaunchDescriptionSource", (), {}
        )

    launch_logging: Any = sys.modules.get(
        "launch.logging"
    ) or import_module_if_available("launch.logging")
    if launch_logging is None:
        launch_logging = types.ModuleType("launch.logging")
        sys.modules["launch.logging"] = launch_logging
    if not hasattr(launch_logging, "get_logger"):
        launch_logging.get_logger = lambda *_args, **_kwargs: SimpleNamespace(
            warning=lambda *_a, **_k: None,
            warn=lambda *_a, **_k: None,
            info=lambda *_a, **_k: None,
        )

    launch_substitutions: Any = sys.modules.get(
        "launch.substitutions"
    ) or import_module_if_available("launch.substitutions")
    if launch_substitutions is None:
        launch_substitutions = types.ModuleType("launch.substitutions")
        sys.modules["launch.substitutions"] = launch_substitutions
    if not hasattr(launch_substitutions, "LaunchConfiguration"):
        launch_substitutions.LaunchConfiguration = type("LaunchConfiguration", (), {})

    if not include_launch_ros:
        return

    launch_ros: Any = sys.modules.get("launch_ros") or import_module_if_available(
        "launch_ros"
    )
    if launch_ros is None:
        launch_ros = types.ModuleType("launch_ros")
        sys.modules["launch_ros"] = launch_ros

    launch_ros_actions: Any = sys.modules.get(
        "launch_ros.actions"
    ) or import_module_if_available("launch_ros.actions")
    if launch_ros_actions is None:
        launch_ros_actions = types.ModuleType("launch_ros.actions")
        sys.modules["launch_ros.actions"] = launch_ros_actions
    if not hasattr(launch_ros_actions, "Node"):
        launch_ros_actions.Node = type("Node", (), {})
    launch_ros.actions = launch_ros_actions


def install_auto_launch_import_stubs(package_root: Path) -> None:
    ensure_std_msgs_stub()
    ensure_std_srvs_stub()
    ensure_launch_import_stubs(package_root)
    ensure_runtime_rclpy_stubs()


def install_payload_mode_import_stubs() -> None:
    ensure_basic_rclpy_stubs()

    if "cv_bridge" not in sys.modules:
        cv_bridge: Any = types.ModuleType("cv_bridge")

        class CvBridge:
            def imgmsg_to_cv2(self, *_args, **_kwargs):
                raise NotImplementedError

            def cv2_to_compressed_imgmsg(self, *_args, **_kwargs):
                return SimpleNamespace(header=SimpleNamespace(stamp=None))

        cv_bridge.CvBridge = CvBridge
        sys.modules["cv_bridge"] = cv_bridge

    if "sensor_msgs" not in sys.modules:
        sensor_msgs: Any = types.ModuleType("sensor_msgs")
        sensor_msgs_msg: Any = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CompressedImage = type("CompressedImage", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sensor_msgs.msg = sensor_msgs_msg
        sys.modules.update(
            {"sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg}
        )

    if "uav.vehicles.Payload" not in sys.modules:
        payload_module: Any = types.ModuleType("uav.vehicles.Payload")
        payload_module.Payload = type("Payload", (), {})
        sys.modules["uav.vehicles.Payload"] = payload_module

    if "uav.vision_nodes" not in sys.modules:
        vision_nodes: Any = types.ModuleType("uav.vision_nodes")
        vision_nodes.PayloadAprilTagNode = type("PayloadAprilTagNode", (), {})
        sys.modules["uav.vision_nodes"] = vision_nodes

    if "uav.vision_nodes.payload_perception_common" not in sys.modules:
        common: Any = types.ModuleType("uav.vision_nodes.payload_perception_common")
        common.DEFAULT_TAG_FAMILY = "tag36h11"
        sys.modules["uav.vision_nodes.payload_perception_common"] = common

    if "uav_interfaces" not in sys.modules:
        sys.modules["uav_interfaces"] = types.ModuleType("uav_interfaces")

    if "uav_interfaces.srv" not in sys.modules:
        srv_module: Any = types.ModuleType("uav_interfaces.srv")

        class PayloadAprilTagState:
            class Request:
                pass

            class Response:
                pass

        srv_module.PayloadAprilTagState = PayloadAprilTagState
        sys.modules["uav_interfaces.srv"] = srv_module
