import os
import glob
import re
from pathlib import Path

from uav.vehicles.AirframeClass import AirframeClass

R_earth = 6378137.0  # Earth's radius in meters (WGS84)

pink = ((140, 120, 120), (175, 255, 255))
green = ((30, 110, 20), (40, 255, 255))
blue = ((85, 120, 60), (140, 255, 255))
yellow = ((10, 100, 100), (30, 255, 255))
red = (((0, 80, 80), (10, 255, 255)), ((160, 80, 80), (179, 255, 255)))
vehicle_id_dict = {
    "quadcopter": 4010,
    "tiltrotor_vtol": 4020,
    "fixed_wing": 4003,
    "standard_vtol": 4004,
    "quadtailsitter": 4018,
}
# Resolves vehicle_class without a PX4 checkout; intended to eventually replace the ROMFS parse.
airframe_class_fallback = {
    4003: AirframeClass.PLANE,
    4004: AirframeClass.VTOL,
    4010: AirframeClass.MULTICOPTER,
    4014: AirframeClass.MULTICOPTER,
    4018: AirframeClass.VTOL,
    4020: AirframeClass.VTOL,
}
"""
Airframe IDs
All PX4 supported IDs can be found here: https://docs.px4.io/main/en/airframes/airframe_reference
However, IDs available for simulation can be found in PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
"""
vehicle_camera_map = {
    # Standard PX4 Sim Models (Update mapping as needed)
    "gz_x500": False,
    "gz_x500_mono_cam": True,
    "gz_x500_mono_cam_down": True,
    "gz_x500_depth": True,
    "gz_standard_vtol": True,
    "gz_tiltrotor": False,
    "gz_rc_cessna": False,
    "gz_quadtailsitter": False,
    # Custom/Team Models (Add custom model names below)
}


def camel_to_snake(name):
    # Convert CamelCase to snake_case.
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def find_folder(folder_name, search_path):
    for root, dirs, files in os.walk(search_path):
        if folder_name in dirs:
            return os.path.join(root, folder_name)
    return None


def find_folder_with_heuristic(folder_name, home_dir=None, keywords=("penn", "air")):
    # Normalize home_dir
    if home_dir is None:
        home_dir = str(Path.home())
    home_dir = os.path.abspath(os.path.expanduser(str(home_dir)))

    # 1) If home_dir itself is the folder, return it
    if os.path.basename(os.path.normpath(home_dir)) == folder_name:
        return home_dir

    # 2) Look at immediate subdirectories
    try:
        immediate_dirs = [
            d for d in os.listdir(home_dir) if os.path.isdir(os.path.join(home_dir, d))
        ]
    except FileNotFoundError:
        # home_dir doesn't exist
        return None

    # 2a) Prefer keyword-matching subdirs (e.g. "pennair")
    for d in immediate_dirs:
        if any(kw.lower() in d.lower() for kw in keywords):
            candidate_root = os.path.join(home_dir, d)
            result = find_folder(folder_name, candidate_root)
            if result:
                return result

    # 2b) If it's directly under home_dir, use that
    if folder_name in immediate_dirs:
        return os.path.join(home_dir, folder_name)

    # 3) Fallback: search entire home_dir
    return find_folder(folder_name, home_dir)


def get_airframe_details(px4_path, airframe_id):
    """
    Parses PX4 airframe files to find vehicle type and model name from an ID.
    Returns: (vehicle_class, model_name)
    Example: (4001) -> (AirframeClass.MULTICOPTER, 'x500')
    """
    # 1. Locate the Airframe File
    # PX4 stores these in ROMFS/px4fmu_common/init.d-posix/airframes
    # Filenames format: "4001_gz_x500" (ID_NAME)
    airframes_dir = (
        os.path.join(px4_path, "ROMFS", "px4fmu_common", "init.d-posix", "airframes")
        if px4_path
        else None
    )

    # Find any file starting with the ID
    matches = glob.glob(os.path.join(airframes_dir, f"{airframe_id}_*")) if airframes_dir else []

    if not matches:
        print(f"AIRFRAME_ID: {airframe_id}")
        fallback_class = airframe_class_fallback.get(int(airframe_id))
        if fallback_class is not None:
            return fallback_class, ""
        print(f"Warning: Airframe ID {airframe_id} not found in {airframes_dir}")
        return AirframeClass.UNKNOWN, "gz_ERROR"

    # 2. Extract Model Name from Filename
    filename = os.path.basename(matches[0])
    # Ex. "4001_gz_x500" --> "x500"
    model_name = "_".join(filename.split("_")[1:])

    # 3. Parse File Content for Vehicle Class
    with open(matches[0], "r") as f:
        content = f.read()

        if "rc.mc_defaults" in content:
            vehicle_class = AirframeClass.MULTICOPTER
        elif "rc.fw_defaults" in content:
            vehicle_class = AirframeClass.PLANE
        elif "rc.vtol_defaults" in content:
            vehicle_class = AirframeClass.VTOL
        else:
            vehicle_class = AirframeClass.OTHER

    return vehicle_class, model_name


def clean_text(text):
    """Remove ANSI escape codes from text."""
    ansi_escape = re.compile(r"\x1b\[[0-9;]*m")
    return ansi_escape.sub("", text).strip()
