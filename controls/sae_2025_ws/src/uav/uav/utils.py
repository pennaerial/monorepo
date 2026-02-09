import os
import re
import yaml
import glob
from enum import IntEnum
from pathlib import Path
import xml.etree.ElementTree as ET

R_earth = 6378137.0  # Earth's radius in meters (WGS84)

pink = ((140, 120, 120), (175, 255, 255))
green = ((30, 110, 20), (40, 255, 255))
blue = ((85, 120, 60), (140, 255, 255))
yellow = ((10, 100, 100), (30, 255, 255))
vehicle_id_dict = {'quadcopter': 4010, 'tiltrotor_vtol': 4020, 'fixed_wing': 4003,
                   'standard_vtol': 4004, 'quadtailsitter': 4018}
'''
Airframe IDs
All PX4 supported IDs can be found here: https://docs.px4.io/main/en/airframes/airframe_reference
However, IDs available for simulation can be found in PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
'''
vehicle_camera_map = {
    # Standard PX4 Sim Models (Update mapping as needed)
    'gz_x500': False,
    'gz_x500_mono_cam': True,    
    'gz_x500_mono_cam_down': True,  
    'gz_x500_depth': True,       
    'gz_standard_vtol': False,
    'gz_tiltrotor': False,
    'gz_rc_cessna': False,

    # Custom/Team Models (Add custom model names below)
}

class Vehicle(IntEnum):
   """Vehicle class enumeration."""
   MULTICOPTER = 0
   PLANE = 1
   VTOL = 2
   OTHER = 3
   UNKNOWN = 4

def camel_to_snake(name):
    # Convert CamelCase to snake_case.
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def find_folder(folder_name, search_path):
    for root, dirs, files in os.walk(search_path):
        if folder_name in dirs:
            return os.path.join(root, folder_name)
    return None

def find_folder_with_heuristic(folder_name, home_dir=None, keywords=('penn', 'air')):
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
            d for d in os.listdir(home_dir)
            if os.path.isdir(os.path.join(home_dir, d))
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

def extract_vision_nodes(yaml_path):
    """
    Reads the mission YAML file, retrieves the vision node source files from
    os.getcwd()/src/uav/uav/autonomous_modes/, searches for imports from uav.vision_nodes,
    and returns a set of vision node class names.
    """
    vision_nodes = set()
    with open(yaml_path, 'r') as f:
        mission_config = yaml.safe_load(f)
    
    for mode, config in mission_config.items():
        class_path = config.get('class')
        if class_path:
            # Extract the class name from the fully qualified path.
            _, _, class_name = class_path.rpartition('.')
            # Build the file path assuming the file is located at:
            # os.getcwd()/src/uav/uav/autonomous_modes/{class_name}.py
            file_path = os.path.join(os.getcwd(), 'src', 'uav', 'uav', 'autonomous_modes', f"{class_name}.py")
            try:
                with open(file_path, 'r', encoding='utf-8') as source_file:
                    source = source_file.read()
                    # Look for any "from uav.vision_nodes import ..." lines.
                    matches = re.findall(r'from\s+uav\.vision_nodes\s+import\s+([^\n]+)', source)
                    for match in matches:
                        # Allow for multiple imports on the same line, e.g., "A, B"
                        imported_nodes = [n.strip() for n in match.split(',')]
                        for node in imported_nodes:
                            if node:
                                vision_nodes.add(node)
            except Exception as e:
                print(f"Error processing {file_path}: {e}")
    return vision_nodes

def get_airframe_details(px4_path, airframe_id):
    """
    Parses PX4 airframe files to find vehicle type and model name from an ID.
    Returns: (vehicle_class, model_name)
    Example: (4001) -> (Vehicle.MULTICOPTER, 'x500')
    """
    # 1. Locate the Airframe File
    # PX4 stores these in ROMFS/px4fmu_common/init.d-posix/airframes
    # Filenames format: "4001_gz_x500" (ID_NAME)
    airframes_dir = os.path.join(px4_path, 'ROMFS', 'px4fmu_common', 'init.d-posix', 'airframes')
    
    # Find any file starting with the ID
    matches = glob.glob(os.path.join(airframes_dir, f"{airframe_id}_*"))
    
    if not matches:
        print(f"Warning: Airframe ID {airframe_id} not found in {airframes_dir}")
        return Vehicle.UNKNOWN, 'gz_ERROR'

    # 2. Extract Model Name from Filename
    filename = os.path.basename(matches[0])
    # Ex. "4001_gz_x500" --> "x500"
    model_name = "_".join(filename.split('_')[1:]) 

    # 3. Parse File Content for Vehicle Class
    with open(matches[0], 'r') as f:
        content = f.read()
        
        if 'rc.mc_defaults' in content:
            vehicle_class = Vehicle.MULTICOPTER
        elif 'rc.fw_defaults' in content:
            vehicle_class = Vehicle.PLANE
        elif 'rc.vtol_defaults' in content:
            vehicle_class = Vehicle.VTOL
        else:
            vehicle_class = Vehicle.OTHER

    return vehicle_class, model_name

def load_launch_parameters():
    params_file = os.path.join(os.getcwd(), 'src', 'uav', 'launch', 'launch_params.yaml')
    with open(params_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def clean_text(text):
    """Remove ANSI escape codes from text."""
    ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
    return ansi_escape.sub('', text).strip()