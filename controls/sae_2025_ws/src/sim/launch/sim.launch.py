#!/usr/bin/env python3

import importlib
import json
import logging
import os
import shutil
from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim.constants import (
    Competition,
    COMPETITION_NAMES,
    DEFAULT_COMPETITION,
    DEFAULT_USE_SCORING,
)
from sim.utils import (
    build_node_arguments,
    copy_models_to_gazebo,
    copy_px4_models,
    extract_models_from_sdf,
    find_package_resource,
    load_yaml_to_dict,
)

def load_sim_launch_parameters():
    """Load simulation launch parameters from YAML."""
    # Try source location first (for development - no rebuild needed)
    source_paths = [
        Path(__file__).parent / 'launch_params.yaml',
        Path(os.getcwd()) / 'src' / 'sim' / 'launch' / 'launch_params.yaml',
    ]
    
    for path in source_paths:
        if path.exists():
            return load_yaml_to_dict(path)
    
    from ament_index_python.packages import get_package_share_directory
    package_share = Path(get_package_share_directory('sim'))
    installed_params = package_share / 'launch' / 'launch_params.yaml'
    if installed_params.exists():
        return load_yaml_to_dict(installed_params)
    
    # If not found, raise error
    raise FileNotFoundError(
        f"Launch params file not found. Checked source paths: {source_paths} "
        f"and installed location."
    )

def load_sim_parameters(competition: str, logger: logging.Logger) -> str:
    """
    Find simulation configuration file, checking source location first (for development),
    then falling back to installed location.
    
    Args:
        competition: Competition name (e.g., 'in_house')
        logger: Logger object
        
    Returns:
        Path to the simulation config file (as string)
    """
    config_filename = f"{competition}.yaml"
    config_path = find_package_resource(
        relative_path=f"simulations/{config_filename}",
        package_name='sim',
        resource_type='file',
        logger=logger,
        base_file=Path(__file__)
    )
    return load_yaml_to_dict(config_path), config_path

def initialize_mode(logger: logging.Logger, node_path: str, params: dict) -> Node:
    """
    Initialize a node from a class path and parameters.
    
    Args:
        node_path: Dot-separated path to the node class (e.g., 'sim.world_gen.HoopCourseNode')
        params: Dictionary of parameters to pass to the node constructor
        
    Returns:
        Initialized node instance
    """
    logger.debug(f"Initializing mode: {node_path} with params: {params}")
    
    # Parse node path
    try:
        module_name, class_name = node_path.rsplit(".", 1)
    except ValueError:
        raise ValueError(f"Invalid node path format: '{node_path}'. Expected 'module.ClassName'")
    
    # Import module
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise ImportError(f"Failed to import module '{module_name}': {e}")
    
    # Get class
    if not hasattr(module, class_name):
        raise AttributeError(f"Class '{class_name}' not found in module '{module_name}'")
    
    node_class = getattr(module, class_name)
    
    # Build arguments from parameters
    try:
        args = build_node_arguments(node_class, params)
        logger.debug(f"Initializing {class_name} with args: {list(args.keys())}")
    except ValueError as e:
        raise ValueError(f"Failed to build arguments for {node_path}: {e}")
    
    # Instantiate node
    try:
        return node_class(**args)
    except Exception as e:
        logger.error(f"Failed to initialize {node_path} with args {args}: {e}")
        raise

def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""

    logger = launch.logging.get_logger('sim.launch')
    
    # Get launch arguments
    try:
        params = load_sim_launch_parameters()
    except Exception as e:
        raise RuntimeError(f"Failed to load launch parameters: {e}")
    
    # Map competition numbers to names
    competition_num = params.get("competition", DEFAULT_COMPETITION.value)
    try:
        competition_type = Competition(competition_num)
        competition = COMPETITION_NAMES[competition_type]
    except (ValueError, KeyError):
        valid_values = [e.value for e in Competition]
        raise ValueError(
            f"Invalid competition: {competition_num}. Must be one of {valid_values}"
        )

    scoring_param = params.get("use_scoring", DEFAULT_USE_SCORING)

    if isinstance(scoring_param, str):
        use_scoring = scoring_param.lower() == 'true'
    else:
        use_scoring = bool(scoring_param)


    px4_path_raw = LaunchConfiguration("px4_path").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)

    if px4_path_raw is None:
        raise RuntimeError("PX4 path is required")

    px4_path = os.path.expanduser(px4_path_raw)
    sae_ws_path = os.path.expanduser(os.getcwd())

    # Clear both models and worlds directories for clean state
    models_dir = Path.home() / '.simulation-gazebo' / 'models'
    worlds_dir = Path.home() / '.simulation-gazebo' / 'worlds'

    if models_dir.exists():
        shutil.rmtree(models_dir)
        logger.info(f"Cleared models directory: {models_dir}")

    if worlds_dir.exists():
        shutil.rmtree(worlds_dir)
        logger.info(f"Cleared worlds directory: {worlds_dir}")

    # Copy PX4 models
    logger.info(f"Copying PX4 vehicle model: {vehicle_model}")
    copy_px4_models(px4_path, [vehicle_model])

    sim_params, sim_config_path = load_sim_parameters(competition, logger)

    if "world" not in sim_params:
        raise ValueError(f"Missing 'world' section in simulation config: {sim_config_path}")

    world_params = sim_params["world"]

    # Infer world class name from competition name (convert snake_case to PascalCase)
    world_class_name = ''.join(word.capitalize() for word in competition.split('_'))
    world_file_name = competition

    # Generate world file
    logger.info(f"Generating world file for competition: {competition}")
    try:
        module_name = f"sim.world_gen.{world_class_name}"
        world_module = importlib.import_module(module_name)
        world_generator_class = getattr(world_module, world_class_name)

        # Create generator instance
        world_generator = world_generator_class(**world_params)

        # Call generate_world with output directory
        world_generator.generate_world(worlds_dir)

        # Store hoop positions for passing to node
        hoop_positions = world_generator.hoop_positions
        logger.info(f"World file generated successfully with {len(hoop_positions)} hoops")
    except Exception as e:
        logger.error(f"Failed to generate world file: {e}")
        raise

    # Copy sim models after world generation
    logger.info("Copying sim models...")
    try:
        src_models_dir = find_package_resource(
            relative_path='world_gen/models',
            package_name='sim',
            resource_type='directory',
            logger=logger,
            base_file=Path(__file__)
        )

        # Find all .sdf world files
        world_files = list(worlds_dir.glob('*.sdf'))
        if not world_files:
            logger.warning(f"No world files found in {worlds_dir}")
        else:
            # Extract models from all world files
            all_required_models = set()
            for world_file in world_files:
                models = extract_models_from_sdf(world_file)
                all_required_models.update(models)
                logger.info(f"Found {len(models)} sim models in {world_file.name}: {models}")

            if all_required_models:
                logger.info(f"Copying {len(all_required_models)} sim models: {sorted(all_required_models)}")
                dst_models_dir = Path.home() / '.simulation-gazebo' / 'models'
                copy_models_to_gazebo(src_models_dir, dst_models_dir, models=list(all_required_models))
            else:
                logger.warning("No sim models detected in world files")
    except Exception as e:
        logger.error(f"Failed to copy sim models: {e}")
        raise

    spawn_world = ExecuteProcess(
        cmd=[
            "python3",
            "Tools/simulation/gz/simulation-gazebo",
            f"--world={world_file_name}",
        ],
        cwd=px4_path,
        output="screen",
        name="spawn_world",
    )

    # Start world node for services (like hoop list)
    # Pass hoop_positions directly to avoid regenerating
    world_node_params = {'hoop_positions': hoop_positions}

    world = Node(
        package="sim",
        executable=competition,
        arguments=[json.dumps(world_node_params)],
        output="screen",
        name=world_class_name,
        cwd=sae_ws_path,
    )

    actions = [
        spawn_world,
        world,
    ]

    # Initialize scoring node if requested
    scoring = None
    if use_scoring:
        if "scoring" not in sim_params:
            logger.warning(f"Scoring requested but no 'scoring' section in sim config: {sim_config_path}")
        else:
            scoring = Node(
                package="sim",
                executable="hoop_scoring_node",
                arguments=[json.dumps(sim_params["scoring"])],
                output="screen",
                name="HoopScoringNode",
                cwd=sae_ws_path,
            )

            actions.append(scoring)
    
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            DeclareLaunchArgument("vehicle_model", default_value="x500_mono_cam"),
            OpaqueFunction(function=launch_setup),
        ]
    )
