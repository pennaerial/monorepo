#!/usr/bin/env python3
"""
Utility functions for the sim package.
"""

import ast
import inspect
import os
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Optional, Union

import yaml

def load_yaml_to_dict(params_file: Path) -> dict:
    """Load and validate YAML file."""
    try:
        with open(params_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            if data is None:
                raise ValueError(f"YAML file is empty: {params_file}")
            return data
    except yaml.YAMLError as e:
        raise ValueError(f"Invalid YAML in {params_file}: {e}")
    except Exception as e:
        raise RuntimeError(f"Failed to load {params_file}: {e}")

def convert_parameter_value(param_value: Any, param_annotation: Any, param_name: str) -> Any:
    """
    Convert a parameter value to match its type annotation.
    
    Args:
        param_value: The parameter value (may be a string from YAML)
        param_annotation: The type annotation from the function signature
        param_name: The parameter name (for error messages)
        
    Returns:
        The converted parameter value
        
    Raises:
        ValueError: If conversion fails
    """
    # If value is already the correct type or annotation is empty, return as-is
    if param_annotation in (str, inspect.Parameter.empty):
        return param_value
    
    # If value is not a string, return as-is (already converted)
    if not isinstance(param_value, str):
        return param_value
    
    # Skip special cases that shouldn't be evaluated
    if param_name in ("node"):
        return param_value
    
    # Try to convert string to appropriate type using literal_eval
    try:
        converted = ast.literal_eval(param_value)
    except (ValueError, SyntaxError) as e:
        raise ValueError(
            f"Parameter '{param_name}' must be a valid literal. "
            f"Received: {param_value}. Error: {e}"
        )
    
    # Convert lists to tuples if annotation expects a tuple
    if (
        hasattr(param_annotation, "__origin__")
        and param_annotation.__origin__ is tuple
        and isinstance(converted, list)
    ):
        converted = tuple(converted)
    
    return converted


def build_node_arguments(node_class: type, params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Build arguments dictionary for node initialization from parameters.
    
    Args:
        node_class: The node class to instantiate
        params: Dictionary of parameter values from config
        
    Returns:
        Dictionary of arguments ready to pass to node_class.__init__
        
    Raises:
        ValueError: If required parameters are missing or invalid
    """
    signature = inspect.signature(node_class.__init__)
    args = {}
    
    for name, param in signature.parameters.items():
        if name == "self":
            continue
        
        # Get parameter value
        if name in params:
            param_value = params[name]
        elif param.default != inspect.Parameter.empty:
            param_value = param.default
        else:
            raise ValueError(
                f"Missing required parameter '{name}'. "
                f"Available parameters: {list(params.keys())}"
            )
        
        # Convert parameter value to match annotation
        try:
            param_value = convert_parameter_value(
                param_value, param.annotation, name
            )
        except ValueError as e:
            raise ValueError(f"Invalid parameter '{name}': {e}")
        
        args[name] = param_value
    
    return args


def find_package_resource(
    relative_path: Union[str, Path],
    package_name: str = 'sim',
    resource_type: str = 'file',
    logger: Optional[Any] = None,
    base_file: Optional[Path] = None
) -> Path:
    """
    Find a package resource (file or directory), checking source location first (for development),
    then falling back to installed location.
    
    This allows editing resources during development without needing to rebuild.
    
    Args:
        relative_path: Path relative to package root (e.g., 'simulations/in_house.yaml' or 'world_gen/models')
        package_name: ROS2 package name (default: 'sim')
        resource_type: 'file' or 'directory' (default: 'file')
        logger: Optional logger for info messages (if None, no logging)
        base_file: File to use as reference for source paths (typically Path(__file__) from caller)
        
    Returns:
        Path to the resource
        
    Raises:
        FileNotFoundError: If resource cannot be found
    """
    relative_path = Path(relative_path)
    if base_file is None:
        # Default to utils.py location if not provided (fallback)
        base_file = Path(__file__)
    else:
        base_file = Path(base_file)
    
    # Build source paths (for development - no rebuild needed)
    source_paths = [
        base_file.parent / relative_path,
        base_file.parent.parent / relative_path,
        Path(os.getcwd()) / 'src' / package_name / package_name / relative_path,
    ]
    
    # Check source locations first
    for path in source_paths:
        path = path.resolve()
        if resource_type == 'file' and path.exists() and path.is_file():
            if logger:
                logger.info(f"Using source {resource_type} (development): {path}")
            return path
        elif resource_type == 'directory' and path.exists() and path.is_dir():
            if logger:
                logger.info(f"Using source {resource_type} (development): {path}")
            return path
    
    # Fallback to installed location (for production/deployed packages)
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = Path(get_package_share_directory(package_name))
        installed_path = package_share / relative_path
        installed_path = installed_path.resolve()
        
        if resource_type == 'file' and installed_path.exists() and installed_path.is_file():
            if logger:
                logger.info(f"Using installed {resource_type}: {installed_path}")
            return installed_path
        elif resource_type == 'directory' and installed_path.exists() and installed_path.is_dir():
            if logger:
                logger.info(f"Using installed {resource_type}: {installed_path}")
            return installed_path
    except Exception as e:
        if logger:
            logger.debug(f"Could not get package share directory: {e}")
    
    # If not found, raise error
    all_paths = source_paths + [installed_path if 'installed_path' in locals() else "N/A"]
    resource_name = relative_path.name if relative_path.name else str(relative_path)
    raise FileNotFoundError(
        f"{resource_type.capitalize()} '{resource_name}' not found. "
        f"Checked source paths: {source_paths} and installed location."
    )


def _get_model_dependencies(model_path: Path) -> list[str]:
    """
    Parse model.sdf to find model:// dependencies.

    Args:
        model_path: Path to the model directory

    Returns:
        List of model names that this model depends on
    """
    dependencies = []
    model_sdf = model_path / 'model.sdf'

    if not model_sdf.exists():
        return dependencies

    try:
        with open(model_sdf, 'r') as f:
            content = f.read()
            # Find all model:// references (handles both model://name and model://name/path/to/file)
            uri_patterns = re.findall(r'model://([^/<>]+)', content)
            dependencies.extend(uri_patterns)
            # Also find plain <uri>model_name</uri> without model:// prefix (used by PX4 models)
            plain_uri_patterns = re.findall(r'<uri>([a-zA-Z0-9_-]+)</uri>', content)
            dependencies.extend(plain_uri_patterns)
    except Exception:
        # Silently ignore parse errors for dependency detection
        pass

    return dependencies

def _copy_model_with_dependencies(
    src_models_dir: Path,
    dst_models_dir: Path,
    model_name: str,
    copied_models: set[str]
) -> None:
    """
    Recursively copy a model and its dependencies.

    Args:
        src_models_dir: Source models directory
        dst_models_dir: Destination models directory
        model_name: Name of model to copy
        copied_models: Set of already-copied model names (modified in-place)
    """
    # Skip if already copied
    if model_name in copied_models:
        return

    src_model = src_models_dir / model_name
    if not src_model.exists():
        # Model might be a built-in Gazebo model or PX4 model, skip
        return

    # Copy the model
    dst_model = dst_models_dir / model_name
    if src_model.is_dir():
        shutil.copytree(src_model, dst_model, dirs_exist_ok=True)

    copied_models.add(model_name)

    # Recursively copy dependencies
    dependencies = _get_model_dependencies(src_model)
    for dep in dependencies:
        _copy_model_with_dependencies(src_models_dir, dst_models_dir, dep, copied_models)

def copy_models_to_gazebo(src_models_dir: Path, dst_models_dir: Path, models: Optional[list[str]] = None) -> None:
    """
    Copy sim model files to Gazebo models directory.

    Args:
        src_models_dir: Source models directory (sim package models)
        dst_models_dir: Destination models directory (~/.simulation-gazebo/models)
        models: Optional list of specific sim model names to copy. If None, copies all models.
    """
    if not src_models_dir.exists():
        raise FileNotFoundError(f"Source models directory does not exist: {src_models_dir}")

    if not src_models_dir.is_dir():
        raise ValueError(f"Source path is not a directory: {src_models_dir}")

    # Create destination directory if needed
    try:
        dst_models_dir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        raise OSError(f"Failed to create destination directory {dst_models_dir}: {e}")

    # Track which models have been copied to avoid duplicates
    copied_models = set()

    # Determine which models to copy
    if models is None:
        # Copy all models in the directory
        models_to_copy = [item.name for item in src_models_dir.iterdir() if item.is_dir()]
    else:
        models_to_copy = models

    # Copy each model with dependencies
    try:
        for model_name in models_to_copy:
            _copy_model_with_dependencies(src_models_dir, dst_models_dir, model_name, copied_models)
    except (OSError, shutil.Error) as e:
        raise OSError(f"Failed to copy models from {src_models_dir} to {dst_models_dir}: {e}")

def extract_models_from_sdf(sdf_path: Path) -> list[str]:
    """
    Parse an SDF file and extract all model:// URI references.

    This function scans an SDF world file for <uri>model://...</uri> tags
    and returns a list of unique model names that need to be copied to
    the Gazebo models directory.

    Args:
        sdf_path: Path to the SDF world file

    Returns:
        List of unique model names referenced in the world file.
        Returns empty list if file doesn't exist or parsing fails.

    Example:
        For <uri>model://hoop</uri>, returns ['hoop']
        For <uri>model://dlz_red</uri>, returns ['dlz_red']
    """
    if not sdf_path.exists():
        return []

    try:
        tree = ET.parse(str(sdf_path))
        root = tree.getroot()

        models = set()
        # Find all <uri> elements containing model:// references
        for uri_elem in root.iter('uri'):
            if uri_elem.text and uri_elem.text.startswith('model://'):
                model_name = uri_elem.text.replace('model://', '').strip()
                models.add(model_name)

        return sorted(list(models))
    except ET.ParseError as e:
        # Log parse error but don't crash - just return empty list
        return []

def copy_px4_models(px4_path: str, models: list[str]) -> None:
    """
    Copy required PX4 vehicle models from PX4-Autopilot to Gazebo models directory.

    Clears ~/.simulation-gazebo/models/ first to ensure a clean state,
    then recursively copies the specified models and all their dependencies.

    This is called before sim.launch starts, setting up the base PX4 models that
    will later be merged with sim models (hoop, dlz, etc.) by WorldNode.

    Args:
        px4_path: Path to PX4-Autopilot directory
        models: List of model names to copy (e.g., ['x500_mono_cam'])
    """
    px4_models_dir = Path(px4_path) / 'Tools' / 'simulation' / 'gz' / 'models'
    dst_models_dir = Path.home() / '.simulation-gazebo' / 'models'

    # Clear destination directory first to ensure clean state
    if dst_models_dir.exists():
        shutil.rmtree(dst_models_dir)

    # Create destination directory
    dst_models_dir.mkdir(parents=True, exist_ok=True)

    # Track models to copy and already copied to avoid duplicates
    models_to_copy = set(models)
    copied_models = set()

    while models_to_copy:
        model_name = models_to_copy.pop()

        if model_name in copied_models:
            continue

        src_model = px4_models_dir / model_name
        dst_model = dst_models_dir / model_name

        if not src_model.exists():
            print(f"Warning: PX4 model '{model_name}' not found in {px4_models_dir}")
            continue

        # Copy the model
        if src_model.is_dir():
            shutil.copytree(src_model, dst_model)
            copied_models.add(model_name)
            print(f"Copied PX4 model: {model_name}")

            # Find and queue dependencies
            dependencies = _get_model_dependencies(src_model)
            for dep in dependencies:
                if dep not in copied_models:
                    models_to_copy.add(dep)

def camel_to_snake(name: str) -> str:
    """Convert CamelCase to snake_case."""
    return re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name).lower()