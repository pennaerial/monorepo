#!/usr/bin/env python3
"""
Utility functions for the sim package.
"""

import ast
import inspect
import os
import shutil
from pathlib import Path
from typing import Any, Dict, Optional, Union
import yaml
import re

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


def copy_models_to_gazebo(src_models_dir: Path, dst_models_dir: Path, models: Optional[list[str]] = None) -> None:
    """
    Copy model files from source to Gazebo models directory.

    Note: PX4 vehicle models are copied separately by the UAV launch system,
    which clears the directory first.

    Args:
        src_models_dir: Source models directory (sim package models)
        dst_models_dir: Destination models directory (Gazebo models path)
        models: Optional list of specific sim model names to copy. If None, copies all models.

    Raises:
        OSError: If copy operations fail
        FileNotFoundError: If a specified model doesn't exist
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

    # Determine which models to copy
    if models is None:
        # Copy all models
        items_to_copy = list(src_models_dir.iterdir())
    else:
        # Copy only specified models
        items_to_copy = []
        for model_name in models:
            model_path = src_models_dir / model_name
            if not model_path.exists():
                raise FileNotFoundError(f"Required model '{model_name}' not found in {src_models_dir}")
            items_to_copy.append(model_path)

    # Copy the selected sim models
    try:
        for src_item in items_to_copy:
            dst_item = dst_models_dir / src_item.name

            if src_item.is_dir():
                # Copytree with dirs_exist_ok=True to allow merging/updating
                shutil.copytree(src_item, dst_item, dirs_exist_ok=True)
            elif src_item.is_file():
                shutil.copy2(src_item, dst_item)
    except (OSError, shutil.Error) as e:
        raise OSError(f"Failed to copy models from {src_models_dir} to {dst_models_dir}: {e}")

def camel_to_snake(name: str) -> str:
    """Convert CamelCase to snake_case."""
    return re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name).lower()