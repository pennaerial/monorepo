import os

from vehicle_common.launch_utils import LaunchError

def prepend_env_path(env_var: str, path_to_prepend: str) -> str:
    """Safely prepends a path to an existing PATH environment variable and returns the new value"""

    existing = os.environ.get(env_var, "")
    if not existing:
        return path_to_prepend
    else:
        return f"{path_to_prepend}:{existing}"

def require_env(env_var: str) -> str:
    """Returns a required environment variable or raises a LaunchError."""

    value = os.environ.get(env_var)
    if not value:
        raise LaunchError(
            f"{env_var} is not set. Please source dev_env.sh or manually set it"
        )

    return value

