import os
from pathlib import Path


def px4_path_from_gitmodules(gitmodules):
    # Read the declared submodule path from .gitmodules so resolution survives the submodule moving.
    sections = []
    current = None
    for raw in Path(gitmodules).read_text().splitlines():
        line = raw.strip()
        if line.startswith("[submodule"):
            current = {}
            sections.append(current)
        elif current is not None and "=" in line:
            key, _, value = line.partition("=")
            current[key.strip().lower()] = value.strip()
    for section in sections:
        path = section.get("path", "")
        url = section.get("url", "")
        if "px4-autopilot" in path.lower() or "px4-autopilot" in url.lower():
            return path or None
    return None


def get_px4_submodule_path():
    # Resolve the in-repo PX4-Autopilot submodule path via the repo's .gitmodules.
    # Returns None when the submodule doesn't exist or isn't initialized.
    here = Path(__file__).resolve()
    for parent in here.parents:
        gitmodules = parent / ".gitmodules"
        if gitmodules.is_file():
            rel_path = px4_path_from_gitmodules(gitmodules)
            if rel_path:
                candidate = (parent / rel_path).resolve()
                # Non-empty confirms the submodule is checked out, not a placeholder.
                if candidate.is_dir() and any(candidate.iterdir()):
                    return str(candidate)
            return None
    return None


# Prefer submodule path if available, otherwise fall back to a user-local px4
DEFAULT_PX4_PATH = get_px4_submodule_path() or os.path.expanduser("~/PX4-Autopilot")
