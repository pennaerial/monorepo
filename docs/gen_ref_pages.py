"""
Generate the code reference pages.
Referenced this guide: https://mkdocstrings.github.io/recipes/
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mkdocs_gen_files
from doc_utils import find_ros_packages

root = Path(__file__).resolve().parent.parent
src = root / "controls" / "sae_2025_ws" / "src"

# Auto-detect Python module roots.
# A root is any dir with __init__.py whose parent is a direct-child ROS package
# of src.
PYTHON_PACKAGE_MODULE_ROOTS = {
    child
    for pkg_dir in find_ros_packages(src, ament_type="python")
    for child in pkg_dir.iterdir()
    if child.is_dir() and (child / "__init__.py").exists()
}

# -----------------------------
# Skip ROS/non-Python API dirs
# -----------------------------
PY_PKG_SKIP_DIRS = {"launch", "config", "urdf", "meshes", "worlds", "models"}

SKIP_FILES = {
    "setup",
}

nav = mkdocs_gen_files.Nav()

for path in sorted(src.rglob("*.py")):
    # restict to only listed python packages
    valid_module = False
    for pkg_module in PYTHON_PACKAGE_MODULE_ROOTS:
        if str(path).startswith(str(pkg_module)):
            valid_module = True

    if not valid_module:
        continue

    # ex: uav/uav/modes/Mode
    module_path = path.relative_to(src).with_suffix("")
    parts = list(module_path.parts)

    if not parts:
        continue

    # skip irrelevant directories within package.
    # only relevant code in ros packages are the inner dir, like uav/uav or sim/sim
    # launch files can be hand documented
    if any(part in PY_PKG_SKIP_DIRS for part in parts):
        continue

    if parts[-1] in SKIP_FILES:
        continue

    doc_path = module_path.with_suffix(".md")
    full_doc_path = Path("reference", doc_path)

    if parts[-1] == "__init__":
        parts = parts[:-1]
        doc_path = doc_path.with_name("index.md")
        full_doc_path = full_doc_path.with_name("index.md")
    elif parts[-1] == "__main__":
        continue

    nav[parts] = doc_path.as_posix()

    with mkdocs_gen_files.open(full_doc_path, "w") as fd:
        identifier = ".".join(parts)
        print(f"::: {identifier}", file=fd)

    # print(full_doc_path)
    mkdocs_gen_files.set_edit_path(full_doc_path, path.relative_to(root))

with mkdocs_gen_files.open("reference/SUMMARY.md", "w") as nav_file:
    nav_file.writelines(nav.build_literate_nav())
