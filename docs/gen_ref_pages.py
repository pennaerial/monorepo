"""
Generate the code reference pages.
Referenced this guide: https://mkdocstrings.github.io/recipes/
"""

from pathlib import Path
import mkdocs_gen_files

root = Path(__file__).resolve().parent.parent
src = root / "controls" / "sae_2025_ws" / "src"

# -----------------------------
# Python package roots (REAL import roots)
# -----------------------------
PYTHON_PACKAGE_MODULE_ROOTS = {
    src / "sim" / "sim",
    src / "uav" / "uav",
    src / "udp_bridge" / "udp_bridge",
    src / "tools",
}

# -----------------------------
# Skip ROS/non-Python API dirs
# -----------------------------
PY_PKG_SKIP_DIRS = {
    "launch",
    "config",
    "urdf",
    "meshes",
    "worlds",
    "models"
}

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
