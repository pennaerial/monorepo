"""Shared utilities for mkdocs-gen-files scripts."""

from pathlib import Path
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
GITMODULES = REPO_ROOT / ".gitmodules"


def submodule_dirs() -> set[Path]:
    """Return absolute paths of all submodules declared in the repo's .gitmodules."""
    if not GITMODULES.exists():
        return set()
    paths: set[Path] = set()
    for line in GITMODULES.read_text().splitlines():
        line = line.strip()
        if line.startswith("path ="):
            rel = line.split("=", 1)[1].strip()
            paths.add(REPO_ROOT / rel)
    return paths


def find_ros_packages(
    src: Path, ament_type="all", exclude_submodules=True
) -> set[Path]:
    """Return the set of ROS package directories under *src*.

    A ROS package is any directory that contains a ``package.xml`` file.
    Only direct children of *src* are searched so that vendored/nested
    package trees (e.g. ``ros_gz/ros_gz_sim/``) are excluded.

    Args:
        src: The source directory to search in.
        ament_type: Filter by build type — ``"all"``, ``"python"``
            (``ament_python``), or ``"cmake"`` (``ament_cmake``).
        exclude_submodules: When ``True`` (default), directories that are
            git submodules are excluded.
    """
    if ament_type not in ("all", "python", "cmake"):
        raise ValueError("ament_type must be 'all', 'python', or 'cmake'")

    src = src.resolve()
    pkg_dirs = {pkg_xml.parent for pkg_xml in src.glob("*/package.xml")}

    if exclude_submodules:
        pkg_dirs -= submodule_dirs()

    if ament_type == "all":
        return pkg_dirs

    target_build_type = "ament_python" if ament_type == "python" else "ament_cmake"

    return {
        pkg_dir
        for pkg_dir in pkg_dirs
        if ET.parse(pkg_dir / "package.xml").getroot().findtext("export/build_type")
        == target_build_type
    }


if __name__ == "__main__":
    print("submodules:")
    for smd in submodule_dirs():
        print(smd)

    print("\nALL:")
    for pkg in find_ros_packages(Path("controls/sae_2025_ws/src")):
        print(pkg)

    print("\nPYTHON:")
    for pkg in find_ros_packages(Path("controls/sae_2025_ws/src"), ament_type="python"):
        print(pkg)

    print("\nCMAKE:")
    for pkg in find_ros_packages(Path("controls/sae_2025_ws/src"), ament_type="cmake"):
        print(pkg)
