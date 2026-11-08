"""Locate ROS interface IDL files for the GCS code generator.

Standard library only, so the helper scripts work without a ROS install.

Each interface package has exactly one source:
  * vendored: gcs/idl/<pkg>/{msg,srv}/*.idl, copied in by scripts/add_interface.py
  * monorepo: an interface package under ros_ws/src, read from the colcon install
    space so the GCS always matches what the robot was built with
"""

import os
import pathlib
import re
import subprocess
import xml.etree.ElementTree as ET

GCS_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = GCS_DIR.parent
DEFAULT_IDL_ROOT = GCS_DIR / "idl"
ROS_WS = REPO_ROOT / "ros_ws"
DEFAULT_ROS_WS_INSTALL = ROS_WS / "install"

# Interface packages under ros_ws/src that come from outside this repo. Git-tracked
# discovery already skips submodules; this list is for when git isn't available.
THIRD_PARTY_PACKAGES = {"px4_msgs", "ros_gz_interfaces", "actuator_msgs"}

_INCLUDE_RE = re.compile(r'^\s*#include\s+[<"]([^>"]+)[>"]', re.MULTILINE)


class InterfaceSourceError(ValueError):
    pass


def _is_interface_package(package_xml):
    try:
        root = ET.parse(package_xml).getroot()
    except ET.ParseError as error:
        raise InterfaceSourceError(f"Could not parse {package_xml}: {error}") from error
    groups = [element.text for element in root.iter("member_of_group")]
    return "rosidl_interface_packages" in groups, root.findtext("name")


def _package_xmls(ros_ws_src):
    """package.xml files tracked by this repo (excludes submodules and untracked clones)."""
    try:
        output = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "ls-files", "--", f"{ros_ws_src}/*package.xml"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        return [REPO_ROOT / line for line in output.splitlines() if line]
    except (OSError, subprocess.CalledProcessError):
        return sorted(ros_ws_src.rglob("package.xml"))


def monorepo_interface_packages(ros_ws_src=ROS_WS / "src"):
    """Names of the interface packages defined in this monorepo's ros_ws/src."""
    ros_ws_src = pathlib.Path(ros_ws_src)
    if not ros_ws_src.is_dir():
        return []
    packages = set()
    for package_xml in _package_xmls(ros_ws_src):
        is_interface, name = _is_interface_package(package_xml)
        if is_interface and name and name not in THIRD_PARTY_PACKAGES:
            packages.add(name)
    return sorted(packages)


def installed_share_dir(install_base, package):
    """The directory containing <package>/ in a colcon install space.

    colcon's default isolated layout is install/<pkg>/share/<pkg>; --merge-install
    (used by CI) gives install/share/<pkg>.
    """
    install_base = pathlib.Path(install_base)
    for share in (install_base / package / "share", install_base / "share"):
        if (share / package).is_dir():
            return share
    raise InterfaceSourceError(
        f"Interface package '{package}' is not built in {install_base}. Build it with:\n"
        f"  cd ros_ws && colcon build --packages-select {package}"
    )


def vendored_packages(idl_root):
    idl_root = pathlib.Path(idl_root)
    if not idl_root.is_dir():
        return []
    return sorted(
        path.name for path in idl_root.iterdir() if path.is_dir() and any(path.rglob("*.idl"))
    )


def build_sources(idl_root=DEFAULT_IDL_ROOT, install_base=None, monorepo_packages=None):
    """Map each interface package name to the root directory that contains <pkg>/.

    install_base=None means vendored packages only.
    """
    sources = {package: pathlib.Path(idl_root) for package in vendored_packages(idl_root)}
    if install_base is None:
        return sources
    if monorepo_packages is None:
        monorepo_packages = monorepo_interface_packages()
    for package in monorepo_packages:
        if package in sources:
            raise InterfaceSourceError(
                f"'{package}' is a monorepo interface package but is also vendored in "
                f"{idl_root / package}. Delete the vendored copy; it is read from the "
                "ros_ws install space instead."
            )
        sources[package] = installed_share_dir(install_base, package)
    return sources


def parse_type_name(name):
    """'pkg/srv/Name' or 'pkg/msg/Name' -> (pkg, kind, Name). 'pkg/Name' -> kind None."""
    parts = name.strip().strip("/").split("/")
    if len(parts) == 3 and parts[1] in ("msg", "srv"):
        return parts[0], parts[1], parts[2]
    if len(parts) == 2:
        return parts[0], None, parts[1]
    raise InterfaceSourceError(f"Expected 'pkg/msg/Name' or 'pkg/srv/Name', got {name!r}")


def idl_includes(idl_path):
    """Relative paths ('pkg/msg/Name.idl') of the IDLs this file #includes."""
    text = pathlib.Path(idl_path).read_text(encoding="utf-8")
    return [include for include in _INCLUDE_RE.findall(text) if include.endswith(".idl")]


def ament_share_dirs():
    """share/ directories of every prefix in the sourced ROS environment."""
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "")
    return [pathlib.Path(prefix) / "share" for prefix in prefixes.split(os.pathsep) if prefix]
