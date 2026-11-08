#!/usr/bin/env python3
"""Vendor third-party ROS interface IDLs into gcs/idl/ for the GCS code generator.

The monorepo's own interface packages (ros_ws/src) are read from the colcon install
space by generate_rs.py and are never vendored. Everything else they or the GCS use
(std_msgs, sensor_msgs, example_interfaces, ...) is copied here from the sourced ROS
environment, along with every IDL it #includes.

Run in a shell with ROS sourced (source /opt/ros/$ROS_DISTRO/setup.bash).

Examples:
  # one service type (and its dependencies)
  python3 gcs/scripts/add_interface.py example_interfaces/srv/AddTwoInts
  # a whole package
  python3 gcs/scripts/add_interface.py std_srvs
  # everything the monorepo's interface packages depend on (needs ros_ws built)
  python3 gcs/scripts/add_interface.py --monorepo-deps
"""

import argparse
import filecmp
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from interface_sources import (  # noqa: E402
    DEFAULT_IDL_ROOT,
    DEFAULT_ROS_WS_INSTALL,
    InterfaceSourceError,
    ament_share_dirs,
    idl_includes,
    installed_share_dir,
    monorepo_interface_packages,
    parse_type_name,
)

KINDS = ("msg", "srv")


class Vendorer:
    def __init__(self, share_dirs, idl_root, monorepo_packages, force=False, dry_run=False):
        self.share_dirs = [pathlib.Path(share) for share in share_dirs]
        self.idl_root = pathlib.Path(idl_root)
        self.monorepo_packages = set(monorepo_packages)
        self.force = force
        self.dry_run = dry_run
        self.added, self.updated, self.unchanged = [], [], []
        self.conflicts, self.missing = [], []
        self._seen = set()

    def find(self, rel_path):
        for share in self.share_dirs:
            candidate = share / rel_path
            if candidate.is_file():
                return candidate
        return None

    def request_type(self, name):
        """Queue 'pkg', 'pkg/Name', 'pkg/msg/Name' or 'pkg/srv/Name'."""
        if "/" not in name.strip("/"):
            return self.request_package(name.strip("/"))
        package, kind, type_name = parse_type_name(name)
        if package in self.monorepo_packages:
            print(f"{name}: provided by ros_ws (monorepo package), nothing to vendor")
            return []
        kinds = [kind] if kind else list(KINDS)
        for candidate_kind in kinds:
            rel_path = f"{package}/{candidate_kind}/{type_name}.idl"
            if self.find(rel_path):
                return [rel_path]
        self.missing.append(name)
        return []

    def request_package(self, package):
        if package in self.monorepo_packages:
            print(f"{package}: provided by ros_ws (monorepo package), nothing to vendor")
            return []
        for share in self.share_dirs:
            package_dir = share / package
            if package_dir.is_dir():
                paths = sorted(
                    f"{package}/{kind}/{path.name}"
                    for kind in KINDS
                    for path in (package_dir / kind).glob("*.idl")
                )
                if paths:
                    return paths
        self.missing.append(package)
        return []

    def vendor(self, rel_paths):
        queue = list(rel_paths)
        while queue:
            rel_path = queue.pop(0)
            if rel_path in self._seen:
                continue
            self._seen.add(rel_path)
            if rel_path.split("/", 1)[0] in self.monorepo_packages:
                continue  # comes from the ros_ws install space instead
            source = self.find(rel_path)
            if source is None:
                self.missing.append(rel_path)
                continue
            self._copy(source, rel_path)
            queue.extend(idl_includes(source))

    def _copy(self, source, rel_path):
        destination = self.idl_root / rel_path
        if destination.is_file():
            if filecmp.cmp(source, destination, shallow=False):
                self.unchanged.append(rel_path)
                return
            if not self.force:
                self.conflicts.append(rel_path)
                return
            self.updated.append(rel_path)
        else:
            self.added.append(rel_path)
        if not self.dry_run:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

    def report(self):
        prefix = "would " if self.dry_run else ""
        for label, items in (
            (f"{prefix}add", self.added),
            (f"{prefix}update", self.updated),
        ):
            for item in items:
                print(f"  {label}: {item}")
        print(
            f"{len(self.added)} {prefix}added, {len(self.updated)} {prefix}updated, "
            f"{len(self.unchanged)} already up to date"
        )
        ok = True
        if self.conflicts:
            ok = False
            print(
                "\nThese vendored IDLs differ from the sourced ROS environment "
                "(different ROS distro?). Re-run with --force to overwrite:",
                file=sys.stderr,
            )
            for item in self.conflicts:
                print(f"  {item}", file=sys.stderr)
        if self.missing:
            ok = False
            print("\nNot found in any share directory:", file=sys.stderr)
            for item in self.missing:
                print(f"  {item}", file=sys.stderr)
            print(
                "Is the package installed (e.g. apt install ros-$ROS_DISTRO-<pkg>) "
                "and is ROS sourced?",
                file=sys.stderr,
            )
        return ok


def monorepo_dependency_includes(install_base, packages):
    """Includes of every IDL in the monorepo's built interface packages."""
    includes = []
    for package in packages:
        share = installed_share_dir(install_base, package)
        for kind in KINDS:
            for path in sorted((share / package / kind).glob("*.idl")):
                includes.extend(idl_includes(path))
    return includes


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "types",
        nargs="*",
        metavar="TYPE",
        help="pkg/srv/Name, pkg/msg/Name, pkg/Name, or a whole package name",
    )
    parser.add_argument(
        "--monorepo-deps",
        action="store_true",
        help="vendor every third-party IDL used by the monorepo's interface packages",
    )
    parser.add_argument(
        "--share",
        action="append",
        type=pathlib.Path,
        help="share/ directory to copy from (repeatable; default: $AMENT_PREFIX_PATH)",
    )
    parser.add_argument("--idl-root", type=pathlib.Path, default=DEFAULT_IDL_ROOT)
    parser.add_argument("--ros-ws-install", type=pathlib.Path, default=DEFAULT_ROS_WS_INSTALL)
    parser.add_argument("--force", action="store_true", help="overwrite differing vendored IDLs")
    parser.add_argument("--dry-run", action="store_true", help="show what would change")
    args = parser.parse_args(argv)

    if not args.types and not args.monorepo_deps:
        parser.error("give at least one TYPE or --monorepo-deps")

    share_dirs = args.share or ament_share_dirs()
    if not share_dirs:
        parser.error(
            "no ROS environment found: source /opt/ros/$ROS_DISTRO/setup.bash or pass --share"
        )

    try:
        monorepo_packages = monorepo_interface_packages()
        vendorer = Vendorer(
            share_dirs, args.idl_root, monorepo_packages, force=args.force, dry_run=args.dry_run
        )
        requested = []
        for name in args.types:
            requested.extend(vendorer.request_type(name))
        if args.monorepo_deps:
            requested.extend(monorepo_dependency_includes(args.ros_ws_install, monorepo_packages))
        vendorer.vendor(requested)
    except InterfaceSourceError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    if not vendorer.report():
        return 1
    if (vendorer.added or vendorer.updated) and not args.dry_run:
        print("\nNext: regenerate the crate with  python3 gcs/generate_rs.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
