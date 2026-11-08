"""Tests for interface_sources.py and scripts/add_interface.py. No ROS needed.

python3 -m unittest discover -s gcs/test -p test_interface_tools.py
"""

import pathlib
import sys
import tempfile
import unittest

GCS_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GCS_DIR))
sys.path.insert(0, str(GCS_DIR / "scripts"))

import add_interface  # noqa: E402
import interface_sources  # noqa: E402

TIME_IDL = """module builtin_interfaces { module msg {
    struct Time { int32 sec; uint32 nanosec; };
}; };
"""
HEADER_IDL = """// generated from rosidl_adapter
#include "builtin_interfaces/msg/Time.idl"

module std_msgs { module msg {
    struct Header { builtin_interfaces::msg::Time stamp; string frame_id; };
}; };
"""
ADD_TWO_INTS_IDL = """module example_interfaces { module srv {
    struct AddTwoInts_Request { int64 a; int64 b; };
    struct AddTwoInts_Response { int64 sum; };
}; };
"""
TIMED_DRIVE_IDL = """#include "std_msgs/msg/Header.idl"

module payload_interfaces { module srv {
    struct TimedDrive_Request { std_msgs::msg::Header header; double seconds; };
    struct TimedDrive_Response { boolean success; };
}; };
"""
PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  {group}
</package>
"""


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


class FakeTree(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self._tmp.name)
        # A sourced ROS environment's share/ directory.
        self.share = self.root / "opt/ros/jazzy/share"
        write(self.share / "builtin_interfaces/msg/Time.idl", TIME_IDL)
        write(self.share / "std_msgs/msg/Header.idl", HEADER_IDL)
        write(self.share / "example_interfaces/srv/AddTwoInts.idl", ADD_TWO_INTS_IDL)
        # The GCS's vendored IDL root (starts empty).
        self.idl_root = self.root / "gcs/idl"
        self.idl_root.mkdir(parents=True)
        # A colcon install space with one monorepo interface package (isolated layout).
        self.install = self.root / "ros_ws/install"
        write(
            self.install / "payload_interfaces/share/payload_interfaces/srv/TimedDrive.idl",
            TIMED_DRIVE_IDL,
        )

    def tearDown(self):
        self._tmp.cleanup()

    def vendorer(self, **kwargs):
        return add_interface.Vendorer([self.share], self.idl_root, ["payload_interfaces"], **kwargs)


class InterfaceSourcesTests(FakeTree):
    def test_discovers_only_interface_packages(self):
        src = self.root / "ros_ws/src"
        group = "<member_of_group>rosidl_interface_packages</member_of_group>"
        write(
            src / "a/payload_interfaces/package.xml",
            PACKAGE_XML.format(name="payload_interfaces", group=group),
        )
        write(src / "b/some_node/package.xml", PACKAGE_XML.format(name="some_node", group=""))
        write(src / "c/px4_msgs/package.xml", PACKAGE_XML.format(name="px4_msgs", group=group))
        self.assertEqual(interface_sources.monorepo_interface_packages(src), ["payload_interfaces"])
        self.assertEqual(interface_sources.monorepo_interface_packages(self.root / "nope"), [])

    def test_installed_share_dir_layouts(self):
        isolated = interface_sources.installed_share_dir(self.install, "payload_interfaces")
        self.assertEqual(isolated, self.install / "payload_interfaces/share")
        merged = self.root / "merged"
        (merged / "share/payload_interfaces").mkdir(parents=True)
        self.assertEqual(
            interface_sources.installed_share_dir(merged, "payload_interfaces"), merged / "share"
        )
        with self.assertRaisesRegex(interface_sources.InterfaceSourceError, "colcon build"):
            interface_sources.installed_share_dir(self.install, "uav_interfaces")

    def test_build_sources(self):
        write(self.idl_root / "std_msgs/msg/Header.idl", HEADER_IDL)
        (self.idl_root / "notes").mkdir()  # no IDLs: ignored
        sources = interface_sources.build_sources(self.idl_root)
        self.assertEqual(sources, {"std_msgs": self.idl_root})
        sources = interface_sources.build_sources(
            self.idl_root, self.install, ["payload_interfaces"]
        )
        self.assertEqual(sources["payload_interfaces"], self.install / "payload_interfaces/share")
        # A monorepo package must not also be vendored.
        write(self.idl_root / "payload_interfaces/srv/TimedDrive.idl", TIMED_DRIVE_IDL)
        with self.assertRaisesRegex(interface_sources.InterfaceSourceError, "also vendored"):
            interface_sources.build_sources(self.idl_root, self.install, ["payload_interfaces"])

    def test_parse_type_name_and_includes(self):
        parse = interface_sources.parse_type_name
        self.assertEqual(parse("std_srvs/srv/Trigger"), ("std_srvs", "srv", "Trigger"))
        self.assertEqual(parse("/std_msgs/Header/"), ("std_msgs", None, "Header"))
        with self.assertRaises(interface_sources.InterfaceSourceError):
            parse("a/b/c/d")
        header = self.share / "std_msgs/msg/Header.idl"
        self.assertEqual(
            interface_sources.idl_includes(header), ["builtin_interfaces/msg/Time.idl"]
        )


class AddInterfaceTests(FakeTree):
    def test_vendors_type_with_transitive_includes(self):
        vendorer = self.vendorer()
        vendorer.vendor(vendorer.request_type("std_msgs/Header"))
        self.assertTrue(vendorer.report())
        self.assertEqual(
            sorted(vendorer.added), ["builtin_interfaces/msg/Time.idl", "std_msgs/msg/Header.idl"]
        )
        self.assertEqual((self.idl_root / "std_msgs/msg/Header.idl").read_text(), HEADER_IDL)

        again = self.vendorer()
        again.vendor(again.request_type("std_msgs/msg/Header"))
        self.assertEqual(again.added, [])
        self.assertEqual(len(again.unchanged), 2)

    def test_whole_package_and_services(self):
        vendorer = self.vendorer()
        vendorer.vendor(vendorer.request_type("example_interfaces"))
        self.assertEqual(vendorer.added, ["example_interfaces/srv/AddTwoInts.idl"])

    def test_monorepo_types_are_not_vendored(self):
        vendorer = self.vendorer()
        self.assertEqual(vendorer.request_type("payload_interfaces/srv/TimedDrive"), [])
        self.assertEqual(vendorer.request_type("payload_interfaces"), [])
        self.assertFalse((self.idl_root / "payload_interfaces").exists())

    def test_monorepo_deps(self):
        includes = add_interface.monorepo_dependency_includes(self.install, ["payload_interfaces"])
        self.assertEqual(includes, ["std_msgs/msg/Header.idl"])
        vendorer = self.vendorer()
        vendorer.vendor(includes)
        self.assertTrue(vendorer.report())
        self.assertEqual(
            sorted(vendorer.added), ["builtin_interfaces/msg/Time.idl", "std_msgs/msg/Header.idl"]
        )

    def test_conflicts_missing_force_and_dry_run(self):
        stale = write(self.idl_root / "builtin_interfaces/msg/Time.idl", "// old distro\n")
        vendorer = self.vendorer()
        vendorer.vendor(vendorer.request_type("std_msgs/msg/Header"))
        vendorer.request_type("nonexistent_msgs/msg/Foo")
        self.assertFalse(vendorer.report())
        self.assertEqual(vendorer.conflicts, ["builtin_interfaces/msg/Time.idl"])
        self.assertEqual(vendorer.missing, ["nonexistent_msgs/msg/Foo"])
        self.assertEqual(stale.read_text(), "// old distro\n")

        dry = self.vendorer(force=True, dry_run=True)
        dry.vendor(["builtin_interfaces/msg/Time.idl"])
        self.assertEqual(dry.updated, ["builtin_interfaces/msg/Time.idl"])
        self.assertEqual(stale.read_text(), "// old distro\n")

        forced = self.vendorer(force=True)
        forced.vendor(["builtin_interfaces/msg/Time.idl"])
        self.assertEqual(stale.read_text(), TIME_IDL)

    def test_cli_requires_input(self):
        with self.assertRaises(SystemExit):
            add_interface.main([])
        status = add_interface.main(
            ["--share", str(self.share), "--idl-root", str(self.idl_root), "std_msgs/msg/Header"]
        )
        self.assertEqual(status, 0)
        self.assertTrue((self.idl_root / "builtin_interfaces/msg/Time.idl").is_file())


if __name__ == "__main__":
    unittest.main()
