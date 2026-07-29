#!/usr/bin/env python3
"""Tests for the parts of schema_export.py that don't require a sourced
ROS2 + vehicle_common environment (the JSON-schema shaping helpers and the
filesystem glob helpers). `fleet_schema()`/`mode_registry()`/`schema_index()`
themselves import `vehicle_common` and can't be exercised here -- those need
a real Pi (or a sourced colcon workspace) and are covered by manual/agent
verification instead, same constraint as the rest of this project.

Run with: python3 -m unittest tools/schema-export/test_schema_export.py
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import schema_export


class SchemaFieldTests(unittest.TestCase):
    def test_boolean_field(self) -> None:
        field = schema_export._schema_field("enabled", {"type": "boolean", "default": True}, required=False)
        self.assertEqual(field["schemaType"], "bool")
        self.assertEqual(field["annotation"], "bool")
        self.assertEqual(field["default"], True)
        self.assertEqual(field["defaultKind"], "value")
        self.assertFalse(field["nullable"])
        self.assertEqual(field["choices"], [])

    def test_enum_field(self) -> None:
        field = schema_export._schema_field(
            "network_role", {"enum": ["default", "ap", "client"]}, required=False
        )
        self.assertEqual(field["schemaType"], "enum")
        self.assertEqual(field["choices"], ["default", "ap", "client"])

    def test_tuple_field(self) -> None:
        field = schema_export._schema_field(
            "camera_mount_offsets", {"prefixItems": [{}, {}, {}]}, required=False
        )
        self.assertEqual(field["schemaType"], "tuple")

    def test_ref_field(self) -> None:
        field = schema_export._schema_field("backend", {"$ref": "#/$defs/HardwareBackendModel"}, required=True)
        self.assertEqual(field["annotation"], "HardwareBackendModel")
        self.assertTrue(field["required"])

    def test_nullable_union_collapses_to_the_single_non_null_variant(self) -> None:
        field = schema_export._schema_field(
            "px4_airframe_id",
            {"anyOf": [{"type": "integer"}, {"type": "null"}]},
            required=False,
        )
        self.assertEqual(field["schemaType"], "int")
        self.assertTrue(field["nullable"])

    def test_multi_variant_union_becomes_union_type(self) -> None:
        field = schema_export._schema_field(
            "backend",
            {"anyOf": [{"$ref": "#/$defs/HardwareBackendModel"}, {"$ref": "#/$defs/SimBackendModel"}]},
            required=True,
        )
        self.assertEqual(field["schemaType"], "union")

    def test_missing_default_is_distinguished_from_an_explicit_none_default(self) -> None:
        missing = schema_export._schema_field("x", {"type": "string"}, required=False)
        self.assertEqual(missing["defaultKind"], "missing")
        explicit_none = schema_export._schema_field("x", {"type": "string", "default": None}, required=False)
        self.assertEqual(explicit_none["defaultKind"], "none")


class FieldsFromModelTests(unittest.TestCase):
    def test_derives_required_from_the_schema_required_list(self) -> None:
        schema = {
            "properties": {"name": {"type": "string"}, "mission": {"type": "string"}},
            "required": ["name"],
        }
        fields = schema_export._fields_from_model(schema, [])
        by_name = {f["name"]: f for f in fields}
        self.assertTrue(by_name["name"]["required"])
        self.assertFalse(by_name["mission"]["required"])

    def test_injects_calibration_choices_onto_the_matching_field_only(self) -> None:
        schema = {
            "properties": {
                "camera_calibration_file": {"type": "string"},
                "other": {"type": "string"},
            }
        }
        fields = schema_export._fields_from_model(schema, ["a.yaml", "b.yaml"])
        by_name = {f["name"]: f for f in fields}
        self.assertEqual(by_name["camera_calibration_file"]["choices"], ["a.yaml", "b.yaml"])
        self.assertEqual(by_name["other"]["choices"], [])


class TargetLabelTests(unittest.TestCase):
    def test_uav_package_module_maps_to_uav(self) -> None:
        class FakeUAV:
            pass

        FakeUAV.__module__ = "uav.uav.vehicles.UAV"
        self.assertEqual(schema_export._target_label_for_vehicle_cls(FakeUAV), "uav")

    def test_payload_package_module_maps_to_payload(self) -> None:
        class FakePayload:
            pass

        FakePayload.__module__ = "payload.payload.payload"
        self.assertEqual(schema_export._target_label_for_vehicle_cls(FakePayload), "payload")

    def test_unknown_package_maps_to_empty_string(self) -> None:
        class FakeOther:
            pass

        FakeOther.__module__ = "some_other_package.thing"
        self.assertEqual(schema_export._target_label_for_vehicle_cls(FakeOther), "")


class FilesystemGlobTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tmpdir.name)

    def tearDown(self) -> None:
        self.tmpdir.cleanup()

    def test_available_mission_files_globs_both_uav_and_payload_share_dirs(self) -> None:
        uav_missions = self.root / "install" / "uav" / "share" / "uav" / "missions"
        uav_missions.mkdir(parents=True)
        (uav_missions / "test_takeoff.yaml").write_text("modes: {}")

        payload_missions = self.root / "install" / "payload" / "share" / "payload" / "missions"
        payload_missions.mkdir(parents=True)
        (payload_missions / "corner_navigate.yaml").write_text("modes: {}")

        missions = schema_export._available_mission_files(self.root)
        self.assertEqual(
            sorted(missions, key=lambda m: m["name"]),
            [{"name": "corner_navigate", "target": "payload"}, {"name": "test_takeoff", "target": "uav"}],
        )

    def test_available_mission_files_returns_empty_when_no_install_tree_exists(self) -> None:
        self.assertEqual(schema_export._available_mission_files(self.root), [])

    def test_available_fleet_files_globs_the_uav_share_fleets_dir(self) -> None:
        fleets_dir = self.root / "install" / "uav" / "share" / "uav" / "fleets"
        fleets_dir.mkdir(parents=True)
        (fleets_dir / "example_fleet.yaml").write_text("vehicles: []")
        (fleets_dir / "another.yaml").write_text("vehicles: []")

        self.assertEqual(schema_export._available_fleet_files(self.root), ["another", "example_fleet"])

    def test_available_camera_calibration_files_globs_yaml_and_yml(self) -> None:
        calibrations_dir = self.root / "install" / "uav" / "share" / "uav" / "config" / "camera_calibrations"
        calibrations_dir.mkdir(parents=True)
        (calibrations_dir / "cam1.yaml").write_text("{}")
        (calibrations_dir / "cam2.yml").write_text("{}")

        self.assertEqual(schema_export._available_camera_calibration_files(self.root), ["cam1.yaml", "cam2.yml"])


if __name__ == "__main__":
    unittest.main()
