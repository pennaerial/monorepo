import { describe, expect, it } from "vitest";
import {
  calibrationChoicesFromFleetSchema,
  coerceFleetFieldValue,
  deleteObjectPathValue,
  emptyFleetEditorDocument,
  getObjectPathValue,
  hasFleetFieldValue,
  normalizeFleetEditorDocument,
  normalizeIntegrationFleetDocument,
  parseFleetEditorDocument,
  renderFleetBlockValue,
  renderFleetEditorDocument,
  setObjectPathValue,
  uniqueFleetVehicleName,
  writeFleetFieldValue,
} from "./fleet-document.js";
import type { FleetSchema, SchemaField } from "./types.js";

describe("normalizeFleetEditorDocument / emptyFleetEditorDocument", () => {
  it("coerces missing backend/defaults/vehicles into their canonical shapes", () => {
    const doc = normalizeFleetEditorDocument({});
    expect(doc).toEqual({ backend: {}, defaults: {}, vehicles: [] });
  });

  it("drops non-object vehicle entries", () => {
    const doc = normalizeFleetEditorDocument({ vehicles: [{ name: "a" }, "not an object", 42, null] });
    expect(doc.vehicles).toEqual([{ name: "a" }]);
  });

  it("emptyFleetEditorDocument defaults backend.kind to hardware", () => {
    expect(emptyFleetEditorDocument()).toEqual({ backend: { kind: "hardware" }, defaults: {}, vehicles: [] });
  });
});

describe("parseFleetEditorDocument / renderFleetEditorDocument", () => {
  it("parses real fleet YAML via the yaml package", () => {
    const text = "backend:\n  kind: hardware\ndefaults:\n  camera_calibration_file: cal.yaml\nvehicles:\n  - name: air-01\n    kind: uav\n";
    const result = parseFleetEditorDocument(text);
    expect(result.error).toBe("");
    expect(result.doc?.vehicles).toEqual([{ name: "air-01", kind: "uav" }]);
  });

  it("returns an empty doc for blank input", () => {
    const result = parseFleetEditorDocument("");
    expect(result.error).toBe("");
    expect(result.doc).toEqual(emptyFleetEditorDocument());
  });

  it("captures a parse error without throwing", () => {
    const result = parseFleetEditorDocument("backend: [unterminated");
    expect(result.doc).toBeNull();
    expect(result.error).toBeTruthy();
  });

  it("round-trips parse -> render -> parse", () => {
    const original = parseFleetEditorDocument(
      "backend:\n  kind: hardware\nvehicles:\n  - name: air-01\n    kind: uav\n",
    );
    const rendered = renderFleetEditorDocument(original.doc);
    const reparsed = parseFleetEditorDocument(rendered);
    expect(reparsed.doc).toEqual(original.doc);
  });
});

describe("normalizeIntegrationFleetDocument", () => {
  it("forces backend.kind to hardware and a default px4_path", () => {
    const doc = normalizeIntegrationFleetDocument({ backend: {} });
    expect(doc.backend).toEqual({ kind: "hardware", px4_path: "~/PX4-Autopilot" });
  });

  it("preserves an explicit px4_path", () => {
    const doc = normalizeIntegrationFleetDocument({ backend: { px4_path: "/custom/path" } });
    expect(doc.backend.px4_path).toBe("/custom/path");
  });

  it("strips defaults.auto_launch", () => {
    const doc = normalizeIntegrationFleetDocument({ defaults: { auto_launch: true, other: 1 } });
    expect(doc.defaults).toEqual({ other: 1 });
  });

  it("derives vehicle.mission from mission_path when mission is unset, and drops legacy keys", () => {
    const doc = normalizeIntegrationFleetDocument({
      vehicles: [
        {
          name: "air-01",
          mission_path: "/repo/uav/missions/patrol.yaml",
          controllable: true,
          px4_instance: 1,
          payload_controller: "x",
          auto_launch: true,
        },
      ],
    });
    expect(doc.vehicles[0]).toEqual({ name: "air-01", mission: "patrol" });
  });

  it("infers vehicle.kind from missionTargetByName when kind is unset", () => {
    const doc = normalizeIntegrationFleetDocument(
      { vehicles: [{ name: "pay-01", mission: "corner_pickup" }] },
      { corner_pickup: "payload" },
    );
    expect(doc.vehicles[0].kind).toBe("payload");
  });

  it("strips px4_airframe_id/px4_namespace for non-uav vehicles", () => {
    const doc = normalizeIntegrationFleetDocument({
      vehicles: [{ name: "pay-01", kind: "payload", px4_airframe_id: 5, px4_namespace: "x" }],
    });
    expect(doc.vehicles[0]).not.toHaveProperty("px4_airframe_id");
    expect(doc.vehicles[0]).not.toHaveProperty("px4_namespace");
  });

  it("keeps px4_airframe_id/px4_namespace for uav vehicles", () => {
    const doc = normalizeIntegrationFleetDocument({
      vehicles: [{ name: "air-01", kind: "uav", px4_airframe_id: 5, px4_namespace: "x" }],
    });
    expect(doc.vehicles[0].px4_airframe_id).toBe(5);
    expect(doc.vehicles[0].px4_namespace).toBe("x");
  });
});

describe("getObjectPathValue / setObjectPathValue / deleteObjectPathValue", () => {
  it("gets a nested value by mixed string/number path", () => {
    const root = { vehicles: [{ name: "a" }, { name: "b", px4_airframe_id: 3 }] };
    expect(getObjectPathValue(root, ["vehicles", 1, "px4_airframe_id"])).toBe(3);
    expect(getObjectPathValue(root, ["vehicles", 5, "px4_airframe_id"])).toBeUndefined();
  });

  it("sets a value, auto-vivifying intermediate objects/arrays", () => {
    const root: Record<string, unknown> = {};
    setObjectPathValue(root, ["vehicles", 0, "name"], "air-01");
    expect(root).toEqual({ vehicles: [{ name: "air-01" }] });
  });

  it("deletes a value by path, removing array entries via splice", () => {
    const root = { vehicles: [{ name: "a" }, { name: "b" }] };
    deleteObjectPathValue(root, ["vehicles", 0]);
    expect(root.vehicles).toEqual([{ name: "b" }]);
  });

  it("deletes an object key", () => {
    const root = { vehicles: [{ name: "a", kind: "uav" }] };
    deleteObjectPathValue(root, ["vehicles", 0, "kind"]);
    expect(root.vehicles[0]).toEqual({ name: "a" });
  });
});

describe("hasFleetFieldValue", () => {
  it("treats false and 0 as present, not absent", () => {
    expect(hasFleetFieldValue(false)).toBe(true);
    expect(hasFleetFieldValue(0)).toBe(true);
  });
  it("treats empty string/array/object as absent", () => {
    expect(hasFleetFieldValue("")).toBe(false);
    expect(hasFleetFieldValue([])).toBe(false);
    expect(hasFleetFieldValue({})).toBe(false);
    expect(hasFleetFieldValue(undefined)).toBe(false);
  });
  it("treats a non-empty array/object/string as present", () => {
    expect(hasFleetFieldValue([1])).toBe(true);
    expect(hasFleetFieldValue({ a: 1 })).toBe(true);
    expect(hasFleetFieldValue("x")).toBe(true);
  });
});

describe("coerceFleetFieldValue / writeFleetFieldValue", () => {
  const boolField: SchemaField = { name: "controllable", schemaType: "boolean" };
  const numberField: SchemaField = { name: "px4_instance", schemaType: "integer" };
  const tupleField: SchemaField = { name: "offset", schemaType: "tuple" };
  const textField: SchemaField = { name: "mission", schemaType: "string" };

  it("coerces booleans/numbers/tuples/text per schemaFieldInputKind", () => {
    expect(coerceFleetFieldValue(boolField, "anything")).toBe(true);
    expect(coerceFleetFieldValue(numberField, "3")).toBe(3);
    expect(coerceFleetFieldValue(numberField, "")).toBeUndefined();
    expect(coerceFleetFieldValue(tupleField, [1, 2, 3])).toEqual([1, 2, 3]);
    expect(coerceFleetFieldValue(tupleField, [1, "x", 3])).toBeUndefined();
    expect(coerceFleetFieldValue(textField, "patrol")).toBe("patrol");
    expect(coerceFleetFieldValue(textField, "")).toBeUndefined();
  });

  it("writeFleetFieldValue deletes the path when the coerced value is empty", () => {
    const doc = normalizeFleetEditorDocument({ vehicles: [{ name: "a", mission: "patrol" }] });
    const updated = writeFleetFieldValue(doc, ["vehicles", 0, "mission"], textField, "");
    expect(updated.vehicles[0]).not.toHaveProperty("mission");
  });

  it("writeFleetFieldValue sets the path when the coerced value is present, including false/0", () => {
    const doc = normalizeFleetEditorDocument({ vehicles: [{ name: "a" }] });
    const updated = writeFleetFieldValue(doc, ["vehicles", 0, "controllable"], boolField, false);
    expect(updated.vehicles[0].controllable).toBe(false);
  });
});

describe("renderFleetBlockValue", () => {
  it("passes strings through as-is", () => {
    expect(renderFleetBlockValue("raw text")).toBe("raw text");
  });
  it("stringifies non-string values as YAML", () => {
    expect(renderFleetBlockValue({ a: 1 })).toBe("a: 1");
  });
  it("returns '' for null/undefined", () => {
    expect(renderFleetBlockValue(null)).toBe("");
    expect(renderFleetBlockValue(undefined)).toBe("");
  });
});

describe("uniqueFleetVehicleName", () => {
  it("returns the base name if unused", () => {
    const doc = normalizeFleetEditorDocument({ vehicles: [{ name: "air-01" }] });
    expect(uniqueFleetVehicleName(doc, "air-02")).toBe("air-02");
  });
  it("appends an incrementing suffix on collision", () => {
    const doc = normalizeFleetEditorDocument({ vehicles: [{ name: "vehicle" }] });
    expect(uniqueFleetVehicleName(doc)).toBe("vehicle_1");
  });
});

describe("calibrationChoicesFromFleetSchema", () => {
  it("pulls camera_calibration_file choices out of the defaults section", () => {
    const schema: FleetSchema = {
      sections: [
        { name: "backend", fields: [] },
        { name: "defaults", fields: [{ name: "camera_calibration_file", choices: ["a.yaml", "b.yaml"] }] },
      ],
    };
    expect(calibrationChoicesFromFleetSchema(schema)).toEqual(["a.yaml", "b.yaml"]);
  });
  it("returns [] when the section/field is missing", () => {
    expect(calibrationChoicesFromFleetSchema({ sections: [] })).toEqual([]);
    expect(calibrationChoicesFromFleetSchema(null)).toEqual([]);
  });
});
