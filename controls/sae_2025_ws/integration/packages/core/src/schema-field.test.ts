import { describe, expect, it } from "vitest";
import {
  defaultParamsRawForMode,
  effectiveSchemaFieldValue,
  fieldHelpText,
  normalizeModeRegistry,
  schemaFieldInputKind,
  writeSchemaFieldValue,
} from "./schema-field.js";
import type { ModeMetadata, SchemaField } from "./types.js";

describe("schemaFieldInputKind", () => {
  it("returns select when choices are present, regardless of schemaType", () => {
    expect(schemaFieldInputKind({ name: "x", choices: ["a", "b"], schemaType: "string" })).toBe("select");
  });
  it("maps boolean/bool to boolean", () => {
    expect(schemaFieldInputKind({ name: "x", schemaType: "boolean" })).toBe("boolean");
    expect(schemaFieldInputKind({ name: "x", schemaType: "bool" })).toBe("boolean");
  });
  it("maps integer/number/int/float to number", () => {
    for (const t of ["integer", "number", "int", "float"] as const) {
      expect(schemaFieldInputKind({ name: "x", schemaType: t })).toBe("number");
    }
  });
  it("maps tuple to tuple", () => {
    expect(schemaFieldInputKind({ name: "x", schemaType: "tuple" })).toBe("tuple");
  });
  it("maps array/object/union/list/dict to block", () => {
    for (const t of ["array", "object", "union", "list", "dict"] as const) {
      expect(schemaFieldInputKind({ name: "x", schemaType: t })).toBe("block");
    }
  });
  it("defaults to text, and handles a missing field", () => {
    expect(schemaFieldInputKind({ name: "x" })).toBe("text");
    expect(schemaFieldInputKind(null)).toBe("text");
  });
});

describe("fieldHelpText", () => {
  it("prefers description when present", () => {
    expect(fieldHelpText({ name: "x", description: "the real help text" })).toBe("the real help text");
  });
  it("falls back to annotation unless it's a low-signal bare type name", () => {
    expect(fieldHelpText({ name: "x", annotation: "the vehicle's target altitude in meters" })).toBe(
      "the vehicle's target altitude in meters",
    );
    expect(fieldHelpText({ name: "x", annotation: "int" })).toBe("");
    expect(fieldHelpText({ name: "x", annotation: "bool" })).toBe("");
  });
  it("returns '' when neither is present", () => {
    expect(fieldHelpText({ name: "x" })).toBe("");
  });
});

describe("effectiveSchemaFieldValue / writeSchemaFieldValue", () => {
  const boolField: SchemaField = { name: "enabled", schemaType: "boolean" };
  const numberField: SchemaField = { name: "speed", schemaType: "float", default: 1.5 };
  const tupleField: SchemaField = { name: "position", schemaType: "tuple" };
  const textField: SchemaField = { name: "label", schemaType: "string", default: "fallback" };

  it("reads the field's default when absent from paramsRaw", () => {
    expect(effectiveSchemaFieldValue("", boolField)).toBe("");
    expect(effectiveSchemaFieldValue("", numberField)).toBe(1.5);
  });

  it("reads and coerces a boolean field", () => {
    expect(effectiveSchemaFieldValue("enabled: true", boolField)).toBe(true);
    expect(effectiveSchemaFieldValue("enabled: false", boolField)).toBe(false);
  });

  it("reads and coerces a number field, falling back to default on garbage", () => {
    expect(effectiveSchemaFieldValue("speed: 3.5", numberField)).toBe(3.5);
    expect(effectiveSchemaFieldValue("speed: notanumber", numberField)).toBe(1.5);
  });

  it("reads and coerces a tuple field", () => {
    expect(effectiveSchemaFieldValue("position: [1, 2, 3]", tupleField)).toEqual([1, 2, 3]);
  });

  it("reads a text field as-is", () => {
    expect(effectiveSchemaFieldValue("label: hello", textField)).toBe("hello");
  });

  it("writes each kind back into paramsRaw and round-trips through a read", () => {
    let raw = "";
    raw = writeSchemaFieldValue(raw, boolField, true);
    expect(effectiveSchemaFieldValue(raw, boolField)).toBe(true);

    raw = writeSchemaFieldValue(raw, numberField, 9.9);
    expect(effectiveSchemaFieldValue(raw, numberField)).toBe(9.9);

    raw = writeSchemaFieldValue(raw, tupleField, [4, 5, 6]);
    expect(effectiveSchemaFieldValue(raw, tupleField)).toEqual([4, 5, 6]);

    raw = writeSchemaFieldValue(raw, textField, "new label");
    expect(effectiveSchemaFieldValue(raw, textField)).toBe("new label");
  });
});

describe("normalizeModeRegistry", () => {
  it("flattens {target: entries[]} into a lookup keyed by both public mode id and class path", () => {
    const takeoff: ModeMetadata = {
      name: "Takeoff",
      mode: "uav.modes.takeoff.Takeoff",
      classPath: "uav.modes.takeoff:Takeoff",
      params: [],
    };
    const flat = normalizeModeRegistry({ uav: [takeoff] });
    expect(flat["takeoff.Takeoff"]).toBe(takeoff);
    expect(flat["uav.modes.takeoff:Takeoff"]).toBe(takeoff);
  });

  it("handles an empty/undefined registry", () => {
    expect(normalizeModeRegistry(undefined)).toEqual({});
    expect(normalizeModeRegistry({})).toEqual({});
  });
});

describe("defaultParamsRawForMode", () => {
  it("builds a default params fragment, skipping missing/null/array/object defaults", () => {
    const metadata: ModeMetadata = {
      name: "Takeoff",
      mode: "takeoff.Takeoff",
      params: [
        { name: "target_altitude", schemaType: "float", default: 10.0 },
        { name: "enabled", schemaType: "boolean", default: true },
        { name: "offset", schemaType: "tuple", default: [1, 2, 3] },
        { name: "no_default", schemaType: "string" },
        { name: "explicitly_missing", schemaType: "string", default: 5, defaultKind: "missing" },
        { name: "list_field", schemaType: "array", default: [1, 2] },
      ],
    };
    const raw = defaultParamsRawForMode(metadata);
    expect(raw).toBe("target_altitude: 10\nenabled: true\noffset: [1, 2, 3]");
  });

  it("returns '' for a mode with no params", () => {
    expect(defaultParamsRawForMode({ name: "x", mode: "x", params: [] })).toBe("");
    expect(defaultParamsRawForMode(null)).toBe("");
  });
});
