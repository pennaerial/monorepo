import * as YAML from "yaml";
import { schemaFieldInputKind } from "./schema-field.js";
import type {
  FleetEditorDocument,
  FleetSchema,
  ObjectPathSegment,
  ParsedFleetDocument,
  SchemaField,
} from "./types.js";

export function cloneYamlValue<T>(value: T): T {
  if (value === null || value === undefined) return value;
  return JSON.parse(JSON.stringify(value)) as T;
}

export function normalizeFleetEditorDocument(raw: unknown): FleetEditorDocument {
  const doc = (raw && typeof raw === "object" && !Array.isArray(raw) ? cloneYamlValue(raw) : {}) as Record<
    string,
    unknown
  >;

  if (!doc.backend || typeof doc.backend !== "object" || Array.isArray(doc.backend)) {
    doc.backend = {};
  }
  if (!doc.defaults || typeof doc.defaults !== "object" || Array.isArray(doc.defaults)) {
    doc.defaults = {};
  }
  if (!Array.isArray(doc.vehicles)) {
    doc.vehicles = [];
  }
  doc.vehicles = (doc.vehicles as unknown[])
    .filter((vehicle) => vehicle && typeof vehicle === "object" && !Array.isArray(vehicle))
    .map((vehicle) => ({ ...(vehicle as Record<string, unknown>) }));

  return doc as unknown as FleetEditorDocument;
}

export function emptyFleetEditorDocument(): FleetEditorDocument {
  return normalizeFleetEditorDocument({
    backend: { kind: "hardware" },
    defaults: {},
    vehicles: [],
  });
}

/**
 * Unlike mission YAML, fleet YAML uses a real parser (the `yaml` npm
 * package) -- fleet docs don't need the same comment/formatting-preserving
 * surgical-edit treatment mission YAML does.
 */
export function parseFleetEditorDocument(text: string | undefined | null): ParsedFleetDocument {
  const rawText = `${text ?? ""}`;
  if (!rawText.trim()) {
    return { rawText, doc: emptyFleetEditorDocument(), error: "" };
  }

  try {
    const parsed = YAML.parse(rawText) as unknown;
    return { rawText, doc: normalizeFleetEditorDocument(parsed || {}), error: "" };
  } catch (error) {
    return { rawText, doc: null, error: (error as Error)?.message || "Failed to parse fleet YAML." };
  }
}

export function renderFleetEditorDocument(doc: unknown): string {
  return YAML.stringify(normalizeFleetEditorDocument(doc), { lineWidth: 0 }).trimEnd();
}

export function missionNameFromPathString(path: string | undefined | null): string {
  const normalized = `${path ?? ""}`.trim().replace(/\\/g, "/");
  if (!normalized) return "";
  const basename = normalized.split("/").pop() || "";
  return basename.replace(/\.ya?ml$/i, "");
}

/**
 * Migration/compat normalizer applied on every fleet-doc load and save:
 * canonicalizes whatever the user pasted/loaded into the current fleet
 * schema, dropping legacy per-vehicle keys and deriving `mission`/`kind`.
 * Mirrors the old dashboard's `normalizeIntegrationFleetDocument`.
 */
export function normalizeIntegrationFleetDocument(
  doc: unknown,
  missionTargetByName: Record<string, string> = {},
): FleetEditorDocument {
  const next = normalizeFleetEditorDocument(doc);
  const backend =
    next.backend && typeof next.backend === "object" && !Array.isArray(next.backend) ? { ...next.backend } : {};
  next.backend = {
    kind: "hardware",
    px4_path: `${backend.px4_path || "~/PX4-Autopilot"}`.trim() || "~/PX4-Autopilot",
  };

  const defaults =
    next.defaults && typeof next.defaults === "object" && !Array.isArray(next.defaults) ? { ...next.defaults } : {};
  delete defaults.auto_launch;
  next.defaults = defaults;

  next.vehicles = (next.vehicles || []).map((vehicle) => {
    const normalizedVehicle: Record<string, unknown> = { ...vehicle };
    const missionName = `${
      normalizedVehicle.mission || missionNameFromPathString(normalizedVehicle.mission_path as string) || ""
    }`.trim();
    const inferredKind = `${missionTargetByName[missionName] || normalizedVehicle.kind || ""}`.trim();

    delete normalizedVehicle.controllable;
    delete normalizedVehicle.mission_path;
    delete normalizedVehicle.px4_instance;
    delete normalizedVehicle.payload_controller;
    delete normalizedVehicle.auto_launch;

    if (missionName) {
      normalizedVehicle.mission = missionName;
    } else {
      delete normalizedVehicle.mission;
    }

    if (inferredKind) {
      normalizedVehicle.kind = inferredKind;
    } else {
      delete normalizedVehicle.kind;
    }

    if (`${normalizedVehicle.kind || ""}`.trim() !== "uav") {
      delete normalizedVehicle.px4_airframe_id;
      delete normalizedVehicle.px4_namespace;
    }

    return normalizedVehicle;
  });

  return next;
}

export function getObjectPathValue(root: unknown, path: ObjectPathSegment[]): unknown {
  return path.reduce((current: unknown, key) => {
    if (current === null || current === undefined) return undefined;
    if (typeof key === "number") {
      return Array.isArray(current) ? current[key] : undefined;
    }
    return typeof current === "object" ? (current as Record<string, unknown>)[key] : undefined;
  }, root);
}

export function setObjectPathValue(root: unknown, path: ObjectPathSegment[], value: unknown): void {
  if (!path.length) return;
  let current = root as Record<string, unknown> | unknown[];
  for (let index = 0; index < path.length - 1; index += 1) {
    const key = path[index];
    const nextKey = path[index + 1];
    const nextValue: Record<string, unknown> | unknown[] = typeof nextKey === "number" ? [] : {};
    if (typeof key === "number") {
      if (!Array.isArray(current)) return;
      current[key] = current[key] ?? nextValue;
      current = current[key] as Record<string, unknown> | unknown[];
      continue;
    }
    const obj = current as Record<string, unknown>;
    if (!obj[key] || typeof obj[key] !== "object") {
      obj[key] = nextValue;
    }
    current = obj[key] as Record<string, unknown> | unknown[];
  }
  const finalKey = path[path.length - 1];
  if (typeof finalKey === "number") {
    if (Array.isArray(current)) {
      current[finalKey] = value;
    }
    return;
  }
  (current as Record<string, unknown>)[finalKey] = value;
}

export function deleteObjectPathValue(root: unknown, path: ObjectPathSegment[]): void {
  if (!path.length) return;
  let current: unknown = root;
  for (let index = 0; index < path.length - 1; index += 1) {
    const key = path[index];
    if (typeof key === "number") {
      if (!Array.isArray(current)) return;
      current = current[key];
      continue;
    }
    if (!current || typeof current !== "object") return;
    current = (current as Record<string, unknown>)[key];
  }
  if (!current || typeof current !== "object") return;
  const finalKey = path[path.length - 1];
  if (typeof finalKey === "number") {
    if (Array.isArray(current)) {
      current.splice(finalKey, 1);
    }
    return;
  }
  delete (current as Record<string, unknown>)[finalKey];
}

/** "Is this value non-empty/non-default" -- explicitly true for `false`/`0`, so those aren't mistaken for absent. */
export function hasFleetFieldValue(value: unknown): boolean {
  if (value === false || value === 0) return true;
  if (Array.isArray(value)) {
    return value.some((item) => hasFleetFieldValue(item));
  }
  if (value && typeof value === "object") {
    return Object.keys(value).length > 0;
  }
  return `${value ?? ""}`.trim() !== "";
}

export function coerceFleetFieldValue(field: SchemaField, nextValue: unknown): unknown {
  const kind = schemaFieldInputKind(field);
  if (kind === "boolean") {
    return Boolean(nextValue);
  }
  if (kind === "number") {
    const raw = `${nextValue ?? ""}`.trim();
    if (!raw) return undefined;
    const parsed = Number(raw);
    return Number.isFinite(parsed) ? parsed : undefined;
  }
  if (kind === "tuple") {
    const arr = Array.isArray(nextValue) ? nextValue : [];
    const normalized = arr.map((value) => Number(value));
    return normalized.every((value) => Number.isFinite(value)) ? normalized : undefined;
  }
  if (kind === "block") {
    return nextValue;
  }
  const text = `${nextValue ?? ""}`;
  return text.trim() ? text : undefined;
}

export function writeFleetFieldValue(
  doc: unknown,
  path: ObjectPathSegment[],
  field: SchemaField,
  nextValue: unknown,
): FleetEditorDocument {
  const nextDoc = normalizeFleetEditorDocument(doc);
  const normalized = coerceFleetFieldValue(field, nextValue);
  if (!hasFleetFieldValue(normalized)) {
    deleteObjectPathValue(nextDoc, path);
    return normalizeFleetEditorDocument(nextDoc);
  }
  setObjectPathValue(nextDoc, path, normalized);
  return normalizeFleetEditorDocument(nextDoc);
}

export function renderFleetBlockValue(value: unknown): string {
  if (value === undefined || value === null) return "";
  if (typeof value === "string") return value;
  return YAML.stringify(value, { lineWidth: 0 }).trimEnd();
}

export function uniqueFleetVehicleName(doc: FleetEditorDocument | undefined | null, baseName = "vehicle"): string {
  const existing = new Set(
    (doc?.vehicles || []).map((vehicle) => `${vehicle?.name || ""}`.trim()).filter(Boolean),
  );
  if (!existing.has(baseName)) return baseName;
  let index = 1;
  while (existing.has(`${baseName}_${index}`)) {
    index += 1;
  }
  return `${baseName}_${index}`;
}

export function calibrationChoicesFromFleetSchema(schema: FleetSchema | undefined | null): string[] {
  const defaultsSection = Array.isArray(schema?.sections)
    ? schema.sections.find((section) => section?.name === "defaults")
    : null;
  const calibrationField = Array.isArray(defaultsSection?.fields)
    ? defaultsSection.fields.find((field) => field?.name === "camera_calibration_file")
    : null;
  return Array.isArray(calibrationField?.choices) ? calibrationField.choices.map((choice) => `${choice}`).filter(Boolean) : [];
}
