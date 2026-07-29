import { getYamlBlockValue, setYamlBlockValue } from "./yaml-text.js";
import { publicModeId } from "./mission-document.js";
import type { ModeMetadata, ModeRegistryByTarget, SchemaField, SchemaFieldInputKind } from "./types.js";

export function schemaFieldInputKind(field: SchemaField | undefined | null): SchemaFieldInputKind {
  if (!field) return "text";
  if (field.choices?.length) return "select";
  if (field.schemaType === "boolean" || field.schemaType === "bool") return "boolean";
  if (
    field.schemaType === "integer" ||
    field.schemaType === "number" ||
    field.schemaType === "int" ||
    field.schemaType === "float"
  ) {
    return "number";
  }
  if (field.schemaType === "tuple") return "tuple";
  if (
    field.schemaType === "array" ||
    field.schemaType === "object" ||
    field.schemaType === "union" ||
    field.schemaType === "list" ||
    field.schemaType === "dict"
  ) {
    return "block";
  }
  return "text";
}

const LOW_SIGNAL_SCHEMA_ANNOTATIONS = new Set(["str", "int", "float", "bool", "enum", "list", "dict", "mapping"]);

export function fieldHelpText(field: SchemaField | undefined | null): string {
  if (field?.description) return field.description;
  const annotation = `${field?.annotation || ""}`.trim().toLowerCase();
  if (!annotation || LOW_SIGNAL_SCHEMA_ANNOTATIONS.has(annotation)) return "";
  return field?.annotation || "";
}

export function effectiveSchemaFieldValue(paramsRaw: string, field: SchemaField): unknown {
  const blockValue = getYamlBlockValue(paramsRaw, field.name);
  if (!blockValue) {
    return field.default ?? "";
  }

  const kind = schemaFieldInputKind(field);
  if (kind === "boolean") {
    return `${blockValue}`.trim().toLowerCase() === "true";
  }
  if (kind === "number") {
    const parsed = Number(`${blockValue}`.trim());
    return Number.isFinite(parsed) ? parsed : (field.default ?? 0);
  }
  if (kind === "tuple") {
    const stripped = `${blockValue}`.replace(/^\[/, "").replace(/\]$/, "");
    return stripped.split(",").map((part) => Number(part.trim()) || 0);
  }
  return blockValue;
}

export function writeSchemaFieldValue(paramsRaw: string, field: SchemaField, nextValue: unknown): string {
  const kind = schemaFieldInputKind(field);
  if (kind === "boolean") {
    return setYamlBlockValue(paramsRaw, field.name, nextValue ? "true" : "false");
  }
  if (kind === "number") {
    return setYamlBlockValue(paramsRaw, field.name, `${nextValue}`);
  }
  if (kind === "tuple") {
    const arr = Array.isArray(nextValue) ? nextValue : [0, 0, 0];
    const normalized = [arr[0] ?? 0, arr[1] ?? 0, arr[2] ?? 0].map((v) => Number(v) || 0);
    return setYamlBlockValue(paramsRaw, field.name, `[${normalized.join(", ")}]`);
  }
  return setYamlBlockValue(paramsRaw, field.name, `${nextValue || ""}`);
}

/** Flattens `{target: ModeMetadata[]}` into a lookup keyed by both public mode id and class path. */
export function normalizeModeRegistry(modeRegistry: ModeRegistryByTarget | undefined | null): Record<string, ModeMetadata> {
  const flat: Record<string, ModeMetadata> = {};
  Object.values(modeRegistry || {}).forEach((entries) => {
    (entries || []).forEach((entry) => {
      const modeRef = publicModeId(entry.mode || entry.classPath || "");
      if (modeRef) {
        flat[modeRef] = entry;
      }
      if (entry.classPath) {
        flat[entry.classPath] = entry;
      }
    });
  });
  return flat;
}

export function defaultParamsRawForMode(metadata: ModeMetadata | undefined | null): string {
  if (!metadata?.params?.length) return "";
  const lines: string[] = [];
  metadata.params.forEach((field) => {
    if (field.defaultKind === "missing") return;
    if (field.default === null || field.default === undefined) return;
    if (field.schemaType === "tuple" && Array.isArray(field.default)) {
      lines.push(`${field.name}: [${field.default.join(", ")}]`);
      return;
    }
    if (field.schemaType === "boolean") {
      lines.push(`${field.name}: ${field.default ? "true" : "false"}`);
      return;
    }
    if (field.schemaType === "array" || field.schemaType === "object") {
      return;
    }
    lines.push(`${field.name}: ${field.default}`);
  });
  return lines.join("\n");
}
