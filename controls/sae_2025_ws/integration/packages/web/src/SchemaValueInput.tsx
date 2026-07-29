import type { SchemaFieldInputKind } from "@pennair/integration-core";

/** Renders the right HTML input for a schema field's input kind -- shared by the mission param editor and the fleet vehicle field editor, which each have their own get/set logic around it (raw YAML text vs. a parsed doc). */
export function SchemaValueInput({
  kind,
  choices,
  value,
  onChange,
}: {
  kind: SchemaFieldInputKind;
  choices?: string[];
  value: unknown;
  onChange: (next: unknown) => void;
}) {
  if (kind === "select") {
    return (
      <select value={value === undefined || value === null ? "" : String(value)} onChange={(e) => onChange(e.target.value)}>
        <option value="">--</option>
        {(choices ?? []).map((choice) => (
          <option key={choice} value={choice}>
            {choice}
          </option>
        ))}
      </select>
    );
  }
  if (kind === "boolean") {
    return <input type="checkbox" checked={Boolean(value)} onChange={(e) => onChange(e.target.checked)} />;
  }
  if (kind === "number") {
    return (
      <input
        type="number"
        value={value === undefined || value === null ? "" : String(value)}
        onChange={(e) => onChange(e.target.value)}
      />
    );
  }
  if (kind === "tuple") {
    const arr = Array.isArray(value) ? value : [];
    return (
      <input
        value={arr.join(", ")}
        placeholder="x, y, z"
        onChange={(e) => onChange(e.target.value.split(",").map((part) => Number(part.trim())))}
      />
    );
  }
  const text = typeof value === "string" ? value : value === undefined || value === null ? "" : JSON.stringify(value);
  return <textarea value={text} rows={kind === "block" ? 4 : 1} onChange={(e) => onChange(e.target.value)} />;
}
