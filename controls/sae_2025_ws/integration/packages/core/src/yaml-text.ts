/**
 * Line/regex-oriented YAML text helpers -- deliberately not a real YAML
 * parser. Ported 1:1 from the old dashboard's `App.jsx` (functions of the
 * same name), which used this surgical-edit approach specifically to
 * preserve comments/formatting/unparsed content in mission YAML files that a
 * real parse-mutate-serialize round trip would lose.
 */

export type YamlScalarValue = string | boolean | number | number[];

export function escapeRegex(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

export function splitInlineComment(valuePart: string): { value: string; comment: string } {
  const idx = valuePart.indexOf(" #");
  if (idx >= 0) {
    return { value: valuePart.slice(0, idx).trim(), comment: valuePart.slice(idx) };
  }
  return { value: valuePart.trim(), comment: "" };
}

export function unquoteYamlScalar(value: string): string {
  if ((value.startsWith("'") && value.endsWith("'")) || (value.startsWith('"') && value.endsWith('"'))) {
    return value.slice(1, -1);
  }
  return value;
}

export function parseYamlScalar(raw: string, type: string): YamlScalarValue {
  const value = raw.trim();
  if (type === "boolean") {
    return value.toLowerCase() === "true";
  }
  if (type === "integer") {
    const parsed = Number.parseInt(value, 10);
    return Number.isFinite(parsed) ? parsed : 0;
  }
  if (type === "array3") {
    const stripped = value.replace(/^\[/, "").replace(/\]$/, "");
    const parts = stripped
      .split(",")
      .map((p) => p.trim())
      .filter(Boolean);
    if (parts.length !== 3) return [0, 0, 0];
    return parts.map((p) => Number(p) || 0);
  }
  return unquoteYamlScalar(value);
}

export function serializeYamlScalar(value: unknown, type: string): string {
  if (type === "boolean") {
    return value ? "true" : "false";
  }
  if (type === "integer") {
    const parsed = Number.parseInt(`${value ?? 0}`, 10);
    return `${Number.isFinite(parsed) ? parsed : 0}`;
  }
  if (type === "array3") {
    const arr = Array.isArray(value) ? value : [0, 0, 0];
    const normalized = [arr[0], arr[1], arr[2]].map((v) => Number(v) || 0);
    return `[${normalized.join(", ")}]`;
  }

  const text = `${value ?? ""}`;
  if (text === "") return "''";
  if (/[:#\s]/.test(text)) {
    return `'${text.replace(/'/g, "''")}'`;
  }
  return text;
}

/** Regex-based get of a flat top-level `key: value` line anywhere in `content`. */
export function getYamlFieldValue(content: string, key: string, type: string): YamlScalarValue {
  const keyRe = new RegExp(`^\\s*${escapeRegex(key)}\\s*:\\s*(.*)$`, "m");
  const match = content.match(keyRe);
  if (!match) {
    if (type === "boolean") return false;
    if (type === "array3") return [0, 0, 0];
    return "";
  }
  const { value } = splitInlineComment(match[1]);
  return parseYamlScalar(value, type);
}

/** Regex-based set of a flat top-level `key: value` line, appending it if absent. */
export function setYamlFieldValue(content: string, key: string, type: string, value: unknown): string {
  const lines = content.split("\n");
  const keyRe = new RegExp(`^(\\s*${escapeRegex(key)}\\s*:\\s*)(.*)$`);
  const serialized = serializeYamlScalar(value, type);
  let updated = false;

  const nextLines = lines.map((line) => {
    const match = line.match(keyRe);
    if (!match) return line;
    updated = true;
    const prefix = match[1];
    const rest = match[2];
    const { comment } = splitInlineComment(rest);
    return `${prefix}${serialized}${comment}`;
  });

  if (!updated) {
    nextLines.push(`${key}: ${serialized}`);
  }

  return nextLines.join("\n");
}

export function indentBlock(text: string, spaces: number): string {
  const pad = " ".repeat(spaces);
  return text
    .split("\n")
    .map((line) => (line.trim() ? `${pad}${line}` : line))
    .join("\n");
}

export function dedentBlock(text: string, spaces: number): string {
  const pattern = new RegExp(`^ {0,${spaces}}`);
  return text
    .split("\n")
    .map((line) => line.replace(pattern, ""))
    .join("\n")
    .trim();
}

export function parseInlineYamlValue(line: string): string {
  const idx = line.indexOf(":");
  if (idx < 0) return "";
  return line.slice(idx + 1).trim();
}

export function stripYamlComment(value: string): string {
  const idx = value.indexOf(" #");
  return idx >= 0 ? value.slice(0, idx).trim() : value.trim();
}

export function normalizeYamlFragment(content: string | undefined | null): string {
  const text = `${content ?? ""}`;
  const trimmed = text.trim();
  if (!trimmed || trimmed === "{}" || trimmed === "null") return "";
  return text;
}

export interface YamlFragmentEntry {
  key: string;
  value: string;
  start: number;
  end: number;
  lines: string[];
}

/** Splits a raw multi-line YAML fragment into top-level `key: ...` entries with their line spans, without a real parser. */
export function splitTopLevelYamlEntries(content: string | undefined | null): YamlFragmentEntry[] {
  const text = normalizeYamlFragment(content);
  if (!text) return [];
  const lines = text.split("\n");
  const entries: YamlFragmentEntry[] = [];

  for (let index = 0; index < lines.length; index += 1) {
    const line = lines[index];
    const match = line.match(/^([A-Za-z0-9_.-]+)\s*:\s*(.*)$/);
    if (!match) continue;
    const start = index;
    let end = lines.length;
    for (let next = index + 1; next < lines.length; next += 1) {
      if (/^[A-Za-z0-9_.-]+\s*:\s*(.*)$/.test(lines[next])) {
        end = next;
        break;
      }
    }
    entries.push({ key: match[1], value: match[2], start, end, lines: lines.slice(start, end) });
    index = end - 1;
  }

  return entries;
}

export function getYamlBlockValue(content: string | undefined | null, key: string): string {
  const entry = splitTopLevelYamlEntries(content).find((item) => item.key === key);
  if (!entry) return "";

  const inline = stripYamlComment(entry.value || "");
  if (inline) return inline;
  if (entry.lines.length <= 1) return "";
  return dedentBlock(entry.lines.slice(1).join("\n"), 2).trimEnd();
}

export function setYamlBlockValue(content: string | undefined | null, key: string, rawValue: unknown): string {
  // Must stay index-consistent with splitTopLevelYamlEntries's own split (no
  // filtering blank lines out) -- entry.start/end are computed against that
  // unfiltered array, so splicing into a filtered copy desyncs the indices
  // and corrupts/duplicates keys whenever the fragment has a blank interior
  // line (e.g. a multi-line params block with a blank line between items).
  const normalized = normalizeYamlFragment(content);
  const lines = normalized ? normalized.split("\n") : [];
  const entries = splitTopLevelYamlEntries(content);
  const existing = entries.find((item) => item.key === key);
  const nextLines = [...lines];

  if (existing) {
    nextLines.splice(existing.start, existing.end - existing.start);
  }

  const trimmed = `${rawValue ?? ""}`.trim();
  if (trimmed) {
    // A single-item YAML block sequence ("- [[5, 10, -10], 1, LOCAL]", as
    // getYamlBlockValue returns it verbatim, leading "- " included) has no
    // embedded newline but still can't follow "key: " on the same line --
    // "key: - foo" isn't valid YAML (a block-sequence indicator can't share
    // a line with its mapping key). Needs the nested-block form too.
    const needsOwnLine = trimmed.includes("\n") || /^-(\s|$)/.test(trimmed);
    const replacement = needsOwnLine ? [`${key}:`, ...indentBlock(trimmed, 2).split("\n")] : [`${key}: ${trimmed}`];
    const insertAt = existing ? existing.start : nextLines.length;
    nextLines.splice(insertAt, 0, ...replacement);
  }

  return nextLines.join("\n");
}
