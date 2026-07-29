import { dedentBlock, indentBlock, parseInlineYamlValue, stripYamlComment } from "./yaml-text.js";
import type { MissionMode, ParsedMissionDocument, TransitionEntry } from "./types.js";

// Depth of a mode's params/transitions content: 2 (mode name) + 4 (field key) spaces.
const MODE_FIELD_CONTENT_INDENT = 6;

/**
 * Strips the `uav.modes.` prefix and collapses a repeated trailing segment
 * (`foo.Bar.Bar` -> `foo.Bar`) -- a convention from the old flat
 * `mode_registry.json` class-path format. Mode refs stored in mission YAML
 * still use this shape, so it's still the right normalization for text
 * already on disk, independent of how the live mode registry now shapes
 * itself server-side.
 */
export function publicModeId(modeRef: unknown): string {
  if (typeof modeRef !== "string") return "";
  const trimmed = modeRef.trim();
  const withoutPrefix = trimmed.startsWith("uav.modes.") ? trimmed.slice("uav.modes.".length) : trimmed;
  const parts = withoutPrefix.split(".");
  if (parts.length > 1 && parts.at(-1) === parts.at(-2)) {
    return parts.slice(0, -1).join(".");
  }
  return withoutPrefix;
}

/**
 * Best-effort target-kind inference from a mode id's dotted-path prefix.
 * The live mode registry no longer carries a `mission_target` field (see
 * ModeMetadata's `target`, which the server derives from `RegisteredMode`'s
 * vehicle-class list) -- this string-prefix heuristic is a fallback for
 * mode refs parsed directly out of mission YAML text.
 */
export function inferMissionTarget(modeId: unknown): "uav" | "payload" | "" {
  if (typeof modeId !== "string") return "";
  if (modeId.startsWith("payload.") || modeId.includes(".payload.")) return "payload";
  if (modeId.startsWith("uav.") || modeId.includes(".uav.")) return "uav";
  return "";
}

export function parseTransitionEntries(rawText: string): TransitionEntry[] {
  return rawText
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line && !line.startsWith("#") && line.includes(":"))
    .map((line) => {
      const idx = line.indexOf(":");
      const key = line.slice(0, idx).trim();
      const value = stripYamlComment(line.slice(idx + 1));
      return { key, value };
    })
    .filter((entry) => entry.key);
}

/**
 * Hand-rolled line-scanner for mission YAML (not a real YAML parser, by
 * design) -- preserves comments/formatting in `paramsRaw`/`transitionsRaw`
 * that a real parse-mutate-serialize round trip would lose. Mirrors the old
 * dashboard's `parseMissionDocument` exactly.
 */
export function parseMissionDocument(text: string | undefined | null): ParsedMissionDocument {
  const rawText = `${text ?? ""}`;
  const lines = rawText.split("\n");
  const modesLineIndex = lines.findIndex((line) => /^\s*modes:\s*$/.test(line));

  if (modesLineIndex < 0) {
    return {
      rawText,
      preamble: "",
      modes: [],
      selectedTarget: "",
      warnings: ["Mission YAML does not contain a top-level modes mapping."],
    };
  }

  // Anything before the `modes:` key (header comments, other top-level
  // keys) -- rendered with zero modes falls back to rawText untouched, but
  // once modes exist, render rebuilds the document from `doc.modes` alone,
  // so this has to be captured now and re-prepended on render or it's lost
  // the moment a single field is edited.
  const preamble = lines.slice(0, modesLineIndex).join("\n");

  const modeStarts: { name: string; line: number }[] = [];
  for (let index = modesLineIndex + 1; index < lines.length; index += 1) {
    const line = lines[index];
    const match = line.match(/^ {2}([^:#\s][^:]*)\s*:\s*$/);
    if (match) {
      modeStarts.push({ name: match[1], line: index });
    }
  }

  if (modeStarts.length === 0) {
    return {
      rawText,
      preamble,
      modes: [],
      selectedTarget: "",
      warnings: ["Mission YAML has a modes block but no mode entries."],
    };
  }

  const modes: MissionMode[] = modeStarts.map((modeStart, index) => {
    const nextLine = modeStarts[index + 1]?.line ?? lines.length;
    const blockLines = lines.slice(modeStart.line, nextLine);

    let mode = "";
    let paramsRaw = "";
    let transitionsRaw = "";
    let modeLine = -1;
    let paramsLine = -1;
    let transitionsLine = -1;

    for (let i = 0; i < blockLines.length; i += 1) {
      const line = blockLines[i];
      if (modeLine < 0 && /^\s{4}(?:mode|class):\s*/.test(line)) {
        modeLine = i;
        mode = publicModeId(stripYamlComment(parseInlineYamlValue(line)));
        continue;
      }
      if (paramsLine < 0 && /^\s{4}params:\s*/.test(line)) {
        paramsLine = i;
        const inline = stripYamlComment(parseInlineYamlValue(line));
        const section: string[] = [];
        if (inline) section.push(inline);
        for (let j = i + 1; j < blockLines.length; j += 1) {
          const nested = blockLines[j];
          if (/^\s{4}[A-Za-z0-9_.-]+\s*:\s*/.test(nested)) break;
          section.push(nested);
        }
        paramsRaw = dedentBlock(section.join("\n"), MODE_FIELD_CONTENT_INDENT);
        continue;
      }
      if (transitionsLine < 0 && /^\s{4}transitions:\s*/.test(line)) {
        transitionsLine = i;
        const inline = stripYamlComment(parseInlineYamlValue(line));
        const section: string[] = [];
        if (inline) section.push(inline);
        for (let j = i + 1; j < blockLines.length; j += 1) {
          const nested = blockLines[j];
          if (/^\s{4}[A-Za-z0-9_.-]+\s*:\s*/.test(nested)) break;
          section.push(nested);
        }
        transitionsRaw = dedentBlock(section.join("\n"), MODE_FIELD_CONTENT_INDENT);
      }
    }

    const transitions = parseTransitionEntries(transitionsRaw);
    const target = inferMissionTarget(mode);

    return {
      name: modeStart.name,
      mode,
      paramsRaw,
      transitionsRaw,
      transitions,
      target,
      hasParams: paramsLine >= 0,
      hasTransitions: transitionsLine >= 0,
    };
  });

  const inferredTargets = [...new Set(modes.map((mode) => mode.target).filter(Boolean))];

  return {
    rawText,
    preamble,
    modes,
    selectedTarget: inferredTargets.length === 1 ? inferredTargets[0] : "",
    warnings: inferredTargets.length > 1 ? [`Mission modes mix targets: ${inferredTargets.join(", ")}`] : [],
  };
}

export function renderMissionDocument(doc: ParsedMissionDocument | undefined | null): string {
  if (!doc?.modes?.length) return doc?.rawText || "";

  const lines: string[] = [];
  if (doc.preamble && doc.preamble.trim()) {
    lines.push(doc.preamble, "");
  }
  lines.push("modes:");
  doc.modes.forEach((mode, index) => {
    lines.push(`  ${mode.name}:`);
    lines.push(`    mode: ${mode.mode || ""}`);

    if (mode.paramsRaw && `${mode.paramsRaw}`.trim()) {
      lines.push("    params:");
      lines.push(...indentBlock(mode.paramsRaw.trimEnd(), MODE_FIELD_CONTENT_INDENT).split("\n"));
    } else if (mode.hasParams) {
      lines.push("    params: {}");
    }

    if (mode.transitions?.length) {
      lines.push("    transitions:");
      mode.transitions.forEach((entry) => {
        lines.push(`      ${entry.key}: ${entry.value}`);
      });
    } else if (mode.hasTransitions) {
      lines.push("    transitions: {}");
    }

    if (index < doc.modes.length - 1) {
      lines.push("");
    }
  });

  return lines.join("\n");
}

export function updateMissionDocumentMode(
  doc: ParsedMissionDocument,
  modeName: string,
  updater: (mode: MissionMode) => MissionMode,
): ParsedMissionDocument {
  const next: ParsedMissionDocument = {
    ...doc,
    modes: doc.modes.map((mode) => (mode.name === modeName ? updater({ ...mode }) : mode)),
  };
  next.rawText = renderMissionDocument(next);
  return next;
}

export function uniqueModeName(doc: ParsedMissionDocument, baseName = "mode"): string {
  const names = new Set(doc.modes.map((mode) => mode.name));
  if (!names.has(baseName)) return baseName;
  let index = 1;
  while (names.has(`${baseName}_${index}`)) {
    index += 1;
  }
  return `${baseName}_${index}`;
}
