import { useEffect, useState } from "react";
import {
  OrchestratorClient,
  defaultParamsRawForMode,
  effectiveSchemaFieldValue,
  fieldHelpText,
  normalizeModeRegistry,
  parseMissionDocument,
  renderMissionDocument,
  schemaFieldInputKind,
  uniqueModeName,
  updateMissionDocumentMode,
  writeSchemaFieldValue,
  type ModeMetadata,
  type ModeRegistryByTarget,
  type MissionMode,
  type ParsedMissionDocument,
  type SchemaField,
} from "@pennair/integration-core";
import { SchemaValueInput } from "./SchemaValueInput.js";

const ORCHESTRATOR_URL = import.meta.env.VITE_ORCHESTRATOR_URL ?? "http://localhost:8080";
const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });

/** "start" is the mission entrypoint mode name by convention (mirrors the old dashboard's reserved-name rule) -- renaming or removing it would break the mission. */
const RESERVED_MODE_NAME = "start";

function ParamField({
  field,
  paramsRaw,
  onChange,
}: {
  field: SchemaField;
  paramsRaw: string;
  onChange: (nextRaw: string) => void;
}) {
  const kind = schemaFieldInputKind(field);
  const value = effectiveSchemaFieldValue(paramsRaw, field);
  const help = fieldHelpText(field);

  return (
    <div className="param-field">
      <label>
        {field.name}
        <SchemaValueInput kind={kind} choices={field.choices} value={value} onChange={(next) => onChange(writeSchemaFieldValue(paramsRaw, field, next))} />
      </label>
      {help && <p className="help">{help}</p>}
    </div>
  );
}

export function MissionEditor({ hostname }: { hostname: string }) {
  const [missionNames, setMissionNames] = useState<string[]>([]);
  const [selectedMission, setSelectedMission] = useState("");
  const [doc, setDoc] = useState<ParsedMissionDocument | null>(null);
  const [selectedModeName, setSelectedModeName] = useState("");
  const [modeRegistry, setModeRegistry] = useState<ModeRegistryByTarget>({});
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState("");
  const [modeNameError, setModeNameError] = useState("");
  const [savedRawText, setSavedRawText] = useState("");

  useEffect(() => {
    if (!hostname) return;
    client.missionNames(hostname).then((result) => setMissionNames(result.missions));
    client.schema(hostname).then((result) => {
      if (result.success && result.schema) setModeRegistry(result.schema.modes);
    });
  }, [hostname]);

  async function loadMission(name: string) {
    if (doc && doc.rawText !== savedRawText) {
      if (!window.confirm("You have unsaved changes to this mission. Discard them?")) return;
    }
    setSelectedMission(name);
    setSelectedModeName("");
    setModeNameError("");
    if (!name) {
      setDoc(null);
      setSavedRawText("");
      return;
    }
    setLoading(true);
    const result = await client.readMissionFile(hostname, name);
    setLoading(false);
    if (result.success && result.content !== undefined) {
      setDoc(parseMissionDocument(result.content));
      setSavedRawText(result.content);
      setMessage("");
    } else {
      setDoc(null);
      setSavedRawText("");
      setMessage(result.error ?? "Failed to load mission");
    }
  }

  async function save() {
    if (!doc || !selectedMission) return;
    setSaving(true);
    const result = await client.writeMissionFile(hostname, selectedMission, doc.rawText);
    setSaving(false);
    setMessage(result.error ?? "Saved");
    if (!result.error) setSavedRawText(doc.rawText);
  }

  function updateMode(modeName: string, updater: (mode: MissionMode) => MissionMode) {
    setDoc((prev) => (prev ? updateMissionDocumentMode(prev, modeName, updater) : prev));
  }

  function addMode() {
    if (!doc) return;
    const name = uniqueModeName(doc);
    const newMode: MissionMode = {
      name,
      mode: "",
      paramsRaw: "",
      transitionsRaw: "",
      transitions: [],
      target: "",
      hasParams: true,
      hasTransitions: true,
    };
    const next: ParsedMissionDocument = { ...doc, modes: [...doc.modes, newMode] };
    next.rawText = renderMissionDocument(next);
    setDoc(next);
    setSelectedModeName(name);
  }

  function removeMode(modeName: string) {
    if (!doc || modeName === RESERVED_MODE_NAME) return;
    const next: ParsedMissionDocument = { ...doc, modes: doc.modes.filter((mode) => mode.name !== modeName) };
    next.rawText = renderMissionDocument(next);
    setDoc(next);
    if (selectedModeName === modeName) setSelectedModeName("");
  }

  function applyModeDefaults(mode: MissionMode, metadata: ModeMetadata | undefined) {
    if (!metadata) return;
    updateMode(mode.name, (m) => ({ ...m, paramsRaw: defaultParamsRawForMode(metadata) }));
  }

  /**
   * updateMissionDocumentMode matches *every* mode with the given name, so
   * renaming into a name that collides with another mode (or into the
   * reserved "start" entrypoint name) would silently merge two modes into
   * one on the next edit, and removeMode's filter would then remove both at
   * once -- reject the rename instead of applying it.
   */
  function renameSelectedMode(currentName: string, nextName: string) {
    if (!doc) return;
    if (nextName !== currentName) {
      if (nextName === RESERVED_MODE_NAME) {
        setModeNameError(`"${RESERVED_MODE_NAME}" is reserved for the mission entrypoint`);
        return;
      }
      if (doc.modes.some((mode) => mode.name === nextName)) {
        setModeNameError(`A mode named "${nextName}" already exists`);
        return;
      }
    }
    setModeNameError("");
    updateMode(currentName, (mode) => ({ ...mode, name: nextName }));
    setSelectedModeName(nextName);
  }

  const selectedMode = doc?.modes.find((mode) => mode.name === selectedModeName) ?? null;
  const flatRegistry = normalizeModeRegistry(modeRegistry);
  const modeMetadata = selectedMode ? flatRegistry[selectedMode.mode] : undefined;

  return (
    <section>
      <h2>Mission Editor</h2>

      <label>
        Mission
        <select value={selectedMission} onChange={(e) => loadMission(e.target.value)} disabled={!hostname}>
          <option value="">-- select --</option>
          {missionNames.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>

      {loading && <p>Loading...</p>}

      {doc && (
        <>
          {doc.warnings.map((warning) => (
            <p key={warning} className="warning">
              {warning}
            </p>
          ))}

          <div>
            <button onClick={addMode}>Add mode</button>
          </div>

          <ul className="mode-list">
            {doc.modes.map((mode) => (
              <li key={mode.name}>
                <button
                  onClick={() => {
                    setSelectedModeName(mode.name);
                    setModeNameError("");
                  }}
                >
                  {mode.name === selectedModeName ? `> ${mode.name}` : mode.name}
                </button>
                <button onClick={() => removeMode(mode.name)} disabled={mode.name === RESERVED_MODE_NAME}>
                  Remove
                </button>
              </li>
            ))}
          </ul>

          {selectedMode && (
            <div className="mode-editor">
              <label>
                Name
                <input
                  value={selectedMode.name}
                  disabled={selectedMode.name === RESERVED_MODE_NAME}
                  onChange={(e) => renameSelectedMode(selectedMode.name, e.target.value)}
                />
              </label>
              {modeNameError && <p className="error">{modeNameError}</p>}
              <label>
                Mode class
                <input
                  value={selectedMode.mode}
                  onChange={(e) => updateMode(selectedMode.name, (mode) => ({ ...mode, mode: e.target.value }))}
                />
              </label>

              <h3>Params</h3>
              {modeMetadata && (
                <button onClick={() => applyModeDefaults(selectedMode, modeMetadata)}>Reset to defaults</button>
              )}
              {modeMetadata?.params?.length ? (
                modeMetadata.params.map((field) => (
                  <ParamField
                    key={field.name}
                    field={field}
                    paramsRaw={selectedMode.paramsRaw}
                    onChange={(nextRaw) => updateMode(selectedMode.name, (mode) => ({ ...mode, paramsRaw: nextRaw }))}
                  />
                ))
              ) : (
                <textarea
                  value={selectedMode.paramsRaw}
                  rows={6}
                  onChange={(e) => updateMode(selectedMode.name, (mode) => ({ ...mode, paramsRaw: e.target.value }))}
                />
              )}

              <h3>Transitions</h3>
              {selectedMode.transitions.map((transition, index) => (
                <div key={index} className="transition-row">
                  <input
                    placeholder="on"
                    value={transition.key}
                    onChange={(e) => {
                      const next = [...selectedMode.transitions];
                      next[index] = { ...next[index], key: e.target.value };
                      updateMode(selectedMode.name, (mode) => ({ ...mode, transitions: next }));
                    }}
                  />
                  <input
                    placeholder="target mode"
                    value={transition.value}
                    onChange={(e) => {
                      const next = [...selectedMode.transitions];
                      next[index] = { ...next[index], value: e.target.value };
                      updateMode(selectedMode.name, (mode) => ({ ...mode, transitions: next }));
                    }}
                  />
                  <button
                    onClick={() => {
                      const next = selectedMode.transitions.filter((_, i) => i !== index);
                      updateMode(selectedMode.name, (mode) => ({ ...mode, transitions: next, hasTransitions: true }));
                    }}
                  >
                    Remove
                  </button>
                </div>
              ))}
              <button
                onClick={() => {
                  const next = [...selectedMode.transitions, { key: "", value: "" }];
                  updateMode(selectedMode.name, (mode) => ({ ...mode, transitions: next, hasTransitions: true }));
                }}
              >
                Add transition
              </button>
            </div>
          )}

          <div>
            <button onClick={save} disabled={saving}>
              Save
            </button>
          </div>
        </>
      )}

      {message && <p>{message}</p>}
    </section>
  );
}
