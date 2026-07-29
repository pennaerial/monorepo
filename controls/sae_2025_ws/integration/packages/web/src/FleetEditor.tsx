import { useEffect, useState } from "react";
import {
  normalizeFleetEditorDocument,
  normalizeIntegrationFleetDocument,
  parseFleetEditorDocument,
  renderFleetEditorDocument,
  schemaFieldInputKind,
  uniqueFleetVehicleName,
  writeFleetFieldValue,
  type FleetEditorDocument,
  type FleetSchema,
  type MissionCatalogEntry,
  type SchemaField,
} from "@pennair/integration-core";
import { SchemaValueInput } from "./SchemaValueInput.js";
import { client } from "./orchestratorClient.js";

function FleetFieldInput({
  field,
  value,
  missions,
  onChange,
}: {
  field: SchemaField;
  value: unknown;
  missions: MissionCatalogEntry[];
  onChange: (nextValue: unknown) => void;
}) {
  // "mission" gets a dropdown of real, currently-installed mission names,
  // instead of free text -- the same convenience the old dashboard's
  // FleetFileEditor gave this one field specifically.
  if (field.name === "mission") {
    return (
      <label>
        {field.name}
        <select value={typeof value === "string" ? value : ""} onChange={(e) => onChange(e.target.value)}>
          <option value="">--</option>
          {missions.map((mission) => (
            <option key={mission.name} value={mission.name}>
              {mission.name}
            </option>
          ))}
        </select>
      </label>
    );
  }
  return (
    <label>
      {field.name}
      <SchemaValueInput kind={schemaFieldInputKind(field)} choices={field.choices} value={value} onChange={onChange} />
    </label>
  );
}

export function FleetEditor({ hostname }: { hostname: string }) {
  const [availableFleets, setAvailableFleets] = useState<string[]>([]);
  const [selectedFleet, setSelectedFleet] = useState("");
  const [doc, setDoc] = useState<FleetEditorDocument | null>(null);
  const [fleetSchema, setFleetSchema] = useState<FleetSchema | null>(null);
  const [missions, setMissions] = useState<MissionCatalogEntry[]>([]);
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState("");
  const [savedYaml, setSavedYaml] = useState("");

  useEffect(() => {
    if (!hostname) return;
    client.schema(hostname).then((result) => {
      if (result.success && result.schema) {
        setAvailableFleets(result.schema.availableFleets);
        setFleetSchema(result.schema.fleet);
        setMissions(result.schema.missions);
      }
    });
  }, [hostname]);

  const missionTargetByName = Object.fromEntries(missions.map((mission) => [mission.name, mission.target]));

  async function loadFleet(name: string) {
    if (doc && renderFleetEditorDocument(doc) !== savedYaml) {
      if (!window.confirm("You have unsaved changes to this fleet file. Discard them?")) return;
    }
    setSelectedFleet(name);
    if (!name) {
      setDoc(null);
      setSavedYaml("");
      return;
    }
    setLoading(true);
    const result = await client.readFleetFile(hostname, name);
    setLoading(false);
    if (result.success && result.content !== undefined) {
      const parsed = parseFleetEditorDocument(result.content);
      const normalized = parsed.doc ? normalizeIntegrationFleetDocument(parsed.doc, missionTargetByName) : null;
      setDoc(normalized);
      setSavedYaml(normalized ? renderFleetEditorDocument(normalized) : "");
      setMessage(parsed.error || "");
    } else {
      setDoc(null);
      setSavedYaml("");
      setMessage(result.error ?? "Failed to load fleet file");
    }
  }

  async function save() {
    if (!doc || !selectedFleet) return;
    setSaving(true);
    const normalized = normalizeIntegrationFleetDocument(doc, missionTargetByName);
    const result = await client.writeFleetFile(hostname, selectedFleet, renderFleetEditorDocument(normalized));
    setSaving(false);
    setMessage(result.error ?? "Saved");
    if (!result.error) {
      setDoc(normalized);
      setSavedYaml(renderFleetEditorDocument(normalized));
    }
  }

  function updateVehicleField(index: number, field: SchemaField, nextValue: unknown) {
    if (!doc) return;
    setDoc(writeFleetFieldValue(doc, ["vehicles", index, field.name], field, nextValue));
  }

  function updateBackendField(field: SchemaField, nextValue: unknown) {
    if (!doc) return;
    setDoc(writeFleetFieldValue(doc, ["backend", field.name], field, nextValue));
  }

  function updateDefaultsField(field: SchemaField, nextValue: unknown) {
    if (!doc) return;
    setDoc(writeFleetFieldValue(doc, ["defaults", field.name], field, nextValue));
  }

  function addVehicle() {
    if (!doc) return;
    const name = uniqueFleetVehicleName(doc);
    setDoc(normalizeFleetEditorDocument({ ...doc, vehicles: [...doc.vehicles, { name }] }));
  }

  function removeVehicle(index: number) {
    if (!doc) return;
    setDoc(normalizeFleetEditorDocument({ ...doc, vehicles: doc.vehicles.filter((_, i) => i !== index) }));
  }

  const backendFields = fleetSchema?.sections.find((section) => section.name === "backend")?.fields ?? [];
  const defaultsFields = fleetSchema?.sections.find((section) => section.name === "defaults")?.fields ?? [];
  const vehicleCommonFields = fleetSchema?.sections.find((section) => section.name === "vehicle.common")?.fields ?? [];
  const vehicleHardwareFields = fleetSchema?.sections.find((section) => section.name === "vehicle.hardware")?.fields ?? [];

  return (
    <section>
      <h2>Fleet Editor</h2>

      <label>
        Fleet file
        <select value={selectedFleet} onChange={(e) => loadFleet(e.target.value)} disabled={!hostname}>
          <option value="">-- select --</option>
          {availableFleets.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>

      {loading && <p>Loading...</p>}

      {doc && (
        <>
          {backendFields.length > 0 && (
            <fieldset className="backend-fieldset">
              <legend>Backend</legend>
              {backendFields.map((field) => (
                <FleetFieldInput
                  key={field.name}
                  field={field}
                  value={doc.backend[field.name]}
                  missions={missions}
                  onChange={(nextValue) => updateBackendField(field, nextValue)}
                />
              ))}
            </fieldset>
          )}

          {defaultsFields.length > 0 && (
            <fieldset className="defaults-fieldset">
              <legend>Defaults</legend>
              {defaultsFields.map((field) => (
                <FleetFieldInput
                  key={field.name}
                  field={field}
                  value={doc.defaults[field.name]}
                  missions={missions}
                  onChange={(nextValue) => updateDefaultsField(field, nextValue)}
                />
              ))}
            </fieldset>
          )}

          <div>
            <button onClick={addVehicle}>Add vehicle</button>
          </div>

          {doc.vehicles.map((vehicle, index) => {
            const isUav = vehicle.kind === "uav";
            const hardwareFields = isUav
              ? vehicleHardwareFields
              : vehicleHardwareFields.filter((field) => field.name !== "px4_airframe_id" && field.name !== "px4_namespace");
            return (
              <fieldset key={index} className="vehicle-fieldset">
                <legend>{vehicle.name || `vehicle ${index}`}</legend>
                <button onClick={() => removeVehicle(index)}>Remove</button>
                {[...vehicleCommonFields, ...hardwareFields].map((field) => (
                  <FleetFieldInput
                    key={field.name}
                    field={field}
                    value={vehicle[field.name]}
                    missions={missions}
                    onChange={(nextValue) => updateVehicleField(index, field, nextValue)}
                  />
                ))}
              </fieldset>
            );
          })}

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
