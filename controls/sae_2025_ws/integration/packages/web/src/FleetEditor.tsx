import { useEffect, useState } from "react";
import {
  OrchestratorClient,
  normalizeFleetEditorDocument,
  normalizeIntegrationFleetDocument,
  parseFleetEditorDocument,
  renderFleetEditorDocument,
  schemaFieldInputKind,
  uniqueFleetVehicleName,
  writeFleetFieldValue,
  type FleetEditorDocument,
  type FleetSchema,
  type FleetVehicle,
  type MissionCatalogEntry,
  type SchemaField,
} from "@pennair/integration-core";
import { SchemaValueInput } from "./SchemaValueInput.js";

const ORCHESTRATOR_URL = import.meta.env.VITE_ORCHESTRATOR_URL ?? "http://localhost:8080";
const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });

function VehicleFieldInput({
  field,
  vehicle,
  missions,
  onChange,
}: {
  field: SchemaField;
  vehicle: FleetVehicle;
  missions: MissionCatalogEntry[];
  onChange: (nextValue: unknown) => void;
}) {
  const value = vehicle[field.name];
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
    setSelectedFleet(name);
    if (!name) {
      setDoc(null);
      return;
    }
    setLoading(true);
    const result = await client.readFleetFile(hostname, name);
    setLoading(false);
    if (result.success && result.content !== undefined) {
      const parsed = parseFleetEditorDocument(result.content);
      setDoc(parsed.doc ? normalizeIntegrationFleetDocument(parsed.doc, missionTargetByName) : null);
      setMessage(parsed.error || "");
    } else {
      setDoc(null);
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
    if (!result.error) setDoc(normalized);
  }

  function updateVehicleField(index: number, field: SchemaField, nextValue: unknown) {
    if (!doc) return;
    setDoc(writeFleetFieldValue(doc, ["vehicles", index, field.name], field, nextValue));
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
                  <VehicleFieldInput
                    key={field.name}
                    field={field}
                    vehicle={vehicle}
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
