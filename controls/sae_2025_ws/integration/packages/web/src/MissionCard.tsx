import { useEffect, useRef, useState } from "react";
import { OrchestratorClient, type LaunchStatus } from "@pennair/integration-core";

const ORCHESTRATOR_URL = import.meta.env.VITE_ORCHESTRATOR_URL ?? "http://localhost:8080";
const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });

export function MissionCard({ hostname }: { hostname: string }) {
  const [vehicleName, setVehicleName] = useState("");
  const [status, setStatus] = useState<LaunchStatus | null>(null);
  const [message, setMessage] = useState("");
  const [logs, setLogs] = useState("");
  const [streaming, setStreaming] = useState(false);
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    return () => {
      socketRef.current?.close();
    };
  }, []);

  async function refreshStatus() {
    if (!hostname) return;
    setStatus(await client.missionStatus(hostname));
  }

  async function prepare() {
    if (!hostname) return;
    const result = await client.prepareMission(hostname);
    setMessage(result.error ?? result.output ?? "");
    await refreshStatus();
  }

  async function stop() {
    if (!hostname) return;
    const result = await client.stopMission(hostname);
    setMessage(result.error ?? result.output ?? "");
    await refreshStatus();
  }

  async function startMission() {
    if (!vehicleName) return;
    const result = await client.startMission(vehicleName);
    setMessage(result.error ?? result.output ?? "");
  }

  async function failsafe() {
    if (!vehicleName) return;
    const result = await client.triggerFailsafe(vehicleName);
    setMessage(result.error ?? result.output ?? "");
  }

  function toggleLogStream() {
    if (socketRef.current) {
      socketRef.current.close();
      socketRef.current = null;
      setStreaming(false);
      return;
    }
    if (!hostname) return;
    const socket = new WebSocket(client.missionLogsUrl(hostname));
    socket.onmessage = (event) => setLogs((prev) => `${prev}${event.data}`);
    socket.onerror = () => setMessage("Log stream error");
    socket.onclose = () => setStreaming(false);
    socketRef.current = socket;
    setStreaming(true);
  }

  return (
    <section>
      <h2>Mission</h2>

      <div>
        <button onClick={refreshStatus} disabled={!hostname}>
          Refresh status
        </button>
        <button onClick={prepare} disabled={!hostname}>
          Prepare (start process)
        </button>
        <button onClick={stop} disabled={!hostname}>
          Stop (stop process)
        </button>
        <button onClick={toggleLogStream} disabled={!hostname}>
          {streaming ? "Stop log stream" : "Stream logs"}
        </button>
      </div>

      {status && (
        <dl>
          <dt>State</dt>
          <dd>{status.state}</dd>
          <dt>Running</dt>
          <dd>{String(status.running)}</dd>
          {status.pid && (
            <>
              <dt>PID</dt>
              <dd>{status.pid}</dd>
            </>
          )}
          {status.error && (
            <>
              <dt>Error</dt>
              <dd>{status.error}</dd>
            </>
          )}
        </dl>
      )}

      <div>
        <input
          placeholder="Vehicle name (e.g. air-01)"
          value={vehicleName}
          onChange={(e) => setVehicleName(e.target.value)}
        />
        <button onClick={startMission} disabled={!vehicleName}>
          Start mission
        </button>
        <button onClick={failsafe} disabled={!vehicleName}>
          Failsafe
        </button>
      </div>

      {message && <p>{message}</p>}
      {logs && <pre>{logs}</pre>}
    </section>
  );
}
