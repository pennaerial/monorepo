import { useState } from "react";
import { OrchestratorClient, type WifiNetwork, type WifiStatus } from "@pennair/integration-core";

const ORCHESTRATOR_URL = import.meta.env.VITE_ORCHESTRATOR_URL ?? "http://localhost:8080";
const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });

export function WifiCard({ hostname }: { hostname: string }) {
  const [status, setStatus] = useState<WifiStatus | null>(null);
  const [networks, setNetworks] = useState<WifiNetwork[]>([]);
  const [ssid, setSsid] = useState("");
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");

  async function refreshStatus() {
    if (!hostname) return;
    setBusy(true);
    setStatus(await client.wifiStatus(hostname));
    setBusy(false);
  }

  async function scan() {
    if (!hostname) return;
    setBusy(true);
    const result = await client.wifiScan(hostname);
    setNetworks(result.networks);
    setMessage(result.error ?? "");
    setBusy(false);
  }

  async function connect() {
    if (!hostname || !ssid) return;
    setBusy(true);
    const result = await client.wifiConnect(hostname, ssid, password);
    setMessage(result.error ?? result.output ?? "");
    setBusy(false);
    await refreshStatus();
  }

  async function activateHotspot() {
    if (!hostname) return;
    setBusy(true);
    const result = await client.wifiHotspot(hostname);
    setMessage(result.error ?? result.output ?? "");
    setBusy(false);
    await refreshStatus();
  }

  return (
    <section>
      <h2>Wifi</h2>

      <div>
        <button onClick={refreshStatus} disabled={busy || !hostname}>
          Refresh status
        </button>
        <button onClick={scan} disabled={busy || !hostname}>
          Scan networks
        </button>
        <button onClick={activateHotspot} disabled={busy || !hostname}>
          Activate hotspot
        </button>
      </div>

      {status && (
        <dl>
          <dt>Success</dt>
          <dd>{String(status.success)}</dd>
          <dt>Current wifi</dt>
          <dd>{status.currentWifi ?? "(none)"}</dd>
          <dt>Is hotspot</dt>
          <dd>{String(status.isHotspot ?? false)}</dd>
          <dt>Effective role</dt>
          <dd>{status.effectiveRole ?? "(unknown)"}</dd>
          <dt>Travel router locked</dt>
          <dd>{String(status.travelRouterLocked ?? false)}</dd>
          <dt>Allowed AP hosts</dt>
          <dd>{status.allowedApHosts?.join(", ") || "(none)"}</dd>
          {status.error && (
            <>
              <dt>Error</dt>
              <dd>{status.error}</dd>
            </>
          )}
        </dl>
      )}

      {networks.length > 0 && (
        <ul>
          {networks.map((n) => (
            <li key={n.ssid}>
              {n.ssid} ({n.signal}%, {n.security || "open"})
            </li>
          ))}
        </ul>
      )}

      <div>
        <input placeholder="SSID" value={ssid} onChange={(e) => setSsid(e.target.value)} />
        <input
          placeholder="Password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
        />
        <button onClick={connect} disabled={busy || !hostname || !ssid}>
          Connect
        </button>
      </div>

      {message && <p>{message}</p>}
    </section>
  );
}
