import { useEffect, useRef, useState } from "react";
import type { FleetBoardResult } from "@pennair/integration-core";
import { client } from "./orchestratorClient.js";

const REFRESH_INTERVAL_MS = 5000;

function statusPill(ready: boolean, connected: boolean): string {
  if (!connected) return "Offline";
  return ready ? "Ready" : "Not ready";
}

export function FleetBoard({ onSelectHostname }: { onSelectHostname?: (hostname: string) => void }) {
  const [board, setBoard] = useState<FleetBoardResult | null>(null);
  const [refreshing, setRefreshing] = useState(false);
  // Discards a stale response that resolves after a newer one already
  // landed (requestIdRef) -- but that alone doesn't stop overlapping
  // network calls: a single stuck/degraded Pi can make one fan-out take up
  // to ~21s (discovery + the 20s missionStatus timeout) against a 5s poll
  // interval, so inFlightRef additionally skips starting a new poll while
  // one is still running, rather than piling up several concurrent fetches
  // (and full re-discoveries) all hammering the same struggling Pi.
  const requestIdRef = useRef(0);
  const inFlightRef = useRef(false);

  async function refresh() {
    if (inFlightRef.current) return;
    inFlightRef.current = true;
    const requestId = ++requestIdRef.current;
    setRefreshing(true);
    try {
      const result = await client.fleetBoard();
      if (requestId === requestIdRef.current) {
        setBoard(result);
      }
    } finally {
      inFlightRef.current = false;
      if (requestId === requestIdRef.current) {
        setRefreshing(false);
      }
    }
  }

  useEffect(() => {
    refresh();
    const interval = setInterval(refresh, REFRESH_INTERVAL_MS);
    return () => clearInterval(interval);
  }, []);

  return (
    <section>
      <h2>Fleet Board</h2>
      <button onClick={refresh} disabled={refreshing}>
        Refresh
      </button>

      {board?.error && <p className="error">{board.error}</p>}

      {board?.summary && (
        <p>
          {board.summary.readyDevices} / {board.summary.totalDevices} ready ({board.summary.connectedDevices}{" "}
          connected, {board.summary.runningDevices} running)
        </p>
      )}

      <ul className="fleet-board-grid">
        {board?.devices.map((device) => (
          <li key={device.hostname} className="fleet-board-card">
            <button onClick={() => onSelectHostname?.(device.hostname)}>{device.hostname}</button>
            <span className={`pill ${device.ready ? "pill-ready" : "pill-not-ready"}`}>
              {statusPill(device.ready, device.connected)}
            </span>
            <div>Runtime: {device.runtimeState}</div>
            {device.buildInfo && <div>Build: {device.buildInfo}</div>}
            {device.notes.length > 0 && (
              <ul className="fleet-board-notes">
                {device.notes.map((note) => (
                  <li key={note}>{note}</li>
                ))}
              </ul>
            )}
          </li>
        ))}
      </ul>

      {board && board.devices.length === 0 && !board.error && <p>No Pis discovered.</p>}
    </section>
  );
}
