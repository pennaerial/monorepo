import { useEffect, useState } from "react";
import { client } from "./orchestratorClient.js";
import { WifiCard } from "./WifiCard.js";
import { MissionCard } from "./MissionCard.js";
import { MissionEditor } from "./MissionEditor.js";
import { FleetEditor } from "./FleetEditor.js";
import { FleetBoard } from "./FleetBoard.js";

export function App() {
  const [pis, setPis] = useState<string[]>([]);
  const [hostname, setHostname] = useState("");

  useEffect(() => {
    client.discoverPis().then((discovered) => {
      const hostnames = discovered.map((pi) => pi.hostname);
      setPis(hostnames);
      if (hostnames.length > 0 && !hostname) {
        setHostname(hostnames[0]);
      }
    });
    // Intentionally run once on mount; discovery is re-triggered by the button below.
  }, []);

  async function refreshDiscovery() {
    setPis((await client.discoverPis()).map((pi) => pi.hostname));
  }

  return (
    <main>
      <h1>PennAiR Fleet Ops</h1>

      <label>
        Pi hostname
        <input
          list="discovered-pis"
          value={hostname}
          onChange={(e) => setHostname(e.target.value)}
          placeholder="air-01.local"
        />
        <datalist id="discovered-pis">
          {pis.map((h) => (
            <option key={h} value={h} />
          ))}
        </datalist>
      </label>
      <button onClick={refreshDiscovery}>Rediscover</button>

      <FleetBoard onSelectHostname={setHostname} />

      <WifiCard hostname={hostname} />
      <MissionCard hostname={hostname} />
      <MissionEditor hostname={hostname} />
      <FleetEditor hostname={hostname} />
    </main>
  );
}
