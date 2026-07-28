import { useEffect, useState } from "react";
import { Button, ScrollView, Text, TextInput } from "react-native";
import { SafeAreaProvider, SafeAreaView } from "react-native-safe-area-context";
import { OrchestratorClient } from "@pennair/integration-core";
import { WifiScreen } from "./WifiScreen.js";
import { MissionScreen } from "./MissionScreen.js";

const ORCHESTRATOR_URL = process.env.EXPO_PUBLIC_ORCHESTRATOR_URL ?? "http://localhost:8080";
const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });

function Dashboard() {
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
    <SafeAreaView>
      <ScrollView>
        <Text>PennAiR Fleet Ops</Text>

        <Text>Discovered: {pis.join(", ") || "(none yet)"}</Text>
        <Button title="Rediscover" onPress={refreshDiscovery} />
        <TextInput placeholder="Pi hostname (e.g. air-01.local)" value={hostname} onChangeText={setHostname} />

        <WifiScreen hostname={hostname} />
        <MissionScreen hostname={hostname} />
      </ScrollView>
    </SafeAreaView>
  );
}

export default function App() {
  return (
    <SafeAreaProvider>
      <Dashboard />
    </SafeAreaProvider>
  );
}
