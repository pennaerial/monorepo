import { useEffect, useRef, useState } from "react";
import { Button, Text, TextInput, View } from "react-native";
import type { LaunchStatus } from "@pennair/integration-core";
import { client } from "./orchestratorClient.js";

export function MissionScreen({ hostname }: { hostname: string }) {
  const [vehicleName, setVehicleName] = useState("");
  const [status, setStatus] = useState<LaunchStatus | null>(null);
  const [message, setMessage] = useState("");
  const [logs, setLogs] = useState("");
  const [streaming, setStreaming] = useState(false);
  // React Native ships a global WebSocket implementation -- same API as web.
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
    <View>
      <Text>Mission</Text>

      <Button title="Refresh status" onPress={refreshStatus} disabled={!hostname} />
      <Button title="Prepare (start process)" onPress={prepare} disabled={!hostname} />
      <Button title="Stop (stop process)" onPress={stop} disabled={!hostname} />
      <Button
        title={streaming ? "Stop log stream" : "Stream logs"}
        onPress={toggleLogStream}
        disabled={!hostname}
      />

      {status && (
        <View>
          <Text>State: {status.state}</Text>
          <Text>Running: {String(status.running)}</Text>
          {status.pid && <Text>PID: {status.pid}</Text>}
          {status.error && <Text>Error: {status.error}</Text>}
        </View>
      )}

      <TextInput placeholder="Vehicle name (e.g. air-01)" value={vehicleName} onChangeText={setVehicleName} />
      <Button title="Start mission" onPress={startMission} disabled={!vehicleName} />
      <Button title="Failsafe" onPress={failsafe} disabled={!vehicleName} />

      {message ? <Text>{message}</Text> : null}
      {logs ? <Text>{logs}</Text> : null}
    </View>
  );
}
