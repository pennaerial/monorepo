import { useState } from "react";
import { Button, Text, TextInput, View } from "react-native";
import type { WifiNetwork, WifiStatus } from "@pennair/integration-core";
import { client } from "./orchestratorClient.js";

export function WifiScreen({ hostname }: { hostname: string }) {
  const [status, setStatus] = useState<WifiStatus | null>(null);
  const [networks, setNetworks] = useState<WifiNetwork[]>([]);
  const [ssid, setSsid] = useState("");
  const [password, setPassword] = useState("");
  const [message, setMessage] = useState("");

  async function refreshStatus() {
    if (!hostname) return;
    setStatus(await client.wifiStatus(hostname));
  }

  async function scan() {
    if (!hostname) return;
    const result = await client.wifiScan(hostname);
    setNetworks(result.networks);
    setMessage(result.error ?? "");
  }

  async function connect() {
    if (!hostname || !ssid) return;
    const result = await client.wifiConnect(hostname, ssid, password);
    setMessage(result.error ?? result.output ?? "");
    await refreshStatus();
  }

  async function activateHotspot() {
    if (!hostname) return;
    const result = await client.wifiHotspot(hostname);
    setMessage(result.error ?? result.output ?? "");
    await refreshStatus();
  }

  return (
    <View>
      <Text>Wifi</Text>

      <Button title="Refresh status" onPress={refreshStatus} disabled={!hostname} />
      <Button title="Scan networks" onPress={scan} disabled={!hostname} />
      <Button title="Activate hotspot" onPress={activateHotspot} disabled={!hostname} />

      {status && (
        <View>
          <Text>Success: {String(status.success)}</Text>
          <Text>Current wifi: {status.currentWifi ?? "(none)"}</Text>
          <Text>Is hotspot: {String(status.isHotspot ?? false)}</Text>
          <Text>Effective role: {status.effectiveRole ?? "(unknown)"}</Text>
          <Text>Travel router locked: {String(status.travelRouterLocked ?? false)}</Text>
          <Text>Allowed AP hosts: {status.allowedApHosts?.join(", ") || "(none)"}</Text>
          {status.error && <Text>Error: {status.error}</Text>}
        </View>
      )}

      {networks.map((n) => (
        <Text key={n.ssid}>
          {n.ssid} ({n.signal}%, {n.security || "open"})
        </Text>
      ))}

      <TextInput placeholder="SSID" value={ssid} onChangeText={setSsid} />
      <TextInput placeholder="Password" secureTextEntry value={password} onChangeText={setPassword} />
      <Button title="Connect" onPress={connect} disabled={!hostname || !ssid} />

      {message ? <Text>{message}</Text> : null}
    </View>
  );
}
