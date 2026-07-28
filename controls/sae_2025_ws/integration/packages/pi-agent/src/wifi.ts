import { execFile as execFileCallback } from "node:child_process";
import { promisify } from "node:util";
import type { WifiConnection, WifiNetwork, WifiScanResult, WifiStatus } from "@pennair/integration-core";

const execFileAsync = promisify(execFileCallback);

const HOTSPOT_NAME = process.env.HOTSPOT_NAME ?? "penn-desktop";

export interface ExecResult {
  code: number;
  stdout: string;
  stderr: string;
}

export type CommandRunner = (command: string, args: string[], timeoutMs: number) => Promise<ExecResult>;

/** Real command runner -- the default, unless a test injects a fake one. */
export const runCommand: CommandRunner = async (command, args, timeoutMs) => {
  try {
    const { stdout, stderr } = await execFileAsync(command, args, { timeout: timeoutMs });
    return { code: 0, stdout, stderr };
  } catch (error) {
    const execError = error as {
      code?: number | string;
      stdout?: string;
      stderr?: string;
      message: string;
    };
    // On a spawn failure (command not found, etc.) `code` is a string like
    // "ENOENT" and `stderr` is an empty string (not absent) -- fall back to
    // `.message` for a useful error, and normalize `code` to a number.
    const numericCode = typeof execError.code === "number" ? execError.code : -1;
    return {
      code: numericCode,
      stdout: execError.stdout ?? "",
      stderr: execError.stderr || execError.message,
    };
  }
};

/** Splits on runs of 2+ spaces, matching nmcli's fixed-width column output. */
function splitColumns(line: string): string[] {
  return line.trim().split(/\s{2,}/);
}

async function policyStatus(run: CommandRunner): Promise<Record<string, unknown>> {
  const result = await run(
    "sh",
    [
      "-c",
      "if command -v pennair-wifi-failover >/dev/null 2>&1; then " +
        "sudo -n pennair-wifi-failover --status 2>/dev/null || " +
        "pennair-wifi-failover --status 2>/dev/null; " +
        "fi",
    ],
    15_000,
  );
  const raw = result.stdout.trim();
  if (result.code !== 0 || !raw) {
    return {};
  }
  try {
    const parsed: unknown = JSON.parse(raw);
    return typeof parsed === "object" && parsed !== null ? (parsed as Record<string, unknown>) : {};
  } catch {
    return {};
  }
}

export async function wifiStatus(run: CommandRunner = runCommand): Promise<WifiStatus> {
  const policy = await policyStatus(run);
  const result = await run("nmcli", ["-f", "NAME,TYPE,DEVICE,STATE", "con", "show", "--active"], 15_000);
  if (result.code !== 0) {
    return { success: false, connections: [], error: result.stderr };
  }

  const connections: WifiConnection[] = result.stdout
    .trim()
    .split("\n")
    .slice(1) // header row
    .filter((line) => line.trim())
    .map(splitColumns)
    .filter((parts) => parts.length >= 4)
    .map(([name, type, device, state]) => ({ name, type, device, state }));

  const legacyHotspot = connections.some((c) => c.name === HOTSPOT_NAME);
  const wifiConnection = connections.find((c) => c.type === "802-11-wireless");

  return {
    success: true,
    isHotspot: typeof policy.is_hotspot === "boolean" ? policy.is_hotspot : legacyHotspot,
    currentWifi: (policy.current_wifi as string | undefined) ?? wifiConnection?.name,
    currentMode: policy.current_mode as string | undefined,
    effectiveRole: policy.effective_role as string | undefined,
    connections,
  };
}

export async function wifiScan(run: CommandRunner = runCommand): Promise<WifiScanResult> {
  const result = await run(
    "nmcli",
    ["-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list", "--rescan", "yes"],
    20_000,
  );
  if (result.code !== 0) {
    return { success: false, networks: [], error: result.stderr };
  }

  const strongestBySsid = new Map<string, WifiNetwork>();
  for (const line of result.stdout.trim().split("\n").slice(1)) {
    if (!line.trim()) continue;
    const parts = splitColumns(line);
    if (parts.length < 3) continue;
    const [ssid, signalRaw, security] = parts;
    if (!ssid) continue;
    const signal = Number.parseInt(signalRaw, 10) || 0;
    const existing = strongestBySsid.get(ssid);
    if (!existing || signal > existing.signal) {
      strongestBySsid.set(ssid, { ssid, signal, security: security ?? "" });
    }
  }

  return {
    success: true,
    networks: [...strongestBySsid.values()].sort((a, b) => b.signal - a.signal),
  };
}

export async function wifiConnect(
  ssid: string,
  password: string,
  run: CommandRunner = runCommand,
): Promise<{ success: boolean; output?: string; error?: string }> {
  const policy = await policyStatus(run);
  const hotspotName = (policy.local_ap_profile as string | undefined) ?? HOTSPOT_NAME;

  await run("nmcli", ["con", "down", hotspotName], 15_000);

  const connectArgs = ["dev", "wifi", "connect", ssid];
  if (password) {
    connectArgs.push("password", password);
  }
  const result = await run("nmcli", connectArgs, 30_000);

  if (result.code !== 0) {
    await run("nmcli", ["con", "up", hotspotName], 15_000);
    return { success: false, error: result.stderr || "Failed to connect" };
  }

  return { success: true, output: `Pi connected to ${ssid}` };
}

export async function wifiHotspot(
  run: CommandRunner = runCommand,
): Promise<{ success: boolean; output?: string; error?: string }> {
  const policy = await policyStatus(run);
  const hotspotName = (policy.local_ap_profile as string | undefined) ?? HOTSPOT_NAME;

  await run("nmcli", ["dev", "disconnect", "wlan0"], 15_000);
  const result = await run("nmcli", ["con", "up", hotspotName], 15_000);

  if (result.code !== 0) {
    return { success: false, error: result.stderr };
  }
  return { success: true, output: "Hotspot activated" };
}
