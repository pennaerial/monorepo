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

/** Real delay -- the default, unless a test injects a no-op one. */
export const delay = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * Parses one line of `nmcli -t` (terse) output: colon-separated fields,
 * where a literal `:` or `\` inside a value is backslash-escaped. Terse mode
 * is nmcli's own recommended machine-readable format -- unlike the
 * fixed-width tabular output, it can't be desynced by an SSID containing
 * spaces or a blank/hidden SSID.
 */
function parseTerseLine(line: string): string[] {
  const fields: string[] = [];
  let current = "";
  for (let i = 0; i < line.length; i++) {
    const ch = line[i];
    if (ch === "\\" && i + 1 < line.length) {
      current += line[i + 1];
      i++;
    } else if (ch === ":") {
      fields.push(current);
      current = "";
    } else {
      current += ch;
    }
  }
  fields.push(current);
  return fields;
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

function stringField(policy: Record<string, unknown>, key: string): string | undefined {
  const value = policy[key];
  return typeof value === "string" && value ? value : undefined;
}

function boolField(policy: Record<string, unknown>, key: string): boolean | undefined {
  const value = policy[key];
  return typeof value === "boolean" ? value : undefined;
}

export async function wifiStatus(run: CommandRunner = runCommand): Promise<WifiStatus> {
  const policy = await policyStatus(run);
  const result = await run("nmcli", ["-t", "-f", "NAME,TYPE,DEVICE,STATE", "con", "show", "--active"], 15_000);
  if (result.code !== 0) {
    return { success: false, connections: [], error: result.stderr };
  }

  const connections: WifiConnection[] = result.stdout
    .trim()
    .split("\n")
    .filter((line) => line.trim())
    .map(parseTerseLine)
    .filter((parts) => parts.length >= 4)
    .map(([name, type, device, state]) => ({ name, type, device, state }));

  const legacyHotspot = connections.some((c) => c.name === HOTSPOT_NAME);
  const wifiConnection = connections.find((c) => c.type === "802-11-wireless");

  return {
    success: true,
    isHotspot: boolField(policy, "is_hotspot") ?? legacyHotspot,
    currentWifi: stringField(policy, "current_wifi") ?? wifiConnection?.name,
    currentMode: stringField(policy, "current_mode"),
    effectiveRole: stringField(policy, "effective_role"),
    policySource: stringField(policy, "policy_source"),
    runtimeActive: boolField(policy, "runtime_active"),
    missionStarted: boolField(policy, "mission_started"),
    travelRouterLocked: boolField(policy, "travel_router_locked"),
    switchingDisabled: boolField(policy, "switching_disabled"),
    localApProfile: stringField(policy, "local_ap_profile"),
    travelRouterProfile: stringField(policy, "travel_router_profile"),
    allowedApHosts: Array.isArray(policy.allowed_ap_hosts) ? (policy.allowed_ap_hosts as string[]) : [],
    connections,
  };
}

export async function wifiScan(run: CommandRunner = runCommand): Promise<WifiScanResult> {
  const result = await run(
    "nmcli",
    ["-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list", "--rescan", "yes"],
    20_000,
  );
  if (result.code !== 0) {
    return { success: false, networks: [], error: result.stderr };
  }

  const strongestBySsid = new Map<string, WifiNetwork>();
  for (const line of result.stdout.trim().split("\n")) {
    if (!line.trim()) continue;
    const parts = parseTerseLine(line);
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
  wait: (ms: number) => Promise<void> = delay,
): Promise<{ success: boolean; output?: string; error?: string }> {
  const policy = await policyStatus(run);
  const hotspotName = stringField(policy, "local_ap_profile") ?? HOTSPOT_NAME;

  await run("nmcli", ["con", "down", hotspotName], 15_000);
  await wait(2000);

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
  wait: (ms: number) => Promise<void> = delay,
): Promise<{ success: boolean; output?: string; error?: string }> {
  const policy = await policyStatus(run);
  const hotspotName = stringField(policy, "local_ap_profile") ?? HOTSPOT_NAME;

  await run("nmcli", ["dev", "disconnect", "wlan0"], 15_000);
  await wait(1000);
  const result = await run("nmcli", ["con", "up", hotspotName], 15_000);

  if (result.code !== 0) {
    return { success: false, error: result.stderr };
  }
  return { success: true, output: "Hotspot activated" };
}
