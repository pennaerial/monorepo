import { describe, expect, it } from "vitest";
import type { CommandRunner, ExecResult } from "./exec.js";
import { wifiConnect, wifiHotspot, wifiScan, wifiStatus } from "./wifi.js";

const noWait = async () => {};

function ok(stdout: string): ExecResult {
  return { code: 0, stdout, stderr: "" };
}

function fail(stderr: string): ExecResult {
  return { code: 1, stdout: "", stderr };
}

/** Fake CommandRunner keyed by command+args, with no real nmcli/network involved. */
function fakeRunner(responses: Record<string, ExecResult>): CommandRunner {
  return async (command, args) => {
    const key = [command, ...args].join(" ");
    return responses[key] ?? fail(`no fake response registered for: ${key}`);
  };
}

const NO_POLICY_KEY =
  "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi";

describe("wifiStatus", () => {
  it("parses active connections (terse, colon-separated) and flags the wireless one as current", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(""),
      "nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active": ok(
        "pennair_5G:802-11-wireless:wlan0:activated\nlo:loopback:lo:unmanaged\n",
      ),
    });

    const status = await wifiStatus(run);
    expect(status.success).toBe(true);
    expect(status.currentWifi).toBe("pennair_5G");
    expect(status.connections).toHaveLength(2);
    expect(status.isHotspot).toBe(false);
  });

  it("un-escapes backslash-escaped colons within a terse field", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(""),
      "nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active": ok(
        "weird\\:name:802-11-wireless:wlan0:activated\n",
      ),
    });

    const status = await wifiStatus(run);
    expect(status.currentWifi).toBe("weird:name");
  });

  it("prefers pennair-wifi-failover's policy fields when present, including the previously-dropped ones", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(
        JSON.stringify({
          is_hotspot: true,
          current_wifi: "pennair-ap-air-01",
          effective_role: "ap",
          policy_source: "fleet",
          runtime_active: true,
          mission_started: false,
          travel_router_locked: true,
          switching_disabled: false,
          local_ap_profile: "pennair-ap-air-01",
          travel_router_profile: "pennair_5G",
          allowed_ap_hosts: ["air-02"],
        }),
      ),
      "nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active": ok(""),
    });

    const status = await wifiStatus(run);
    expect(status.isHotspot).toBe(true);
    expect(status.currentWifi).toBe("pennair-ap-air-01");
    expect(status.effectiveRole).toBe("ap");
    expect(status.policySource).toBe("fleet");
    expect(status.runtimeActive).toBe(true);
    expect(status.travelRouterLocked).toBe(true);
    expect(status.localApProfile).toBe("pennair-ap-air-01");
    expect(status.travelRouterProfile).toBe("pennair_5G");
    expect(status.allowedApHosts).toEqual(["air-02"]);
  });

  it("does not treat an empty-string policy field as a real value", async () => {
    // A policy JSON with is_hotspot but empty-string current_wifi should
    // still fall back to the nmcli-derived value, not keep "".
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(JSON.stringify({ current_wifi: "" })),
      "nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active": ok(
        "pennair_5G:802-11-wireless:wlan0:activated\n",
      ),
    });

    const status = await wifiStatus(run);
    expect(status.currentWifi).toBe("pennair_5G");
  });

  it("returns success:false with the nmcli error on failure", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(""),
      "nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active": fail("nmcli: command not found"),
    });

    const status = await wifiStatus(run);
    expect(status.success).toBe(false);
    expect(status.error).toBe("nmcli: command not found");
  });
});

describe("wifiScan", () => {
  it("dedupes repeated SSIDs keeping the strongest signal, sorted descending", async () => {
    const run = fakeRunner({
      "nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes": ok(
        "pennair_5G:40:WPA2\npennair_5G:75:WPA2\nother_ap:60:--\n",
      ),
    });

    const scan = await wifiScan(run);
    expect(scan.success).toBe(true);
    expect(scan.networks).toEqual([
      { ssid: "pennair_5G", signal: 75, security: "WPA2" },
      { ssid: "other_ap", signal: 60, security: "--" },
    ]);
  });

  it("handles an SSID containing a colon without desyncing columns", async () => {
    const run = fakeRunner({
      "nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes": ok("weird\\:ssid:50:WPA2\n"),
    });

    const scan = await wifiScan(run);
    expect(scan.networks).toEqual([{ ssid: "weird:ssid", signal: 50, security: "WPA2" }]);
  });
});

describe("wifiConnect", () => {
  it("brings the hotspot down, waits for it to settle, connects, and reports success", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(""),
      "nmcli con down penn-desktop": ok(""),
      "nmcli dev wifi connect pennair_5G password hunter2": ok(""),
    });
    const waited: number[] = [];

    const result = await wifiConnect("pennair_5G", "hunter2", run, async (ms) => {
      waited.push(ms);
    });

    expect(result).toEqual({ success: true, output: "Pi connected to pennair_5G" });
    expect(waited).toEqual([2000]);
  });

  it("rolls back to the hotspot if connecting fails", async () => {
    let rolledBack = false;
    const run: CommandRunner = async (command, args) => {
      const key = [command, ...args].join(" ");
      if (key === "nmcli con up penn-desktop") {
        rolledBack = true;
        return ok("");
      }
      if (key === "nmcli dev wifi connect bad_ssid") {
        return fail("Error: No network with SSID 'bad_ssid' found.");
      }
      return ok("");
    };

    const result = await wifiConnect("bad_ssid", "", run, noWait);
    expect(result.success).toBe(false);
    expect(result.error).toContain("bad_ssid");
    expect(rolledBack).toBe(true);
  });

  it("falls back to the default hotspot name if the policy's local_ap_profile is an empty string", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(JSON.stringify({ local_ap_profile: "" })),
      "nmcli con down penn-desktop": ok(""),
      "nmcli dev wifi connect pennair_5G": ok(""),
    });

    const result = await wifiConnect("pennair_5G", "", run, noWait);
    expect(result.success).toBe(true);
  });
});

describe("wifiHotspot", () => {
  it("disconnects wlan0, waits for it to settle, and brings the hotspot profile up", async () => {
    const run = fakeRunner({
      [NO_POLICY_KEY]: ok(""),
      "nmcli dev disconnect wlan0": ok(""),
      "nmcli con up penn-desktop": ok(""),
    });
    const waited: number[] = [];

    const result = await wifiHotspot(run, async (ms) => {
      waited.push(ms);
    });

    expect(result).toEqual({ success: true, output: "Hotspot activated" });
    expect(waited).toEqual([1000]);
  });
});
