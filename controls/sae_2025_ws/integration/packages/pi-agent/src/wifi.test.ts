import { describe, expect, it } from "vitest";
import type { CommandRunner, ExecResult } from "./wifi.js";
import { wifiConnect, wifiHotspot, wifiScan, wifiStatus } from "./wifi.js";

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

describe("wifiStatus", () => {
  it("parses active connections and flags the wireless one as current", async () => {
    const run = fakeRunner({
      "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi":
        ok(""),
      "nmcli -f NAME,TYPE,DEVICE,STATE con show --active": ok(
        "NAME        TYPE             DEVICE  STATE\n" +
          "pennair_5G  802-11-wireless  wlan0   activated\n" +
          "lo          loopback         lo      unmanaged\n",
      ),
    });

    const status = await wifiStatus(run);
    expect(status.success).toBe(true);
    expect(status.currentWifi).toBe("pennair_5G");
    expect(status.connections).toHaveLength(2);
    expect(status.isHotspot).toBe(false);
  });

  it("prefers pennair-wifi-failover's policy fields when present", async () => {
    const run = fakeRunner({
      "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi":
        ok(JSON.stringify({ is_hotspot: true, current_wifi: "pennair-ap-air-01", effective_role: "ap" })),
      "nmcli -f NAME,TYPE,DEVICE,STATE con show --active": ok("NAME  TYPE  DEVICE  STATE\n"),
    });

    const status = await wifiStatus(run);
    expect(status.isHotspot).toBe(true);
    expect(status.currentWifi).toBe("pennair-ap-air-01");
    expect(status.effectiveRole).toBe("ap");
  });

  it("returns success:false with the nmcli error on failure", async () => {
    const run = fakeRunner({
      "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi":
        ok(""),
      "nmcli -f NAME,TYPE,DEVICE,STATE con show --active": fail("nmcli: command not found"),
    });

    const status = await wifiStatus(run);
    expect(status.success).toBe(false);
    expect(status.error).toBe("nmcli: command not found");
  });
});

describe("wifiScan", () => {
  it("dedupes repeated SSIDs keeping the strongest signal, sorted descending", async () => {
    const run = fakeRunner({
      "nmcli -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes": ok(
        "SSID        SIGNAL  SECURITY\n" +
          "pennair_5G  40      WPA2\n" +
          "pennair_5G  75      WPA2\n" +
          "other_ap    60      --\n",
      ),
    });

    const scan = await wifiScan(run);
    expect(scan.success).toBe(true);
    expect(scan.networks).toEqual([
      { ssid: "pennair_5G", signal: 75, security: "WPA2" },
      { ssid: "other_ap", signal: 60, security: "--" },
    ]);
  });
});

describe("wifiConnect", () => {
  it("brings the hotspot down, connects, and reports success", async () => {
    const run = fakeRunner({
      "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi":
        ok(""),
      "nmcli con down penn-desktop": ok(""),
      "nmcli dev wifi connect pennair_5G password hunter2": ok(""),
    });

    const result = await wifiConnect("pennair_5G", "hunter2", run);
    expect(result).toEqual({ success: true, output: "Pi connected to pennair_5G" });
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

    const result = await wifiConnect("bad_ssid", "", run);
    expect(result.success).toBe(false);
    expect(result.error).toContain("bad_ssid");
    expect(rolledBack).toBe(true);
  });
});

describe("wifiHotspot", () => {
  it("disconnects wlan0 and brings the hotspot profile up", async () => {
    const run = fakeRunner({
      "sh -c if command -v pennair-wifi-failover >/dev/null 2>&1; then sudo -n pennair-wifi-failover --status 2>/dev/null || pennair-wifi-failover --status 2>/dev/null; fi":
        ok(""),
      "nmcli dev disconnect wlan0": ok(""),
      "nmcli con up penn-desktop": ok(""),
    });

    const result = await wifiHotspot(run);
    expect(result).toEqual({ success: true, output: "Hotspot activated" });
  });
});
