import { computeFleetReadiness, discoverPis, summarizeFleetBoard, type FleetBoardDevice, type FleetBoardResult } from "@pennair/integration-core";
import { PiAgentClient } from "./pi-agent-client.js";

export interface FetchFleetBoardOptions {
  prefixes: string[];
  suffixes: number[];
  port: number;
  discoveryTimeoutMs: number;
  fetchImpl?: typeof fetch;
  piAgentClientFor?: (hostname: string) => PiAgentClient;
}

/**
 * Mirrors the old dashboard's `fleet.py`'s `get_board()` -- discovers every
 * reachable Pi, then fans out a 3-way probe per Pi (health/mission-status/
 * current-build) in parallel, same shape as the old board's per-device
 * `_summarize_device` (health check + `launch_status` + `current_build`).
 * A single unreachable Pi can't fail the whole board: PiAgentClient's own
 * methods already degrade to a graceful `{success:false, ...}` shape on
 * failure/timeout rather than throwing, so `Promise.all` per device is safe
 * without needing `allSettled` -- matches the old board's "per-row failure
 * doesn't fail the board" behavior for free.
 *
 * Deliberately excludes wifi status (unlike a natural "show everything"
 * urge might suggest): wifi's worst-case relay timeout is up to 35s, and
 * fanning that out across a whole fleet on every board refresh would make
 * one unreachable/slow Pi visibly stall the board -- a bad tradeoff against
 * this project's explicit low-latency priority. Also excludes fleet-YAML
 * vehicle-assignment cross-referencing (the old board's `vehicle_assigned`
 * readiness criterion) -- that needs a parsed fleet-vehicle-preview layer
 * that doesn't exist yet in this rewrite; scoped out of Phase 5, not
 * silently dropped.
 */
export async function fetchFleetBoard(options: FetchFleetBoardOptions): Promise<FleetBoardResult> {
  const discovered = await discoverPis({
    prefixes: options.prefixes,
    suffixes: options.suffixes,
    port: options.port,
    timeoutMs: options.discoveryTimeoutMs,
    fetchImpl: options.fetchImpl,
  });
  // Defense in depth against a duplicate hostname reaching the board (e.g. a
  // misconfigured DISCOVERY_PREFIXES) -- summarizeFleetBoard has no
  // uniqueness safeguard of its own (it just counts whatever it's given), so
  // dedupe at the source, which also avoids probing the same Pi twice.
  const seen = new Set<string>();
  const pis = discovered.filter((pi) => {
    if (seen.has(pi.hostname)) return false;
    seen.add(pi.hostname);
    return true;
  });

  // Forward fetchImpl into the default client too -- a caller supplying
  // only fetchImpl (without also supplying piAgentClientFor) would
  // otherwise get discovery faked but the per-Pi probes silently falling
  // through to the real global fetch.
  const clientFor =
    options.piAgentClientFor ??
    ((hostname: string) => new PiAgentClient({ hostname, port: options.port, fetchImpl: options.fetchImpl }));

  const devices: FleetBoardDevice[] = await Promise.all(
    pis.map(async (pi) => {
      const client = clientFor(pi.hostname);
      const [connected, missionStatus, currentBuild] = await Promise.all([
        client.isHealthy(),
        client.missionStatus(),
        client.currentBuild(),
      ]);

      const readiness = computeFleetReadiness({
        connected,
        buildInstalled: currentBuild.installed,
        runtimeState: missionStatus.state,
      });

      return {
        hostname: pi.hostname,
        connected,
        buildInstalled: currentBuild.installed,
        releaseId: currentBuild.releaseId,
        buildInfo: currentBuild.info,
        runtimeState: missionStatus.state,
        runtimeRunning: missionStatus.running,
        ready: readiness.ready,
        notes: readiness.notes,
      };
    }),
  );

  return { success: true, devices, summary: summarizeFleetBoard(devices) };
}
