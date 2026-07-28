export interface DiscoveryOptions {
  /** Hostname prefixes to probe, e.g. ["air", "payload"]. */
  prefixes: string[];
  /** Numeric suffixes to try for each prefix, e.g. [1, 2, ..., 20]. */
  suffixes: number[];
  /** Port the pi-agent health endpoint listens on. */
  port: number;
  /** Per-probe timeout in milliseconds. */
  timeoutMs?: number;
  /** Injectable for tests; defaults to global fetch. */
  fetchImpl?: typeof fetch;
}

export interface DiscoveredPi {
  hostname: string;
}

function candidateHostnames(options: DiscoveryOptions): string[] {
  const padded = (n: number) => String(n).padStart(2, "0");
  const hostnames: string[] = [];
  for (const prefix of options.prefixes) {
    for (const suffix of options.suffixes) {
      hostnames.push(`${prefix}-${padded(suffix)}.local`);
    }
  }
  return hostnames;
}

async function probeOne(
  hostname: string,
  port: number,
  timeoutMs: number,
  fetchImpl: typeof fetch,
): Promise<DiscoveredPi | null> {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetchImpl(`http://${hostname}:${port}/health`, {
      signal: controller.signal,
    });
    return response.ok ? { hostname } : null;
  } catch {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

/**
 * Probes a known range of Pi hostnames directly (no mDNS service-browsing,
 * which browsers/React Native can't do) since this fleet's Pis follow a
 * fixed naming convention (air-0x.local, payload-0x.local).
 */
export async function discoverPis(options: DiscoveryOptions): Promise<DiscoveredPi[]> {
  const timeoutMs = options.timeoutMs ?? 1000;
  const fetchImpl = options.fetchImpl ?? fetch;
  const results = await Promise.all(
    candidateHostnames(options).map((hostname) =>
      probeOne(hostname, options.port, timeoutMs, fetchImpl),
    ),
  );
  return results.filter((result): result is DiscoveredPi => result !== null);
}
