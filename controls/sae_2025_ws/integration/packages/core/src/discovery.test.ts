import { describe, expect, it, vi } from "vitest";
import { discoverPis } from "./discovery.js";

describe("discoverPis", () => {
  it("returns only hostnames that respond ok to /health", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      const reachable = url.toString().includes("air-01.local");
      return { ok: reachable } as Response;
    });

    const found = await discoverPis({
      prefixes: ["air"],
      suffixes: [1, 2],
      port: 8090,
      fetchImpl: fetchImpl as unknown as typeof fetch,
    });

    expect(found).toEqual([{ hostname: "air-01.local" }]);
  });

  it("treats a fetch rejection (unreachable host) as not found", async () => {
    const fetchImpl = vi.fn(async () => {
      throw new Error("network error");
    });

    const found = await discoverPis({
      prefixes: ["air"],
      suffixes: [1],
      port: 8090,
      fetchImpl: fetchImpl as unknown as typeof fetch,
    });

    expect(found).toEqual([]);
  });
});
