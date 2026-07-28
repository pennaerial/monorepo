import { describe, expect, it } from "vitest";
import { listBuilds } from "./github-builds.js";

function jsonResponse(body: unknown, init?: ResponseInit): Response {
  return new Response(JSON.stringify(body), {
    status: 200,
    headers: { "content-type": "application/json" },
    ...init,
  });
}

function release(overrides: Record<string, unknown> = {}) {
  return {
    tag_name: "build-1",
    name: "Build one",
    published_at: "2026-01-01T00:00:00Z",
    body: "",
    assets: [{ browser_download_url: "https://example.com/a.tar.gz", size: 5_000_000 }],
    ...overrides,
  };
}

function artifact(overrides: Record<string, unknown> = {}) {
  return {
    id: 1,
    name: "ros2-build-uav",
    size_in_bytes: 2_000_000,
    updated_at: "2026-01-02T00:00:00Z",
    archive_download_url: "https://example.com/artifact.zip",
    workflow_run: { id: 10, head_sha: "deadbeef", head_branch: "main" },
    ...overrides,
  };
}

/** Builds a fake fetch that dispatches on URL shape, matching the real GitHub REST endpoints this module calls. */
function fakeFetch(routes: {
  releases?: Record<string, unknown>[][];
  artifacts?: Record<string, unknown>[][];
  commits?: Record<string, string | null>;
  runs?: Record<string, Record<string, unknown>>;
  onRequest?: (url: string) => Response | undefined;
}): typeof fetch {
  const releasePages = routes.releases ?? [[]];
  const artifactPages = routes.artifacts ?? [[]];

  return (async (input: Parameters<typeof fetch>[0]) => {
    const url = String(input);
    const override = routes.onRequest?.(url);
    if (override) return override;

    if (url.includes("/releases")) {
      const page = Number(new URL(url).searchParams.get("page") ?? "1");
      return jsonResponse(releasePages[page - 1] ?? []);
    }
    if (url.includes("/actions/artifacts")) {
      const page = Number(new URL(url).searchParams.get("page") ?? "1");
      return jsonResponse({ artifacts: artifactPages[page - 1] ?? [] });
    }
    const commitMatch = url.match(/\/commits\/([^/?]+)$/);
    if (commitMatch) {
      const sha = commitMatch[1];
      const subject = routes.commits?.[sha];
      if (subject === undefined) return jsonResponse({}, { status: 404 });
      return jsonResponse({ commit: { message: `${subject}\n\nmore detail` } });
    }
    const runMatch = url.match(/\/actions\/runs\/([^/?]+)$/);
    if (runMatch) {
      const run = routes.runs?.[runMatch[1]];
      if (!run) return jsonResponse({}, { status: 404 });
      return jsonResponse(run);
    }
    throw new Error(`fakeFetch: unhandled url ${url}`);
  }) as typeof fetch;
}

describe("listBuilds", () => {
  it("reports an error when no repo is configured", async () => {
    const result = await listBuilds({ githubRepo: "", fetchImpl: fakeFetch({}) });
    expect(result.success).toBe(false);
    expect(result.error).toMatch(/not configured/);
  });

  it("only includes releases whose tag starts with build-", async () => {
    const result = await listBuilds({
      githubRepo: "acme/repo-tagfilter",
      fetchImpl: fakeFetch({
        releases: [[release({ tag_name: "build-9", name: "Nine" }), release({ tag_name: "v1.0.0", name: "Not a build" })]],
        commits: { "9": "commit nine" },
      }),
    });
    expect(result.success).toBe(true);
    expect(result.releases).toHaveLength(1);
    expect(result.releases[0].tag).toBe("build-9");
  });

  it("filters actions artifacts to deployable ones and excludes fallback by default", async () => {
    const result = await listBuilds({
      githubRepo: "acme/repo-artifacts",
      fetchImpl: fakeFetch({
        artifacts: [
          [
            artifact({ id: 101, name: "ros2-build-uav", workflow_run: { id: 201, head_sha: "aaa1111", head_branch: "main" } }),
            artifact({ id: 102, name: "ros2-build-fallback-uav", workflow_run: { id: 202, head_sha: "bbb2222", head_branch: "main" } }),
            artifact({ id: 103, name: "unrelated-artifact", workflow_run: { id: 203, head_sha: "ccc3333", head_branch: "main" } }),
          ],
        ],
        runs: {
          "201": { id: 201, name: "ARM Artifact", event: "push", conclusion: "success", head_branch: "main", head_sha: "aaa1111" },
          "202": { id: 202, name: "ARM Fallback", event: "push", conclusion: "success", head_branch: "main", head_sha: "bbb2222" },
        },
        commits: { aaa1111: "good commit", bbb2222: "fallback commit" },
      }),
    });
    expect(result.success).toBe(true);
    expect(result.artifacts.map((a) => a.artifactId)).toEqual(["101"]);
    expect(result.artifacts[0].fallback).toBe(false);
  });

  it("includes fallback artifacts when includeFallback is set", async () => {
    const result = await listBuilds({
      githubRepo: "acme/repo-includefallback",
      includeFallback: true,
      fetchImpl: fakeFetch({
        artifacts: [
          [artifact({ id: 201, name: "ros2-build-fallback-uav", workflow_run: { id: 301, head_sha: "fff9999", head_branch: "main" } })],
        ],
        runs: { "301": { id: 301, name: "ARM Fallback", event: "push", head_branch: "main", head_sha: "fff9999" } },
        commits: { fff9999: "fallback build" },
      }),
    });
    expect(result.success).toBe(true);
    expect(result.artifacts).toHaveLength(1);
    expect(result.artifacts[0].fallback).toBe(true);
  });

  it("paginates releases across multiple pages of 100", async () => {
    const fullPage = Array.from({ length: 100 }, (_, i) => release({ tag_name: `build-${i}`, name: `Build ${i}` }));
    const secondPage = [release({ tag_name: "build-100", name: "Build 100" })];
    const result = await listBuilds({
      githubRepo: "acme/repo-paginate",
      fetchImpl: fakeFetch({
        releases: [fullPage, secondPage],
        commits: Object.fromEntries(Array.from({ length: 101 }, (_, i) => [`${i}`, `commit ${i}`])),
      }),
    });
    expect(result.success).toBe(true);
    expect(result.releases).toHaveLength(101);
  });

  it("filters combined results by q substring across name/tag/branch", async () => {
    const result = await listBuilds({
      githubRepo: "acme/repo-qfilter",
      q: "special",
      fetchImpl: fakeFetch({
        releases: [[release({ tag_name: "build-special", name: "Special build" }), release({ tag_name: "build-plain", name: "Plain build" })]],
        commits: { special: "special commit", plain: "plain commit" },
      }),
    });
    expect(result.success).toBe(true);
    expect(result.releases).toHaveLength(1);
    expect(result.releases[0].tag).toBe("build-special");
  });

  it("surfaces a rate-limit-specific error message with retry hint on 403", async () => {
    const result = await listBuilds({
      githubRepo: "acme/repo-ratelimited",
      fetchImpl: fakeFetch({
        onRequest: (url) => {
          if (url.includes("/releases")) {
            return new Response(JSON.stringify({ message: "API rate limit exceeded for xxx." }), {
              status: 403,
              headers: { "retry-after": "42" },
            });
          }
          return undefined;
        },
      }),
    });
    expect(result.success).toBe(false);
    expect(result.error).toMatch(/rate limit/i);
    expect(result.error).toMatch(/42 seconds/);
  });

  it("falls back to a stale cached result when the cache has expired and a refresh is rate-limited", async () => {
    let clock = 0;
    const now = () => clock;

    const first = await listBuilds({
      githubRepo: "acme/repo-stalecache",
      now,
      fetchImpl: fakeFetch({
        releases: [[release({ tag_name: "build-stale", name: "Stale build" })]],
        commits: { stale: "stale commit" },
      }),
    });
    expect(first.success).toBe(true);

    clock += 61_000; // past the 60s TTL, forcing a real refresh attempt
    const rateLimited = await listBuilds({
      githubRepo: "acme/repo-stalecache",
      now,
      fetchImpl: fakeFetch({
        onRequest: (url) => {
          if (url.includes("/releases")) {
            return new Response(JSON.stringify({ message: "API rate limit exceeded for xxx." }), { status: 403 });
          }
          return undefined;
        },
      }),
    });
    expect(rateLimited.success).toBe(true);
    expect(rateLimited.error).toMatch(/cached build list/);
    expect(rateLimited.releases[0].tag).toBe("build-stale");
  });
});
