import { describe, expect, it } from "vitest";
import { listBuilds } from "./github-builds.js";

function jsonResponse(body: unknown, init?: ResponseInit): Response {
  return new Response(JSON.stringify(body), {
    status: 200,
    headers: { "content-type": "application/json" },
    ...init,
  });
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

/** Same fake-fetch shape as github-builds.test.ts, but tracks how many times
 *  each artifacts page was actually requested, to make the pagination bug
 *  observable. */
function fakeFetchTrackingPages(artifactPages: Record<string, unknown>[][]) {
  const requestedPages: number[] = [];
  const fetchImpl = (async (input: Parameters<typeof fetch>[0]) => {
    const url = String(input);
    if (url.includes("/actions/artifacts")) {
      const page = Number(new URL(url).searchParams.get("page") ?? "1");
      requestedPages.push(page);
      return jsonResponse({ artifacts: artifactPages[page - 1] ?? [] });
    }
    if (url.includes("/releases")) return jsonResponse([]);
    const runMatch = url.match(/\/actions\/runs\/([^/?]+)$/);
    if (runMatch) return jsonResponse({ id: Number(runMatch[1]), name: "ARM Artifact", event: "push", head_branch: "main", head_sha: "irrelevant" });
    const commitMatch = url.match(/\/commits\/([^/?]+)$/);
    if (commitMatch) return jsonResponse({ commit: { message: "irrelevant commit" } });
    throw new Error(`fakeFetch: unhandled url ${url}`);
  }) as typeof fetch;
  return { fetchImpl, requestedPages };
}

/**
 * Regression test for a pagination bug in listBuilds/collectArtifacts
 * (packages/orchestrator/src/github-builds.ts, surfaced through
 * GET /api/deploy/builds in packages/orchestrator/src/index.ts).
 *
 * `collectArtifacts` used to always start at page=1 and accumulate every page
 * from 1 through the requested page into one `rows` array, so a client
 * requesting artifactPage=2 got page 1's items concatenated with page 2's --
 * not just page 2, despite the response shape (artifactPage/artifactPageSize/
 * artifactHasMore) implying single-page-cursor semantics. Fixed: an
 * unfiltered request now fetches exactly the requested page (one GitHub
 * call); a filtered request still has to walk pages looking for matches
 * (there's no GitHub-side filter to page against), bounded by
 * MAX_FILTERED_ARTIFACT_PAGES.
 */
describe("listBuilds artifact pagination", () => {
  it("requesting artifactPage 2 returns only page 2's items, not page 1 concatenated with page 2", async () => {
    const page1 = [artifact({ id: 101, workflow_run: { id: 1001, head_sha: "aaa0001", head_branch: "main" } })];
    const page2 = [artifact({ id: 102, workflow_run: { id: 1002, head_sha: "aaa0002", head_branch: "main" } })];
    const { fetchImpl } = fakeFetchTrackingPages([page1, page2]);

    const result = await listBuilds({
      githubRepo: "acme/repo-pagebug",
      artifactPage: 2,
      artifactPageSize: 1,
      fetchImpl,
    });

    expect(result.success).toBe(true);
    const idsReturned = result.artifacts.map((a) => a.artifactId);
    expect(idsReturned).toEqual(["102"]);
  });

  it("fetching a given page costs exactly one GitHub request for that page, not a re-fetch of every earlier page", async () => {
    const page1 = [artifact({ id: 201, workflow_run: { id: 2001, head_sha: "bbb0001", head_branch: "main" } })];
    const page2 = [artifact({ id: 202, workflow_run: { id: 2002, head_sha: "bbb0002", head_branch: "main" } })];
    const page3 = [artifact({ id: 203, workflow_run: { id: 2003, head_sha: "bbb0003", head_branch: "main" } })];
    const { fetchImpl, requestedPages } = fakeFetchTrackingPages([page1, page2, page3]);

    await listBuilds({ githubRepo: "acme/repo-refetch", artifactPage: 3, artifactPageSize: 1, fetchImpl });

    expect(requestedPages).toEqual([3]);
  });
});
