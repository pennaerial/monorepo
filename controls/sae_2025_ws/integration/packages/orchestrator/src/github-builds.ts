export interface BuildListItem {
  source: "release" | "actions";
  tag: string;
  sha: string;
  commitSha?: string;
  commitSubject?: string;
  name: string;
  date: string;
  downloadUrl?: string;
  sizeMb?: number;
  runId?: string;
  artifactId?: string;
  artifactName?: string;
  branch?: string;
  workflowName?: string;
  workflowEvent?: string;
  workflowConclusion?: string;
  deployable: boolean;
  fallback: boolean;
}

export interface ListBuildsOptions {
  githubRepo: string;
  githubToken?: string;
  artifactPage?: number;
  artifactPageSize?: number;
  q?: string;
  sha?: string;
  commitSubject?: string;
  branch?: string;
  includeFallback?: boolean;
  fetchImpl?: typeof fetch;
  now?: () => number;
}

export interface ListBuildsResult {
  success: boolean;
  builds: BuildListItem[];
  releases: BuildListItem[];
  artifacts: BuildListItem[];
  artifactPage: number;
  artifactPageSize: number;
  artifactHasMore: boolean;
  error?: string;
}

// -- Caches, matching the old dashboard's module-level TTL caches (this is a
// single orchestrator process serving one operator, so module-level state is
// the same shape the old backend used, not a multi-tenant concern). --------
const BUILD_LIST_CACHE_TTL_MS = 60_000;
const LOOKUP_CACHE_TTL_MS = 1_800_000; // 30 min

const buildListCache = new Map<string, { at: number; payload: ListBuildsResult }>();
const commitSubjectCache = new Map<string, { at: number; subject: string | null }>();
const workflowRunCache = new Map<string, { at: number; payload: Record<string, unknown> | null }>();

function cacheKey(options: ListBuildsOptions): string {
  return JSON.stringify({
    repo: options.githubRepo,
    page: options.artifactPage ?? 1,
    size: options.artifactPageSize ?? 20,
    q: (options.q ?? "").trim(),
    sha: (options.sha ?? "").trim(),
    commitSubject: (options.commitSubject ?? "").trim(),
    branch: (options.branch ?? "").trim(),
    includeFallback: !!options.includeFallback,
  });
}

function githubHeaders(token?: string): Record<string, string> {
  const headers: Record<string, string> = { Accept: "application/vnd.github+json" };
  if (token) headers.Authorization = `Bearer ${token}`;
  return headers;
}

/**
 * Only 403-with-"rate limit" gets special handling (retry-hint parsing,
 * stale-cache fallback) -- matches the old dashboard exactly: 404/401/5xx/
 * network errors all just surface as a plain generic failure message.
 */
async function githubRateLimitMessage(
  response: Response,
  tokenConfigured: boolean,
  context: "builds" | "commit-lookup" | "cached",
): Promise<string | null> {
  if (response.status !== 403) return null;
  let message = "";
  try {
    const payload = (await response.clone().json()) as { message?: string };
    message = (payload.message ?? "").trim();
  } catch {
    message = (await response.clone().text()).trim();
  }
  if (!message.toLowerCase().includes("rate limit")) return null;

  let retryHint = "";
  const retryAfter = response.headers.get("retry-after");
  if (retryAfter && /^\d+$/.test(retryAfter)) {
    retryHint = ` Retry after about ${retryAfter} seconds.`;
  } else {
    const resetAt = response.headers.get("x-ratelimit-reset");
    if (resetAt && /^\d+$/.test(resetAt)) {
      const resetTime = new Date(Number(resetAt) * 1000).toISOString().replace("T", " ").slice(0, 19);
      retryHint = ` GitHub resets at ${resetTime} UTC.`;
    }
  }
  const authHint = tokenConfigured
    ? ""
    : " Configure a GitHub token for higher limits.";

  if (context === "commit-lookup") {
    return `GitHub commit-title lookups were rate-limited. Showing available builds with fallback titles.${retryHint}${authHint}`;
  }
  if (context === "cached") {
    return `GitHub API rate limit exceeded. Showing a cached build list.${retryHint}${authHint}`;
  }
  return `GitHub API rate limit exceeded while loading deployable builds.${retryHint}${authHint}`;
}

function isDeployableArtifact(artifact: Record<string, unknown>): boolean {
  const name = String(artifact.name ?? "").toLowerCase();
  const workflowRun = (artifact.workflow_run ?? {}) as Record<string, unknown>;
  const workflowName = String(workflowRun.name ?? "").toLowerCase();
  return name.startsWith("ros2-build-") || workflowName === "arm artifact" || workflowName === "arm fallback";
}

function isFallbackArtifact(artifact: Record<string, unknown>): boolean {
  const name = String(artifact.name ?? "").toLowerCase();
  const workflowRun = (artifact.workflow_run ?? {}) as Record<string, unknown>;
  const workflowName = String(workflowRun.name ?? "").toLowerCase();
  return name.includes("fallback") || workflowName.includes("fallback");
}

function releaseBodyValue(release: Record<string, unknown>, label: string): string | null {
  const body = String(release.body ?? "");
  for (const line of body.split("\n")) {
    const prefix = `${label}:`;
    if (line.trim().startsWith(prefix)) {
      return line.trim().slice(prefix.length).trim();
    }
  }
  return null;
}

async function collectReleases(repo: string, headers: Record<string, string>, fetchImpl: typeof fetch) {
  const releases: Record<string, unknown>[] = [];
  for (let page = 1; ; page++) {
    const response = await fetchImpl(
      `https://api.github.com/repos/${repo}/releases?per_page=100&page=${page}`,
      { headers },
    );
    if (!response.ok) {
      throw Object.assign(new Error(`GitHub releases request failed: ${response.status}`), { response });
    }
    const rows = (await response.json()) as Record<string, unknown>[];
    releases.push(...rows);
    if (rows.length < 100) break;
  }
  return releases;
}

// A filtered query has no native GitHub-side filter to page against, so it
// has to walk multiple raw pages looking for matches -- bounded so a plain
// search-box query can't walk a whole artifact history and exhaust the rate
// limit on one request.
const MAX_FILTERED_ARTIFACT_PAGES = 20;

async function fetchArtifactPage(
  repo: string,
  headers: Record<string, string>,
  fetchImpl: typeof fetch,
  artifactPageSize: number,
  page: number,
): Promise<Record<string, unknown>[]> {
  const response = await fetchImpl(
    `https://api.github.com/repos/${repo}/actions/artifacts?per_page=${artifactPageSize}&page=${page}`,
    { headers },
  );
  if (!response.ok) {
    throw Object.assign(new Error(`GitHub artifacts request failed: ${response.status}`), { response });
  }
  const payload = (await response.json()) as { artifacts?: Record<string, unknown>[] };
  return payload.artifacts ?? [];
}

/**
 * Unfiltered: returns exactly the requested page's raw rows (one GitHub
 * request), matching normal page-cursor semantics -- a client asking for page
 * 2 gets page 2, not page 1 concatenated with page 2.
 *
 * Filtered: there's no GitHub-side filter to page against, so this walks
 * pages from the start looking for matches, capped at
 * MAX_FILTERED_ARTIFACT_PAGES raw pages.
 */
async function collectArtifacts(
  repo: string,
  headers: Record<string, string>,
  fetchImpl: typeof fetch,
  artifactPageSize: number,
  artifactPage: number,
  filtersActive: boolean,
): Promise<{ rows: Record<string, unknown>[]; hasMore: boolean }> {
  if (!filtersActive) {
    const rows = await fetchArtifactPage(repo, headers, fetchImpl, artifactPageSize, artifactPage);
    return { rows, hasMore: rows.length >= artifactPageSize };
  }

  const rows: Record<string, unknown>[] = [];
  for (let page = 1; page <= MAX_FILTERED_ARTIFACT_PAGES; page++) {
    const pageRows = await fetchArtifactPage(repo, headers, fetchImpl, artifactPageSize, page);
    rows.push(...pageRows);
    if (pageRows.length < artifactPageSize) return { rows, hasMore: false };
  }
  return { rows, hasMore: true };
}

async function commitSubjectFor(
  repo: string,
  headers: Record<string, string>,
  fetchImpl: typeof fetch,
  sha: string,
  rateLimited: { hit: boolean; message?: string },
  now: () => number,
): Promise<string | null> {
  if (!sha) return null;
  const cached = commitSubjectCache.get(sha);
  if (cached && now() - cached.at < LOOKUP_CACHE_TTL_MS) return cached.subject;
  if (rateLimited.hit) return null;

  try {
    const response = await fetchImpl(`https://api.github.com/repos/${repo}/commits/${sha}`, { headers });
    if (!response.ok) {
      const message = await githubRateLimitMessage(response, !!headers.Authorization, "commit-lookup");
      if (message) {
        rateLimited.hit = true;
        rateLimited.message = message;
      }
      commitSubjectCache.set(sha, { at: now(), subject: null });
      return null;
    }
    const payload = (await response.json()) as { commit?: { message?: string } };
    const subject = (payload.commit?.message ?? "").split("\n")[0]?.trim() || null;
    commitSubjectCache.set(sha, { at: now(), subject });
    return subject;
  } catch {
    commitSubjectCache.set(sha, { at: now(), subject: null });
    return null;
  }
}

async function workflowRunFor(
  repo: string,
  headers: Record<string, string>,
  fetchImpl: typeof fetch,
  runId: string,
  now: () => number,
): Promise<Record<string, unknown> | null> {
  const cached = workflowRunCache.get(runId);
  if (cached && now() - cached.at < LOOKUP_CACHE_TTL_MS) return cached.payload;

  try {
    const response = await fetchImpl(`https://api.github.com/repos/${repo}/actions/runs/${runId}`, { headers });
    const payload = response.ok ? ((await response.json()) as Record<string, unknown>) : null;
    workflowRunCache.set(runId, { at: now(), payload });
    return payload;
  } catch {
    workflowRunCache.set(runId, { at: now(), payload: null });
    return null;
  }
}

function matchesFilters(item: BuildListItem, options: ListBuildsOptions): boolean {
  if (!options.includeFallback && item.fallback) return false;
  const lower = (v: string | undefined) => (v ?? "").toLowerCase();
  if (options.sha && !`${item.sha} ${item.commitSha ?? ""}`.toLowerCase().includes(options.sha.toLowerCase())) {
    return false;
  }
  if (options.commitSubject && !lower(item.commitSubject).includes(options.commitSubject.toLowerCase())) {
    return false;
  }
  if (options.branch && !lower(item.branch).includes(options.branch.toLowerCase())) return false;
  if (options.q) {
    const haystack = [item.sha, item.commitSha, item.commitSubject, item.name, item.tag, item.branch, item.workflowName]
      .map(lower)
      .join(" ");
    if (!haystack.includes(options.q.toLowerCase())) return false;
  }
  return true;
}

export async function listBuilds(options: ListBuildsOptions): Promise<ListBuildsResult> {
  const artifactPage = Math.max(Math.trunc(options.artifactPage ?? 1), 1);
  const artifactPageSize = Math.max(1, Math.min(Math.trunc(options.artifactPageSize ?? 20), 50));
  const fetchImpl = options.fetchImpl ?? fetch;
  const now = options.now ?? (() => Date.now());
  const key = cacheKey({ ...options, artifactPage, artifactPageSize });

  if (!options.githubRepo) {
    return {
      success: false,
      error: "GITHUB_REPO not configured",
      builds: [],
      releases: [],
      artifacts: [],
      artifactPage,
      artifactPageSize,
      artifactHasMore: false,
    };
  }

  const cached = buildListCache.get(key);
  if (cached && now() - cached.at < BUILD_LIST_CACHE_TTL_MS) {
    return cached.payload;
  }

  const filtersActive = !!(options.includeFallback || options.q || options.sha || options.commitSubject || options.branch);
  const headers = githubHeaders(options.githubToken);

  try {
    const [rawReleases, artifactPageResult] = await Promise.all([
      collectReleases(options.githubRepo, headers, fetchImpl),
      collectArtifacts(options.githubRepo, headers, fetchImpl, artifactPageSize, artifactPage, filtersActive),
    ]);
    const { rows: rawArtifacts, hasMore: artifactHasMore } = artifactPageResult;

    const releaseRows = rawReleases.filter((r) => String(r.tag_name ?? "").startsWith("build-"));
    const deployableArtifactRows = rawArtifacts.filter(
      (a) => isDeployableArtifact(a) && (options.includeFallback || !isFallbackArtifact(a)),
    );

    const runIds = [...new Set(deployableArtifactRows.map((a) => (a.workflow_run as { id?: unknown })?.id).filter(Boolean))].map(String);
    const workflowRunLookup = new Map<string, Record<string, unknown>>();
    for (const runId of runIds) {
      const run = await workflowRunFor(options.githubRepo, headers, fetchImpl, runId, now);
      if (run) workflowRunLookup.set(runId, run);
    }

    const commitLookupState: { hit: boolean; message?: string } = { hit: false };
    const shortSha = (sha: string) => (sha.length > 7 ? sha.slice(0, 7) : sha);

    const releaseBuilds: BuildListItem[] = [];
    for (const release of releaseRows) {
      const assets = (release.assets as Record<string, unknown>[]) ?? [];
      const asset = assets[0] ?? {};
      const commitSha = releaseBodyValue(release, "Commit") ?? String(release.tag_name ?? "").replace(/^build-/, "");
      const commitSubject = await commitSubjectFor(options.githubRepo, headers, fetchImpl, commitSha, commitLookupState, now);
      const item: BuildListItem = {
        source: "release",
        tag: String(release.tag_name ?? ""),
        sha: shortSha(commitSha),
        commitSha: commitSha || undefined,
        commitSubject: commitSubject ?? undefined,
        name: commitSubject || String(release.name ?? release.tag_name ?? ""),
        date: releaseBodyValue(release, "Built") ?? String(release.published_at ?? ""),
        downloadUrl: asset.browser_download_url as string | undefined,
        sizeMb: assets.length ? Math.round((Number(asset.size ?? 0) / 1_000_000) * 10) / 10 : undefined,
        branch: releaseBodyValue(release, "Branch") ?? undefined,
        workflowName: "ARM Artifact Release",
        workflowEvent: "release",
        deployable: true,
        fallback: false,
      };
      if (matchesFilters(item, options)) releaseBuilds.push(item);
    }

    const artifactBuilds: BuildListItem[] = [];
    for (const artifact of deployableArtifactRows) {
      const embeddedRun = (artifact.workflow_run ?? {}) as Record<string, unknown>;
      const runId = String(embeddedRun.id ?? "");
      const resolvedRun = workflowRunLookup.get(runId) ?? embeddedRun;
      const commitSha = String(resolvedRun.head_sha ?? embeddedRun.head_sha ?? "");
      const commitSubject = await commitSubjectFor(options.githubRepo, headers, fetchImpl, commitSha, commitLookupState, now);
      const item: BuildListItem = {
        source: "actions",
        tag: `run-${resolvedRun.id ?? artifact.id}`,
        sha: shortSha(commitSha),
        commitSha: commitSha || undefined,
        commitSubject: commitSubject ?? undefined,
        name: commitSubject || String(artifact.name ?? `artifact-${artifact.id}`),
        date: String(artifact.updated_at ?? "").trim(),
        downloadUrl:
          (artifact.archive_download_url as string | undefined) ??
          `https://api.github.com/repos/${options.githubRepo}/actions/artifacts/${artifact.id}/zip`,
        sizeMb: Math.round((Number(artifact.size_in_bytes ?? 0) / 1_000_000) * 10) / 10,
        runId: runId || undefined,
        artifactId: String(artifact.id ?? "") || undefined,
        artifactName: artifact.name as string | undefined,
        branch: resolvedRun.head_branch as string | undefined,
        workflowName: resolvedRun.name as string | undefined,
        workflowEvent: resolvedRun.event as string | undefined,
        workflowConclusion: resolvedRun.conclusion as string | undefined,
        deployable: isDeployableArtifact(artifact),
        fallback: isFallbackArtifact(artifact),
      };
      if (matchesFilters(item, options)) artifactBuilds.push(item);
    }

    const combined = [...releaseBuilds, ...artifactBuilds].sort((a, b) => (a.date < b.date ? 1 : a.date > b.date ? -1 : 0));

    const result: ListBuildsResult = {
      success: true,
      builds: combined,
      releases: releaseBuilds,
      artifacts: artifactBuilds,
      artifactPage: filtersActive ? 1 : artifactPage,
      artifactPageSize,
      artifactHasMore,
      error: commitLookupState.message,
    };
    buildListCache.set(key, { at: now(), payload: result });
    return result;
  } catch (error) {
    const response = (error as { response?: Response }).response;
    const staleCached = buildListCache.get(key);
    const rateLimitMessage = response
      ? await githubRateLimitMessage(response, !!options.githubToken, staleCached ? "cached" : "builds")
      : null;

    if (staleCached && rateLimitMessage) {
      return { ...staleCached.payload, error: rateLimitMessage };
    }
    return {
      success: false,
      error: rateLimitMessage || (error as Error).message,
      builds: [],
      releases: [],
      artifacts: [],
      artifactPage,
      artifactPageSize,
      artifactHasMore: false,
    };
  }
}
