import { mkdir, readdir, readFile } from "node:fs/promises";
import { createWriteStream } from "node:fs";
import { pipeline } from "node:stream/promises";
import { Readable } from "node:stream";
import { join, resolve as resolvePath, sep } from "node:path";
import * as tar from "tar";
import type { BuildSourceRecord } from "./build-source-store.js";

export interface FleetCatalogResult {
  catalog: Record<string, string>;
  source: string | null;
  error: string | null;
}

export interface ResolveFleetCatalogOptions {
  record: BuildSourceRecord;
  cacheRoot: string;
  githubToken?: string;
  fetchImpl?: typeof fetch;
}

/**
 * Fleet-catalog resolution here is intentionally scoped down from the old
 * dashboard's `deploy.py`: only .tar.gz bundles (GitHub release assets,
 * uploaded local artifacts) are extracted -- GitHub Actions artifacts are
 * .zip and aren't supported for catalog resolution (or for pi-agent deploy,
 * which also only extracts .tar.gz), and archive-nested-inside-archive
 * recursion isn't implemented. Both are uncommon paths in practice (day-to-day
 * deploys use release builds); a release build source is what should be used
 * for source bundles containing nested archives.
 */

async function pathExists(path: string): Promise<boolean> {
  try {
    await readFile(path);
    return true;
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code === "EISDIR") return true;
    return false;
  }
}

function isFleetYamlMember(memberName: string): boolean {
  const normalized = memberName.replace(/\\/g, "/").toLowerCase();
  return normalized.includes("/fleets/") && (normalized.endsWith(".yaml") || normalized.endsWith(".yml"));
}

async function walkYamlFiles(root: string): Promise<string[]> {
  const results: string[] = [];
  async function walk(dir: string): Promise<void> {
    let entries;
    try {
      entries = await readdir(dir, { withFileTypes: true });
    } catch {
      return;
    }
    for (const entry of entries) {
      const full = join(dir, entry.name);
      if (entry.isDirectory()) {
        await walk(full);
      } else if (entry.isFile() && (entry.name.endsWith(".yaml") || entry.name.endsWith(".yml"))) {
        results.push(full);
      }
    }
  }
  await walk(root);
  return results;
}

/** Mirrors `_catalog_yaml_paths`: yaml files anywhere under a path segment named "fleets" (case-insensitive). */
export async function catalogYamlPaths(root: string): Promise<string[]> {
  const allYaml = await walkYamlFiles(root);
  return allYaml
    .filter((path) => path.split(sep).some((part) => part.toLowerCase() === "fleets"))
    .map((path) => resolvePath(path))
    .sort();
}

function pathEndsWithParts(path: string, suffixParts: string[]): boolean {
  const parts = path.split(sep).map((p) => p.toLowerCase());
  if (parts.length < suffixParts.length) return false;
  const tail = parts.slice(parts.length - suffixParts.length);
  return tail.every((part, i) => part === suffixParts[i]);
}

/** Mirrors `_fleet_catalog_candidate_key`: prefer the installed share/ copy over src/ over a bare fleets/ dir. */
function fleetCatalogCandidateKey(path: string): [number, number, number, string] {
  const fleetFile = path.split(sep).pop()!.toLowerCase();
  let rank: number;
  if (pathEndsWithParts(path, ["install", "uav", "share", "uav", "fleets", fleetFile])) {
    rank = 0;
  } else if (pathEndsWithParts(path, ["src", "uav", "uav", "fleets", fleetFile])) {
    rank = 1;
  } else if (pathEndsWithParts(path, ["uav", "fleets", fleetFile])) {
    rank = 2;
  } else {
    rank = 3;
  }
  return [rank, path.split(sep).length, path.length, path];
}

function compareCandidateKeys(a: [number, number, number, string], b: [number, number, number, string]): number {
  for (let i = 0; i < a.length; i++) {
    if (a[i] < b[i]) return -1;
    if (a[i] > b[i]) return 1;
  }
  return 0;
}

/** Mirrors `_fleet_catalog_lookup_from_paths`: collapse duplicate copies of the same filename to the best-ranked one. */
export function fleetCatalogLookupFromPaths(paths: string[]): Record<string, string> {
  const candidates = new Map<string, Set<string>>();
  for (const path of paths) {
    const fleetFile = path.split(sep).pop()?.trim();
    if (!fleetFile) continue;
    const resolved = resolvePath(path);
    if (!candidates.has(fleetFile)) candidates.set(fleetFile, new Set());
    candidates.get(fleetFile)!.add(resolved);
  }
  const catalog: Record<string, string> = {};
  for (const [fleetFile, fleetPaths] of candidates) {
    const best = [...fleetPaths].sort((a, b) => compareCandidateKeys(fleetCatalogCandidateKey(a), fleetCatalogCandidateKey(b)))[0];
    catalog[fleetFile] = best;
  }
  return Object.fromEntries(Object.entries(catalog).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0)));
}

async function extractFleetYamlFromTarGz(archivePath: string, extractDir: string): Promise<void> {
  await mkdir(extractDir, { recursive: true });
  await tar.x({
    file: archivePath,
    cwd: extractDir,
    filter: (entryPath) => isFleetYamlMember(entryPath),
  });
}

function catalogCacheKey(record: BuildSourceRecord): string {
  if (record.kind === "github") {
    return `github-${record.githubSource}-${record.tag ?? record.runId ?? record.artifactId ?? "unknown"}`;
  }
  if (record.kind === "local_artifact") {
    return `local-artifact-${record.localArtifactPath ?? "unknown"}`;
  }
  return record.kind;
}

/**
 * Resolves the set of fleet YAML files available from whichever build source
 * is currently selected. Persists extracted archives under `cacheRoot` keyed
 * by source identity, so repeated calls for the same selected build don't
 * re-download/re-extract (a simpler, disk-based analog of the old
 * dashboard's in-memory TTL+stale-fallback cache).
 */
export async function resolveFleetCatalog(options: ResolveFleetCatalogOptions): Promise<FleetCatalogResult> {
  const { record } = options;
  const fetchImpl = options.fetchImpl ?? fetch;

  if (record.kind === "none") {
    return { catalog: {}, source: null, error: null };
  }

  if (record.kind === "local_codebase") {
    const root = join(record.codebaseRoot ?? "", "src", "uav", "uav", "fleets");
    try {
      const paths = await catalogYamlPaths(root);
      return { catalog: fleetCatalogLookupFromPaths(paths), source: root, error: null };
    } catch (error) {
      return { catalog: {}, source: root, error: (error as Error).message };
    }
  }

  if (record.kind === "local_artifact") {
    const artifactPath = record.localArtifactPath ?? "";
    if (!(await pathExists(artifactPath))) {
      return { catalog: {}, source: artifactPath, error: "Selected local artifact no longer exists." };
    }
    const extractDir = join(options.cacheRoot, catalogCacheKey(record), "extract");
    try {
      if (!(await pathExists(extractDir))) {
        await extractFleetYamlFromTarGz(artifactPath, extractDir);
      }
      const paths = await catalogYamlPaths(extractDir);
      return { catalog: fleetCatalogLookupFromPaths(paths), source: artifactPath, error: null };
    } catch (error) {
      return { catalog: {}, source: artifactPath, error: (error as Error).message };
    }
  }

  // record.kind === "github"
  if (record.githubSource === "actions") {
    return {
      catalog: {},
      source: "github",
      error: "GitHub Actions artifacts are .zip and aren't supported for fleet catalog resolution -- select a GitHub release build instead.",
    };
  }
  if (!record.downloadUrl) {
    return { catalog: {}, source: "github", error: "Selected GitHub build source does not expose a download URL." };
  }

  const catalogDir = join(options.cacheRoot, catalogCacheKey(record));
  const archivePath = join(catalogDir, "archive.tar.gz");
  const extractDir = join(catalogDir, "extract");
  try {
    if (!(await pathExists(extractDir))) {
      await mkdir(catalogDir, { recursive: true });
      const headers: Record<string, string> = { Accept: "application/octet-stream" };
      if (options.githubToken) headers.Authorization = `Bearer ${options.githubToken}`;
      await ensureDownloadedFile(record.downloadUrl, archivePath, headers, fetchImpl);
      await extractFleetYamlFromTarGz(archivePath, extractDir);
    }
    const paths = await catalogYamlPaths(extractDir);
    return { catalog: fleetCatalogLookupFromPaths(paths), source: record.downloadUrl, error: null };
  } catch (error) {
    return { catalog: {}, source: record.downloadUrl, error: (error as Error).message };
  }
}

async function ensureDownloadedFile(url: string, destination: string, headers: Record<string, string>, fetchImpl: typeof fetch): Promise<void> {
  const response = await fetchImpl(url, { headers });
  if (!response.ok || !response.body) {
    throw new Error(`Failed to download build archive: ${response.status}`);
  }
  await pipeline(Readable.fromWeb(response.body as never), createWriteStream(destination));
}
