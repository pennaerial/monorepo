import { readFile, readdir, writeFile } from "node:fs/promises";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists, resolveSymlinkTarget } from "./deploy-fs.js";

export interface FileResult {
  success: boolean;
  content?: string;
  error?: string;
}

export interface MissionNamesResult {
  success: boolean;
  missions: string[];
  error?: string;
}

const MISSION_PACKAGES = ["uav", "payload"] as const;

/** Strict allowlist, not a blacklist -- names come straight from client-controlled input, this must not permit any path traversal. */
function sanitizeFileName(name: string): string | null {
  return /^[A-Za-z0-9_-]+$/.test(name) ? name : null;
}

async function currentReleaseRoot(paths: DeployPaths): Promise<string | null> {
  const target = await resolveSymlinkTarget(paths.currentLink);
  if (!target || !(await pathExists(target))) return null;
  return target;
}

async function findMissionFilePath(root: string, name: string): Promise<string | null> {
  for (const pkg of MISSION_PACKAGES) {
    for (const ext of [".yaml", ".yml"]) {
      const candidate = `${root}/install/${pkg}/share/${pkg}/missions/${name}${ext}`;
      if (await pathExists(candidate)) return candidate;
    }
  }
  return null;
}

async function findFleetFilePath(root: string, name: string): Promise<string | null> {
  for (const ext of [".yaml", ".yml"]) {
    const candidate = `${root}/install/uav/share/uav/fleets/${name}${ext}`;
    if (await pathExists(candidate)) return candidate;
  }
  return null;
}

/** Lists mission YAML files installed as package share data under the currently-deployed release (mirrors uav/payload setup.py's data_files). */
export async function missionNames(paths: DeployPaths = deployPaths()): Promise<MissionNamesResult> {
  const root = await currentReleaseRoot(paths);
  if (!root) return { success: false, missions: [], error: "No build deployed" };

  const missions: string[] = [];
  for (const pkg of MISSION_PACKAGES) {
    const dir = `${root}/install/${pkg}/share/${pkg}/missions`;
    const entries = await readdir(dir).catch(() => [] as string[]);
    for (const entry of entries) {
      if (entry.endsWith(".yaml") || entry.endsWith(".yml")) {
        missions.push(entry.replace(/\.ya?ml$/, ""));
      }
    }
  }
  return { success: true, missions: missions.sort() };
}

export async function readMissionFile(name: string, paths: DeployPaths = deployPaths()): Promise<FileResult> {
  const safeName = sanitizeFileName(name);
  if (!safeName) return { success: false, error: "Invalid mission name" };
  const root = await currentReleaseRoot(paths);
  if (!root) return { success: false, error: "No build deployed" };
  const filePath = await findMissionFilePath(root, safeName);
  if (!filePath) return { success: false, error: `Mission '${safeName}' not found` };
  const content = await readFile(filePath, "utf-8");
  return { success: true, content };
}

/** Writes back to an existing mission file only -- creating a brand new mission isn't supported yet. */
export async function writeMissionFile(
  name: string,
  content: string,
  paths: DeployPaths = deployPaths(),
): Promise<FileResult> {
  const safeName = sanitizeFileName(name);
  if (!safeName) return { success: false, error: "Invalid mission name" };
  const root = await currentReleaseRoot(paths);
  if (!root) return { success: false, error: "No build deployed" };
  const filePath = await findMissionFilePath(root, safeName);
  if (!filePath) return { success: false, error: `Mission '${safeName}' not found` };
  await writeFile(filePath, content, "utf-8");
  return { success: true };
}

export async function readFleetFile(name: string, paths: DeployPaths = deployPaths()): Promise<FileResult> {
  const safeName = sanitizeFileName(name);
  if (!safeName) return { success: false, error: "Invalid fleet file name" };
  const root = await currentReleaseRoot(paths);
  if (!root) return { success: false, error: "No build deployed" };
  const filePath = await findFleetFilePath(root, safeName);
  if (!filePath) return { success: false, error: `Fleet file '${safeName}' not found` };
  const content = await readFile(filePath, "utf-8");
  return { success: true, content };
}

/** Writes back to an existing fleet file only -- creating a brand new fleet file isn't supported yet. */
export async function writeFleetFile(
  name: string,
  content: string,
  paths: DeployPaths = deployPaths(),
): Promise<FileResult> {
  const safeName = sanitizeFileName(name);
  if (!safeName) return { success: false, error: "Invalid fleet file name" };
  const root = await currentReleaseRoot(paths);
  if (!root) return { success: false, error: "No build deployed" };
  const filePath = await findFleetFilePath(root, safeName);
  if (!filePath) return { success: false, error: `Fleet file '${safeName}' not found` };
  await writeFile(filePath, content, "utf-8");
  return { success: true };
}
