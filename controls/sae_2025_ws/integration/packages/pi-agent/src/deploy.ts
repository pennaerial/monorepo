import { randomBytes } from "node:crypto";
import { mkdir, readdir, readFile, rm, writeFile, chmod } from "node:fs/promises";
import { basename } from "node:path";
import * as tar from "tar";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists, resolveSymlinkTarget, symlinkForce } from "./deploy-fs.js";
import { delay, runCommand, type CommandRunner, type SimpleResult } from "./exec.js";
import { SERVICE_NAME } from "./service-name.js";

const PI_USER = process.env.PI_USER ?? "penn";

// A single Pi only ever has one pi-agent process managing one deploy root, so
// a plain in-process mutex (not file-based locking) is enough to serialize
// deploys/rollbacks -- without it, concurrent requests race on the
// current/previous symlink swap and on release-directory pruning (each call
// computes its own view of "previous" and "what to prune" independently),
// which can dangle `current` or delete a release another call just deployed.
let deployMutex: Promise<unknown> = Promise.resolve();
function withDeployLock<T>(task: () => Promise<T>): Promise<T> {
  const result = deployMutex.then(task, task);
  deployMutex = result.then(
    () => undefined,
    () => undefined,
  );
  return result;
}

// ---------------------------------------------------------------------------
// Extraction + release retention
// ---------------------------------------------------------------------------

export interface ExtractResult {
  releaseId: string;
  releaseDir: string;
  previousTarget: string;
}

function releaseTimestamp(now: () => number = Date.now): string {
  return new Date(now()).toISOString().replace(/[-:]/g, "").replace(/\.\d+Z$/, "").replace("T", "-");
}

/**
 * Defense in depth: `artifactName` is the client-supplied upload filename,
 * which ends up as part of a filesystem path (the release directory name)
 * that later gets interpolated into shell commands elsewhere (e.g.
 * schema-export.ts, which shell-quotes it properly at the point of use, but
 * a release slug restricted to safe characters at the source removes the
 * hazard from propagating into systemd unit content, JSON metadata, and log
 * output too, not just the one place that was actually exploitable).
 */
function sanitizeReleaseSlug(artifactName: string): string {
  const stripped = artifactName.replace(/\.tar\.gz$/, "").replace(/\.tgz$/, "");
  const safe = stripped.replace(/[^A-Za-z0-9._-]/g, "_");
  return safe || "release";
}

async function extractReleaseUnlocked(
  artifactPath: string,
  artifactName: string,
  paths: DeployPaths,
  now: () => number,
): Promise<ExtractResult> {
  const releaseSlug = sanitizeReleaseSlug(artifactName);
  // A random suffix guarantees a fresh release directory even when two
  // deploys land in the same clock tick with the same artifact filename --
  // without it, they'd collide on releaseId and silently merge into one
  // directory, and `previous` could end up pointing at the same release as
  // `current` (rollback would then "succeed" while doing nothing).
  const releaseId = `${releaseTimestamp(now)}-${releaseSlug}-${randomBytes(3).toString("hex")}`;
  const releaseDir = `${paths.releasesDir}/${releaseId}`;

  await mkdir(releaseDir, { recursive: true });
  try {
    await tar.x({ file: artifactPath, cwd: releaseDir });
  } catch (error) {
    await rm(releaseDir, { recursive: true, force: true });
    await rm(artifactPath, { force: true });
    throw error;
  }
  await rm(artifactPath, { force: true });

  if (!(await pathExists(`${releaseDir}/install`))) {
    await rm(releaseDir, { recursive: true, force: true });
    throw new Error(`Extracted release is missing an install/ directory: ${releaseDir}/install`);
  }

  const previousTarget = await resolveSymlinkTarget(paths.currentLink);

  await symlinkForce(releaseDir, paths.currentLink);
  if (previousTarget && (await pathExists(previousTarget))) {
    await symlinkForce(previousTarget, paths.previousLink);
  } else {
    await rm(paths.previousLink, { force: true });
  }

  const entries = await readdir(paths.releasesDir, { withFileTypes: true }).catch(() => []);
  for (const entry of entries) {
    if (!entry.isDirectory()) continue;
    const candidate = `${paths.releasesDir}/${entry.name}`;
    if (candidate !== releaseDir && candidate !== previousTarget) {
      await rm(candidate, { recursive: true, force: true });
    }
  }

  return { releaseId, releaseDir, previousTarget };
}

/**
 * Extracts an uploaded artifact tarball into a new timestamped release
 * directory, atomically swaps the `current` symlink to point at it (keeping
 * the previous target as `previous` for rollback), and prunes any older
 * release directories. Mirrors the old dashboard's `_extract_release_on_pi`,
 * just as local fs/child_process operations instead of an SSH bash heredoc.
 * Serialized against concurrent deploys/rollbacks via `withDeployLock`.
 */
export function extractRelease(
  artifactPath: string,
  artifactName: string,
  paths: DeployPaths = deployPaths(),
  now: () => number = Date.now,
): Promise<ExtractResult> {
  return withDeployLock(() => extractReleaseUnlocked(artifactPath, artifactName, paths, now));
}

// ---------------------------------------------------------------------------
// Systemd unit + runner script
// ---------------------------------------------------------------------------

function runnerScriptText(paths: DeployPaths): string {
  return `#!/usr/bin/env bash
set -euo pipefail
export DEPLOY_ROOT=${paths.deployRoot}
cd "$DEPLOY_ROOT"
mkdir -p "$DEPLOY_ROOT/state/logs"
export ROS_LOG_DIR="$DEPLOY_ROOT/state/logs"
set +u
source /opt/ros/jazzy/setup.bash
source "$DEPLOY_ROOT/current/install/setup.bash"
set -u
exec ros2 launch uav fleet.launch.py fleet_file:="$DEPLOY_ROOT/config/runtime_fleet.yaml"
`;
}

// `provision-pi.sh`/`bootstrap-pi.sh` installs the same-named unit at
// provisioning time with a pigpiod ordering dependency and
// RUNTIME_FLEET_CONFIG/PENNAIR_MISSION_STARTED_MARKER_PATH env vars (see
// `controls/sae_2025_ws/scripts/hardware/bootstrap-pi.sh`'s
// `install_systemd_unit`, and `mode_manager.py`'s marker-path lookup, which
// falls back to a throwaway /tmp path without that env var). Every deploy
// re-installs this same unit, so it must keep matching that shape -- setting
// these at the *unit* level (not just in the runner script) also means
// they're present in the process environment from the very start, so they
// reach the exec'd `ros2 launch` process even though the runner script's own
// internal shell variables mostly aren't exported.
function systemdUnitText(paths: DeployPaths): string {
  return `[Unit]
Description=PennAiR autonomy runtime
After=network-online.target pigpiod.service
Wants=network-online.target pigpiod.service

[Service]
Type=simple
User=${PI_USER}
WorkingDirectory=${paths.deployRoot}
Environment=DEPLOY_ROOT=${paths.deployRoot}
Environment=RUNTIME_FLEET_CONFIG=${paths.runtimeFleet}
Environment=PENNAIR_MISSION_STARTED_MARKER_PATH=${paths.deployRoot}/state/mission-started
ExecStart=${paths.runnerScript}
Restart=on-failure
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
`;
}

/**
 * Writes the runner script (owned by the deploy user, no sudo needed) and
 * installs the systemd unit (needs sudo, same as the passwordless-sudo rules
 * `provision-pi.sh` already sets up for `systemctl`/`install`).
 */
export async function ensureRuntimeService(
  run: CommandRunner = runCommand,
  paths: DeployPaths = deployPaths(),
): Promise<SimpleResult> {
  await mkdir(paths.configDir, { recursive: true });
  await writeFile(paths.runnerScript, runnerScriptText(paths), "utf-8");
  await chmod(paths.runnerScript, 0o755);

  const tmpUnitPath = `${paths.incomingDir}/${SERVICE_NAME}.tmp`;
  await mkdir(paths.incomingDir, { recursive: true });
  await writeFile(tmpUnitPath, systemdUnitText(paths), "utf-8");

  const install = await run(
    "sudo",
    ["-n", "install", "-D", "-m", "644", tmpUnitPath, `/etc/systemd/system/${SERVICE_NAME}`],
    15_000,
  );
  await rm(tmpUnitPath, { force: true });
  if (install.code !== 0) {
    return {
      success: false,
      error:
        install.stderr ||
        "Install systemd unit failed. Bootstrap the Pi or grant passwordless sudo for systemctl/install.",
    };
  }

  await run("sudo", ["-n", "systemctl", "daemon-reload"], 15_000);
  const enable = await run("sudo", ["-n", "systemctl", "enable", SERVICE_NAME], 15_000);
  if (enable.code !== 0) {
    return { success: false, error: enable.stderr || "Failed to enable runtime service" };
  }
  return { success: true, output: "Runtime service installed" };
}

// ---------------------------------------------------------------------------
// Health-check-and-rollback activation
// ---------------------------------------------------------------------------

interface SystemctlProps {
  activeState: string;
  subState: string;
  mainPid: string;
  restartCount: string;
}

async function readSystemctlProps(run: CommandRunner): Promise<SystemctlProps> {
  const result = await run(
    "sudo",
    [
      "-n",
      "systemctl",
      "show",
      SERVICE_NAME,
      "-p",
      "ActiveState",
      "-p",
      "SubState",
      "-p",
      "MainPID",
      "-p",
      "NRestarts",
      "--value",
    ],
    8_000,
  );
  const [activeState = "", subState = "", mainPid = "", restartCount = ""] = result.stdout.trim().split("\n");
  return { activeState, subState, mainPid, restartCount };
}

async function revertToPrevious(
  paths: DeployPaths,
  releaseDir: string,
  previousTarget: string,
  run: CommandRunner,
  errorMessage: string,
): Promise<SimpleResult> {
  if (previousTarget && (await pathExists(previousTarget))) {
    await symlinkForce(previousTarget, paths.currentLink);
    await symlinkForce(releaseDir, paths.previousLink);
    await run("sudo", ["-n", "systemctl", "restart", SERVICE_NAME], 15_000); // best-effort, ignore outcome
    return { success: false, error: `${errorMessage} Reverted to the previous release.` };
  }
  return { success: false, error: errorMessage };
}

async function activateReleaseUnlocked(
  releaseDir: string,
  previousTarget: string,
  run: CommandRunner,
  wait: (ms: number) => Promise<void>,
  now: () => number,
  paths: DeployPaths,
): Promise<SimpleResult> {
  const restart = await run("sudo", ["-n", "systemctl", "restart", SERVICE_NAME], 15_000);
  if (restart.code !== 0) {
    return revertToPrevious(
      paths,
      releaseDir,
      previousTarget,
      run,
      restart.stderr || "Failed to restart runtime service.",
    );
  }

  const deadline = now() + 8_000;
  let stableMainPid = "";
  let stableRestarts = "";

  while (now() < deadline) {
    const props = await readSystemctlProps(run);
    const healthy =
      props.activeState === "active" && props.subState === "running" && !!props.mainPid && props.mainPid !== "0";

    if (!healthy) {
      await wait(1000);
      continue;
    }

    if (!stableMainPid) {
      stableMainPid = props.mainPid;
      stableRestarts = props.restartCount;
    } else if (props.mainPid !== stableMainPid || props.restartCount !== stableRestarts) {
      return revertToPrevious(paths, releaseDir, previousTarget, run, "Runtime service became unstable after restart.");
    }
    await wait(1000);
  }

  if (!stableMainPid) {
    return revertToPrevious(paths, releaseDir, previousTarget, run, "Runtime service did not become active in time.");
  }

  return { success: true, output: "Release activated" };
}

/**
 * Restarts the runtime service and polls for up to 8 seconds that it's
 * actually stably running (active+running, with a main PID that doesn't
 * change and a restart count that doesn't increment) before declaring
 * success -- an unstable/failed activation automatically rolls back to
 * `previousTarget` if one is available. Mirrors the old dashboard's
 * `_activate_release` bash loop, just as real TS control flow instead of a
 * bash heredoc polling `systemctl show` in a `while` loop. Serialized against
 * concurrent deploys/rollbacks via `withDeployLock`.
 */
export function activateRelease(
  releaseDir: string,
  previousTarget: string,
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
): Promise<SimpleResult> {
  return withDeployLock(() => activateReleaseUnlocked(releaseDir, previousTarget, run, wait, now, paths));
}

async function rollbackReleaseUnlocked(
  run: CommandRunner,
  wait: (ms: number) => Promise<void>,
  now: () => number,
  paths: DeployPaths,
): Promise<SimpleResult> {
  if (!(await pathExists(paths.previousLink))) {
    return { success: false, error: "No previous release to roll back to" };
  }

  const currentTarget = await resolveSymlinkTarget(paths.currentLink);
  const previousTarget = await resolveSymlinkTarget(paths.previousLink);
  if (!previousTarget) {
    return { success: false, error: "No previous release to roll back to" };
  }

  await symlinkForce(previousTarget, paths.currentLink);
  if (currentTarget && (await pathExists(currentTarget))) {
    await symlinkForce(currentTarget, paths.previousLink);
  }

  return activateReleaseUnlocked(previousTarget, currentTarget, run, wait, now, paths);
}

/**
 * A deliberate, operator-triggered "go back to the previous release" action
 * -- distinct from activateRelease's automatic rollback-on-failure. Swaps
 * current/previous symlinks (so a second rollback undoes the first, same as
 * the old dashboard's `rollback_build`), then reactivates through the same
 * health-check path. Serialized against concurrent deploys/rollbacks via
 * `withDeployLock`.
 */
export function rollbackRelease(
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
): Promise<SimpleResult> {
  return withDeployLock(() => rollbackReleaseUnlocked(run, wait, now, paths));
}

// ---------------------------------------------------------------------------
// Release metadata + current-build status
// ---------------------------------------------------------------------------

export interface ReleaseMetadata {
  releaseId: string;
  sourceType: string;
  sourceLabel: string;
  targetId?: string;
  vehicleName?: string;
  fleetFile?: string;
  deployedAtUtc: string;
  packages?: string[];
}

export async function writeReleaseMetadata(releaseDir: string, metadata: ReleaseMetadata): Promise<void> {
  await writeFile(`${releaseDir}/RELEASE_METADATA.json`, JSON.stringify(metadata, null, 2), "utf-8");
}

function summarizeReleaseMetadata(metadata: Partial<ReleaseMetadata>): string {
  const rows: string[] = [];
  if (metadata.releaseId) rows.push(`Release: ${metadata.releaseId}`);
  if (metadata.sourceType) rows.push(`Source: ${metadata.sourceType}`);
  if (metadata.sourceLabel) rows.push(`Source label: ${metadata.sourceLabel}`);
  if (metadata.targetId) rows.push(`Target: ${metadata.targetId}`);
  if (metadata.vehicleName) rows.push(`Vehicle: ${metadata.vehicleName}`);
  if (metadata.deployedAtUtc) rows.push(`Deployed: ${metadata.deployedAtUtc}`);
  if (metadata.packages?.length) rows.push(`Packages: ${metadata.packages.join(", ")}`);
  return rows.join("\n");
}

export interface CurrentBuildStatus {
  success: boolean;
  installed: boolean;
  info: string;
  releaseId?: string;
}

/** "Is a build installed" = the current symlink exists and resolves to a real
 *  directory -- RELEASE_METADATA.json/BUILD_INFO.txt are only used to build a
 *  human-readable `info` string, matching the old dashboard's `current_build`. */
export async function currentBuild(paths: DeployPaths = deployPaths()): Promise<CurrentBuildStatus> {
  const realTarget = await resolveSymlinkTarget(paths.currentLink);
  if (!realTarget || !(await pathExists(realTarget))) {
    return { success: true, installed: false, info: "No build deployed" };
  }
  const releaseId = basename(realTarget);

  let metadataSummary = "";
  try {
    const raw = await readFile(`${realTarget}/RELEASE_METADATA.json`, "utf-8");
    metadataSummary = summarizeReleaseMetadata(JSON.parse(raw) as Partial<ReleaseMetadata>);
  } catch {
    // no metadata file -- fine, just less info
  }

  let buildInfo = "";
  try {
    buildInfo = (await readFile(`${realTarget}/install/BUILD_INFO.txt`, "utf-8")).trim();
  } catch {
    // no build info file -- fine
  }

  const infoParts = [metadataSummary, buildInfo].filter(Boolean);
  const info = infoParts.join("\n\n").trim() || "Build installed";
  return { success: true, installed: true, info, releaseId };
}

// ---------------------------------------------------------------------------
// Top-level orchestration: the full "here's an uploaded artifact, make it
// live" sequence, mirroring the old dashboard's `_deploy_artifact_path`.
// ---------------------------------------------------------------------------

export interface DeployArtifactOptions {
  artifactPath: string;
  artifactName: string;
  sourceType: string;
  sourceLabel: string;
  vehicleName?: string;
  fleetFile?: string;
}

export interface DeployArtifactResult extends SimpleResult {
  releaseId?: string;
}

/**
 * The whole extract -> install service -> write metadata -> activate
 * sequence runs under a single lock acquisition (not one per sub-step), so a
 * concurrent deploy or rollback can't interleave between, say, this call's
 * extraction and its own activation/rollback -- which could otherwise revert
 * a different, concurrently-deployed release out from under it.
 */
export function deployArtifact(
  options: DeployArtifactOptions,
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
): Promise<DeployArtifactResult> {
  return withDeployLock(async () => {
    const { releaseId, releaseDir, previousTarget } = await extractReleaseUnlocked(
      options.artifactPath,
      options.artifactName,
      paths,
      now,
    );

    const serviceResult = await ensureRuntimeService(run, paths);
    if (!serviceResult.success) {
      // Extraction already succeeded at this point -- report which release it
      // was, even though it never got activated, rather than dropping that
      // diagnostic detail just because this particular stage failed.
      return { ...serviceResult, releaseId };
    }

    await writeReleaseMetadata(releaseDir, {
      releaseId,
      sourceType: options.sourceType,
      sourceLabel: options.sourceLabel,
      vehicleName: options.vehicleName,
      fleetFile: options.fleetFile,
      deployedAtUtc: new Date(now()).toISOString(),
    });

    const activation = await activateReleaseUnlocked(releaseDir, previousTarget, run, wait, now, paths);
    return { ...activation, releaseId };
  });
}
