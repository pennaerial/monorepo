import { mkdir, readdir, readFile, rm, writeFile, chmod } from "node:fs/promises";
import { basename } from "node:path";
import * as tar from "tar";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists, resolveSymlinkTarget, symlinkForce } from "./deploy-fs.js";
import { delay, runCommand, type CommandRunner } from "./exec.js";

const SERVICE_NAME = process.env.MISSION_SERVICE_NAME ?? "pennair-autonomy.service";
const PI_USER = process.env.PI_USER ?? "penn";

interface SimpleResult {
  success: boolean;
  output?: string;
  error?: string;
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
 * Extracts an uploaded artifact tarball into a new timestamped release
 * directory, atomically swaps the `current` symlink to point at it (keeping
 * the previous target as `previous` for rollback), and prunes any older
 * release directories. Mirrors the old dashboard's `_extract_release_on_pi`,
 * just as local fs/child_process operations instead of an SSH bash heredoc.
 */
export async function extractRelease(
  artifactPath: string,
  artifactName: string,
  paths: DeployPaths = deployPaths(),
  now: () => number = Date.now,
): Promise<ExtractResult> {
  const releaseSlug = artifactName.replace(/\.tar\.gz$/, "").replace(/\.tgz$/, "");
  const releaseId = `${releaseTimestamp(now)}-${releaseSlug}`;
  const releaseDir = `${paths.releasesDir}/${releaseId}`;

  await mkdir(releaseDir, { recursive: true });
  await tar.x({ file: artifactPath, cwd: releaseDir });
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

// ---------------------------------------------------------------------------
// Systemd unit + runner script
// ---------------------------------------------------------------------------

function runnerScriptText(paths: DeployPaths): string {
  return `#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT=${paths.deployRoot}
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

function systemdUnitText(paths: DeployPaths): string {
  return `[Unit]
Description=PennAiR autonomy runtime
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${PI_USER}
WorkingDirectory=${paths.deployRoot}
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

/**
 * Restarts the runtime service and polls for up to 8 seconds that it's
 * actually stably running (active+running, with a main PID that doesn't
 * change and a restart count that doesn't increment) before declaring
 * success -- an unstable/failed activation automatically rolls back to
 * `previousTarget` if one is available. Mirrors the old dashboard's
 * `_activate_release` bash loop, just as real TS control flow instead of a
 * bash heredoc polling `systemctl show` in a `while` loop.
 */
export async function activateRelease(
  releaseDir: string,
  previousTarget: string,
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
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
 * A deliberate, operator-triggered "go back to the previous release" action
 * -- distinct from activateRelease's automatic rollback-on-failure. Swaps
 * current/previous symlinks (so a second rollback undoes the first, same as
 * the old dashboard's `rollback_build`), then reactivates through the same
 * health-check path.
 */
export async function rollbackRelease(
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
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

  return activateRelease(previousTarget, currentTarget, run, wait, now, paths);
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

export async function deployArtifact(
  options: DeployArtifactOptions,
  run: CommandRunner = runCommand,
  wait: (ms: number) => Promise<void> = delay,
  now: () => number = Date.now,
  paths: DeployPaths = deployPaths(),
): Promise<DeployArtifactResult> {
  const { releaseId, releaseDir, previousTarget } = await extractRelease(
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

  const activation = await activateRelease(releaseDir, previousTarget, run, wait, now, paths);
  return { ...activation, releaseId };
}
