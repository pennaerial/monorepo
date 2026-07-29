import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { resolveSymlinkTarget, pathExists } from "./deploy-fs.js";
import { runCommand, type CommandRunner } from "./exec.js";

const SCHEMA_EXPORT_TIMEOUT_MS = 30_000;

function defaultSchemaExportScriptPath(): string {
  if (process.env.SCHEMA_EXPORT_SCRIPT_PATH) return process.env.SCHEMA_EXPORT_SCRIPT_PATH;
  // Resolved relative to this module's own location (stable regardless of
  // cwd/systemd WorkingDirectory), not process.cwd(): dist/schema-export.js
  // -> packages/pi-agent/dist -> up to integration/ -> tools/schema-export.
  const moduleDir = dirname(fileURLToPath(import.meta.url));
  return join(moduleDir, "..", "..", "..", "tools", "schema-export", "schema_export.py");
}

export interface SchemaExportResult {
  success: boolean;
  schema?: unknown;
  error?: string;
}

/**
 * Runs `schema_export.py` against the currently-deployed release, sourcing
 * ROS2 + the release's own colcon overlay first -- the Pi is guaranteed to
 * have a working ROS2 + vehicle_common environment (that's what it's
 * provisioned for), unlike the orchestrator, which may just be an
 * operator's laptop.
 */
export async function exportSchema(
  run: CommandRunner = runCommand,
  paths: DeployPaths = deployPaths(),
  scriptPath: string = defaultSchemaExportScriptPath(),
): Promise<SchemaExportResult> {
  const releaseRoot = await resolveSymlinkTarget(paths.currentLink);
  if (!releaseRoot || !(await pathExists(releaseRoot))) {
    return { success: false, error: "No build deployed" };
  }

  const command = [
    "source /opt/ros/jazzy/setup.bash 2>/dev/null",
    `source "${releaseRoot}/install/setup.bash" 2>/dev/null`,
    `python3 "${scriptPath}" --root "${releaseRoot}"`,
  ].join(" && ");

  const result = await run("bash", ["-c", command], SCHEMA_EXPORT_TIMEOUT_MS);
  if (result.code !== 0) {
    return { success: false, error: result.stderr || "schema export failed" };
  }
  try {
    return { success: true, schema: JSON.parse(result.stdout) };
  } catch {
    return { success: false, error: "schema export produced invalid JSON output" };
  }
}
