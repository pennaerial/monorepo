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
 * POSIX single-quote shell escaping. releaseRoot is derived from the
 * uploaded artifact's filename (see deploy.ts's releaseSlug), which is
 * client-controlled -- interpolating it into a `bash -c` string without this
 * is a shell injection vector (a filename containing `"`/`$()`/backticks
 * could execute arbitrary commands on the Pi the next time /api/schema is
 * called). Single quotes disable all shell interpreting; only an embedded
 * single quote itself needs escaping, via the standard close-quote,
 * escaped-quote, reopen-quote technique.
 */
function shellQuote(value: string): string {
  return `'${value.replace(/'/g, `'\\''`)}'`;
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

  // Semicolons, not && -- sourcing ROS2/the release overlay is best-effort
  // (2>/dev/null is deliberately silencing a missing-file error from
  // `source` itself, not just python's stderr). Chaining with && meant a
  // missing setup.bash short-circuited the whole command before python3
  // ever ran, with no output at all -- always run the script regardless, so
  // a real missing-vehicle_common failure surfaces its own clear stderr.
  const command = [
    "source /opt/ros/jazzy/setup.bash 2>/dev/null",
    `source ${shellQuote(`${releaseRoot}/install/setup.bash`)} 2>/dev/null`,
    `python3 ${shellQuote(scriptPath)} --root ${shellQuote(releaseRoot)}`,
  ].join("; ");

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
