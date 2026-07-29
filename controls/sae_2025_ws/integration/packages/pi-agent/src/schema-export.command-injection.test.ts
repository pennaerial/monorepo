import { mkdir, mkdtemp, readFile, rm, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { exportSchema } from "./schema-export.js";
import { runCommand } from "./exec.js";

/**
 * Reproduces a real command-injection vulnerability, using the REAL
 * `runCommand` (packages/pi-agent/src/exec.ts), which really spawns
 * `bash -c <command>`, not a mocked CommandRunner.
 *
 * schema-export.ts builds its command string as:
 *
 *   `source "${releaseRoot}/install/setup.bash" 2>/dev/null`
 *   `python3 "${scriptPath}" --root "${releaseRoot}"`
 *
 * `releaseRoot` is interpolated into a double-quoted bash -c string with no
 * escaping (schema-export.ts lines 47-51). `releaseRoot` is the resolved
 * target of the deploy `current` symlink, which points at
 * `<releasesDir>/<releaseId>`.
 *
 * `releaseId` is NOT purely server-derived: it's built in deploy.ts as
 * `${releaseTimestamp()}-${releaseSlug}-${randomBytes(3)}` where
 * `releaseSlug = artifactName.replace(/\.tar\.gz$/, "").replace(/\.tgz$/, "")`
 * and `artifactName` is the client-supplied multipart upload filename,
 * completely unsanitized (pi-agent/src/index.ts line 62:
 * `artifactPath = \`${paths.incomingDir}/${part.filename}\`;` and line 86:
 * `const filename = artifactPath.split("/").pop() ?? "artifact.tar.gz";` --
 * this only strips "/", not quote characters). A double quote character
 * anywhere in an uploaded artifact's filename survives, unescaped, all the
 * way into the release directory name, and from there into this shell
 * command string.
 *
 * This test simulates that: a "release directory" whose name contains a
 * `"` followed by an injected shell command, exactly as would result from
 * uploading a deploy artifact with a crafted filename. It uses the real
 * exec.ts runCommand (real `bash -c`, real child process) and proves
 * arbitrary command execution by observing a sentinel file get created
 * outside of anything schema_export.py itself would ever touch.
 */
describe("exportSchema command construction with an attacker-influenced release directory name", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-schema-export-injection-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("executes an injected shell command embedded in the release directory name", async () => {
    const sentinel = join(scratchRoot, "PWNED");

    // Mirrors deploy.ts's releaseId shape: <timestamp>-<artifactName-derived slug>-<random>.
    // The slug here is exactly what an attacker-controlled upload filename
    // (unsanitized end to end) would produce.
    const maliciousReleaseDirName = `20260101-000000-test"; touch "${sentinel}"; echo "pwned`;
    const releaseDir = join(scratchRoot, "releases", maliciousReleaseDirName);
    await mkdir(releaseDir, { recursive: true });
    await mkdir(scratchRoot, { recursive: true });
    await symlink(releaseDir, paths.currentLink);

    // A harmless stub standing in for schema_export.py.
    const scriptPath = join(scratchRoot, "stub_schema_export.py");
    await writeFile(scriptPath, "print('{}')\n", "utf-8");

    await exportSchema(runCommand, paths, scriptPath);

    const sentinelExists = await readFile(sentinel, "utf-8")
      .then(() => true)
      .catch(() => false);

    // What should happen: a release directory name should never be able to
    // inject arbitrary shell commands into schema export, regardless of
    // what characters ended up in it. Currently it can -- this assertion
    // fails against the buggy implementation (the sentinel file gets
    // created, proving real command execution).
    expect(sentinelExists).toBe(false);
  });
});
