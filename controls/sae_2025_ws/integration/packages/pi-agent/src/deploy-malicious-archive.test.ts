import { mkdir, mkdtemp, readdir, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { randomBytes } from "node:crypto";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists } from "./deploy-fs.js";
import { extractRelease } from "./deploy.js";

describe("extractRelease with malformed/malicious archives (packages/pi-agent/src/deploy.ts)", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-malicious-archive-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  /**
   * Regression test: when `tar.x` throws (a non-gzip file, a
   * truncated/corrupted download, garbage bytes), extractRelease now cleans
   * up both the raw uploaded artifact and the just-created empty release
   * directory before rethrowing, instead of leaking them on every
   * failed/garbage upload -- a real disk-exhaustion risk on a
   * storage-constrained Raspberry Pi given a flaky field WiFi link.
   */
  it("cleans up the incoming artifact file and the empty release dir when the archive is not a valid tar.gz", async () => {
    const artifactPath = join(scratchRoot, "garbage.tar.gz");
    await writeFile(artifactPath, randomBytes(4096)); // not a valid gzip/tar stream

    await expect(extractRelease(artifactPath, "garbage.tar.gz", paths)).rejects.toThrow();

    expect(await pathExists(artifactPath), "the raw uploaded artifact should not be left behind after a failed extraction").toBe(false);

    const releasesEntries = await readdir(paths.releasesDir).catch(() => []);
    expect(releasesEntries, "no empty/partial release directory should be left behind after a failed extraction").toEqual([]);
  });

  it("does not let a crafted archive with ../-escaping entry names write outside the release directory", async () => {
    // node-tar (v7.x) sanitizes both absolute paths and ../ traversal by
    // default -- this is a real check (not assumed), confirming the
    // extraction path itself is safe against a classic path-traversal tar.
    const outsideDir = join(scratchRoot, "outside-canary");
    await mkdir(outsideDir, { recursive: true });
    await writeFile(join(outsideDir, "canary.txt"), "ORIGINAL", "utf-8");

    // Hand-build a minimal ustar archive with a ../-escaping entry, since
    // the real `tar` package's own .c() sanitizes input paths on creation
    // (we need to bypass that to build a genuinely malicious archive).
    function ustarHeader(name: string, size: number): Buffer {
      const buf = Buffer.alloc(512);
      buf.write(name, 0, 100, "utf-8");
      buf.write("0000777\0", 100, 8, "utf-8");
      buf.write("0000000\0", 108, 8, "utf-8");
      buf.write("0000000\0", 116, 8, "utf-8");
      buf.write(size.toString(8).padStart(11, "0") + "\0", 124, 12, "utf-8");
      buf.write("00000000000\0", 136, 12, "utf-8");
      buf.write("        ", 148, 8, "utf-8");
      buf.write("0", 156, 1, "utf-8");
      buf.write("ustar\0", 257, 6, "utf-8");
      buf.write("00", 263, 2, "utf-8");
      let sum = 0;
      for (let i = 0; i < 512; i++) sum += buf[i];
      buf.write(sum.toString(8).padStart(6, "0") + "\0 ", 148, 8, "utf-8");
      return buf;
    }
    function fileEntry(name: string, content: string): Buffer {
      const data = Buffer.from(content, "utf-8");
      const header = ustarHeader(name, data.length);
      const padLen = (512 - (data.length % 512)) % 512;
      return Buffer.concat([header, data, Buffer.alloc(padLen)]);
    }

    const zlib = await import("node:zlib");
    const entries = Buffer.concat([
      fileEntry("install/marker.txt", "v1"),
      fileEntry("../../outside-canary/canary.txt", "OVERWRITTEN"),
      Buffer.alloc(1024),
    ]);
    const archivePath = join(scratchRoot, "traversal.tar.gz");
    await writeFile(archivePath, zlib.gzipSync(entries));

    const result = await extractRelease(archivePath, "traversal.tar.gz", paths);
    expect(await pathExists(`${result.releaseDir}/install/marker.txt`)).toBe(true);

    const { readFile } = await import("node:fs/promises");
    const canaryContent = await readFile(join(outsideDir, "canary.txt"), "utf-8");
    expect(canaryContent).toBe("ORIGINAL"); // confirms the traversal entry did NOT escape releaseDir
  });
});
