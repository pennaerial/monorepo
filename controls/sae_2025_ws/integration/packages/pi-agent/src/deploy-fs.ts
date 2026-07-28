import { lstat, readlink, rm, symlink } from "node:fs/promises";

export async function pathExists(path: string): Promise<boolean> {
  try {
    await lstat(path);
    return true;
  } catch {
    return false;
  }
}

/** Resolves a symlink to an absolute target -- closer to `readlink -f` than
 *  plain `fs.readlink` (which can return a relative target), but doesn't
 *  require the target to actually exist (unlike `fs.realpath`), matching
 *  the old bash's tolerance for a broken/dangling previous-release link. */
export async function resolveSymlinkTarget(path: string): Promise<string> {
  const stat = await lstat(path).catch(() => null);
  if (!stat) return "";
  if (stat.isDirectory()) return path;
  if (!stat.isSymbolicLink()) return "";
  const target = await readlink(path);
  return target;
}

/** `ln -sfn target path` -- remove whatever's at `path` first (file, dir, or
 *  stale symlink), then create a fresh symlink. Node's `fs.symlink` errors if
 *  something already exists at the destination. */
export async function symlinkForce(target: string, path: string): Promise<void> {
  await rm(path, { force: true, recursive: true });
  await symlink(target, path);
}
