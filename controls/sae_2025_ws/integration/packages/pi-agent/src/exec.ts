import { execFile as execFileCallback } from "node:child_process";
import { promisify } from "node:util";

const execFileAsync = promisify(execFileCallback);

export interface ExecResult {
  code: number;
  stdout: string;
  stderr: string;
}

export type CommandRunner = (command: string, args: string[], timeoutMs: number) => Promise<ExecResult>;

/** Real command runner -- the default, unless a test injects a fake one. */
export const runCommand: CommandRunner = async (command, args, timeoutMs) => {
  try {
    const { stdout, stderr } = await execFileAsync(command, args, { timeout: timeoutMs });
    return { code: 0, stdout, stderr };
  } catch (error) {
    const execError = error as {
      code?: number | string;
      stdout?: string;
      stderr?: string;
      message: string;
    };
    // On a spawn failure (command not found, etc.) `code` is a string like
    // "ENOENT" and `stderr` is an empty string (not absent) -- fall back to
    // `.message` for a useful error, and normalize `code` to a number.
    const numericCode = typeof execError.code === "number" ? execError.code : -1;
    return {
      code: numericCode,
      stdout: execError.stdout ?? "",
      stderr: execError.stderr || execError.message,
    };
  }
};

/** Real delay -- the default, unless a test injects a no-op one. */
export const delay = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));
