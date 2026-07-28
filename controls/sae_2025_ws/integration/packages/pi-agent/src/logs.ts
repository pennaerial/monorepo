import { type ChildProcessWithoutNullStreams, spawn } from "node:child_process";

const SERVICE_NAME = process.env.MISSION_SERVICE_NAME ?? "pennair-autonomy.service";

export type Spawner = (command: string, args: string[]) => ChildProcessWithoutNullStreams;

export const realSpawn: Spawner = (command, args) => spawn(command, args);

export interface LogSocket {
  send(data: string): void;
  close(): void;
  on(event: "close", listener: () => void): void;
}

/**
 * Streams `journalctl -f` for the mission service directly to a WebSocket --
 * real continuous streaming, replacing the old dashboard's 1-second
 * poll-and-resend-the-whole-buffer hack.
 */
export function streamMissionLogs(socket: LogSocket, spawnImpl: Spawner = realSpawn): void {
  const journalctl = spawnImpl("sudo", [
    "-n",
    "journalctl",
    "-f",
    "-u",
    SERVICE_NAME,
    "-n",
    "200",
    "-o",
    "short-iso",
  ]);
  let stoppedIntentionally = false;

  journalctl.stdout.on("data", (chunk: Buffer) => {
    socket.send(chunk.toString("utf-8"));
  });

  journalctl.stderr.on("data", (chunk: Buffer) => {
    socket.send(`[journalctl error] ${chunk.toString("utf-8")}`);
  });

  const cleanup = () => {
    stoppedIntentionally = true;
    if (!journalctl.killed) {
      journalctl.kill();
    }
  };

  socket.on("close", cleanup);

  journalctl.on("error", (error) => {
    socket.send(`[journalctl error] ${error.message}`);
    cleanup();
  });

  // journalctl -f is meant to run forever; if it exits on its own (sudoers
  // misconfigured, the unit rotated away, journalctl itself crashing) the
  // client would otherwise be left with an open socket and no more data and
  // no signal the stream ended -- tell it explicitly and close the socket.
  journalctl.on("exit", (code, signal) => {
    if (stoppedIntentionally) return;
    socket.send(`[journalctl exited unexpectedly] code=${code} signal=${signal}`);
    socket.close();
  });
}
