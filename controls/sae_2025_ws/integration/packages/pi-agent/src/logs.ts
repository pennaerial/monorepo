import { type ChildProcessWithoutNullStreams, spawn } from "node:child_process";

const SERVICE_NAME = process.env.MISSION_SERVICE_NAME ?? "pennair-autonomy.service";

export type Spawner = (command: string, args: string[]) => ChildProcessWithoutNullStreams;

export const realSpawn: Spawner = (command, args) => spawn(command, args);

export interface LogSocket {
  send(data: string): void;
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

  journalctl.stdout.on("data", (chunk: Buffer) => {
    socket.send(chunk.toString("utf-8"));
  });

  journalctl.stderr.on("data", (chunk: Buffer) => {
    socket.send(`[journalctl error] ${chunk.toString("utf-8")}`);
  });

  const cleanup = () => {
    if (!journalctl.killed) {
      journalctl.kill();
    }
  };

  socket.on("close", cleanup);
  journalctl.on("error", (error) => {
    socket.send(`[journalctl error] ${error.message}`);
    cleanup();
  });
}
