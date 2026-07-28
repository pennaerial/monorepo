import { EventEmitter } from "node:events";
import type { ChildProcessWithoutNullStreams } from "node:child_process";
import { describe, expect, it, vi } from "vitest";
import { streamMissionLogs, type LogSocket, type Spawner } from "./logs.js";

function fakeChildProcess(): ChildProcessWithoutNullStreams & { killed: boolean } {
  const proc = new EventEmitter() as unknown as ChildProcessWithoutNullStreams & { killed: boolean };
  proc.stdout = new EventEmitter() as ChildProcessWithoutNullStreams["stdout"];
  proc.stderr = new EventEmitter() as ChildProcessWithoutNullStreams["stderr"];
  proc.killed = false;
  proc.kill = vi.fn(() => {
    proc.killed = true;
    return true;
  }) as ChildProcessWithoutNullStreams["kill"];
  return proc;
}

function fakeSocket() {
  const sent: string[] = [];
  const listeners: Record<string, Array<() => void>> = {};
  const socket: LogSocket = {
    send: (data) => sent.push(data),
    on: (event, listener) => {
      (listeners[event] ??= []).push(listener);
    },
  };
  return { socket, sent, close: () => listeners.close?.forEach((l) => l()) };
}

describe("streamMissionLogs", () => {
  it("spawns journalctl -f for the configured service and forwards stdout to the socket", () => {
    const proc = fakeChildProcess();
    const spawnImpl: Spawner = vi.fn(() => proc);
    const { socket, sent } = fakeSocket();

    streamMissionLogs(socket, spawnImpl);

    expect(spawnImpl).toHaveBeenCalledWith("sudo", [
      "-n",
      "journalctl",
      "-f",
      "-u",
      "pennair-autonomy.service",
      "-n",
      "200",
      "-o",
      "short-iso",
    ]);

    proc.stdout.emit("data", Buffer.from("2026-01-01T00:00:00 unit started\n"));
    expect(sent).toEqual(["2026-01-01T00:00:00 unit started\n"]);
  });

  it("forwards stderr as a prefixed error line", () => {
    const proc = fakeChildProcess();
    const { socket, sent } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    proc.stderr.emit("data", Buffer.from("Failed to get journal\n"));

    expect(sent).toEqual(["[journalctl error] Failed to get journal\n"]);
  });

  it("kills journalctl when the socket closes", () => {
    const proc = fakeChildProcess();
    const { socket, close } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    close();

    expect(proc.kill).toHaveBeenCalled();
  });

  it("does not try to kill an already-killed process", () => {
    const proc = fakeChildProcess();
    const { socket, close } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    close();
    close();

    expect(proc.kill).toHaveBeenCalledTimes(1);
  });
});
