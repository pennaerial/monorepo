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

/** `simulateClientDisconnect` fires the socket's "close" event (the WS client went away).
 *  `socket.close` (a spy) is what OUR code calls to close the socket itself -- distinct things. */
function fakeSocket() {
  const sent: string[] = [];
  const listeners: Record<string, Array<() => void>> = {};
  const socket: LogSocket = {
    send: (data) => sent.push(data),
    close: vi.fn(),
    on: (event, listener) => {
      (listeners[event] ??= []).push(listener);
    },
  };
  return { socket, sent, simulateClientDisconnect: () => listeners.close?.forEach((l) => l()) };
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
    const { socket, simulateClientDisconnect } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    simulateClientDisconnect();

    expect(proc.kill).toHaveBeenCalled();
  });

  it("does not try to kill an already-killed process", () => {
    const proc = fakeChildProcess();
    const { socket, simulateClientDisconnect } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    simulateClientDisconnect();
    simulateClientDisconnect();

    expect(proc.kill).toHaveBeenCalledTimes(1);
  });

  it("notifies and closes the socket if journalctl exits unexpectedly (not from our own cleanup)", () => {
    const proc = fakeChildProcess();
    const { socket, sent } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    proc.emit("exit", 1, null);

    expect(sent).toEqual(["[journalctl exited unexpectedly] code=1 signal=null"]);
    expect(socket.close).toHaveBeenCalledTimes(1);
  });

  it("does not send an unexpected-exit notice for our own intentional kill on client disconnect", () => {
    const proc = fakeChildProcess();
    const { socket, sent, simulateClientDisconnect } = fakeSocket();

    streamMissionLogs(socket, () => proc);
    simulateClientDisconnect();
    proc.emit("exit", null, "SIGTERM"); // the kill() we issued actually taking effect

    expect(sent).toEqual([]);
    expect(socket.close).not.toHaveBeenCalled();
  });
});
