import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { attachHeartbeat, type HeartbeatSocket } from "./heartbeat.js";

function fakeSocket() {
  const listeners: Record<string, Array<() => void>> = {};
  const socket: HeartbeatSocket = {
    ping: vi.fn(),
    terminate: vi.fn(),
    on: (event, listener) => {
      (listeners[event] ??= []).push(listener);
    },
  };
  return { socket, emitPong: () => listeners.pong?.forEach((l) => l()), emitClose: () => listeners.close?.forEach((l) => l()) };
}

describe("attachHeartbeat", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => vi.useRealTimers());

  it("pings on each interval as long as a pong keeps arriving", () => {
    const { socket, emitPong } = fakeSocket();
    attachHeartbeat(socket, 1000);

    vi.advanceTimersByTime(1000);
    expect(socket.ping).toHaveBeenCalledTimes(1);
    emitPong();

    vi.advanceTimersByTime(1000);
    expect(socket.ping).toHaveBeenCalledTimes(2);
    expect(socket.terminate).not.toHaveBeenCalled();
  });

  it("terminates the connection if no pong arrives before the next interval", () => {
    const { socket } = fakeSocket();
    attachHeartbeat(socket, 1000);

    vi.advanceTimersByTime(1000); // sends first ping, no pong follows
    vi.advanceTimersByTime(1000); // next tick: still no pong -> terminate

    expect(socket.terminate).toHaveBeenCalledTimes(1);
  });

  it("stops the interval once the socket closes", () => {
    const { socket, emitClose } = fakeSocket();
    attachHeartbeat(socket, 1000);

    emitClose();
    vi.advanceTimersByTime(10_000);

    expect(socket.ping).not.toHaveBeenCalled();
  });
});
