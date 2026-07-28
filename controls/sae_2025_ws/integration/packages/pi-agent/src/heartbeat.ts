export interface HeartbeatSocket {
  ping(): void;
  terminate(): void;
  on(event: "pong" | "close", listener: () => void): void;
}

/**
 * Detects a client that vanished without a clean close handshake (WiFi
 * drop, app killed, laptop sleep) -- without this, the server only notices
 * via the OS's own (often very slow, sometimes effectively unbounded) TCP
 * dead-peer detection, leaving whatever the connection was holding open
 * (a spawned journalctl process, an upstream relay connection) running
 * longer than necessary.
 */
export function attachHeartbeat(socket: HeartbeatSocket, intervalMs = 30_000): void {
  let alive = true;
  socket.on("pong", () => {
    alive = true;
  });

  const timer = setInterval(() => {
    if (!alive) {
      socket.terminate();
      return;
    }
    alive = false;
    socket.ping();
  }, intervalMs);

  socket.on("close", () => clearInterval(timer));
}
