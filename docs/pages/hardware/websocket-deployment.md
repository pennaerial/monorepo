# Foxglove deployment

## Architecture

Routine deployment goes through the GCS web app and the same Foxglove WebSocket
connection used by the rest of the GCS. There is no deployment CLI connection
and no SSH transport in this path.

| Component | Runs on | Responsibility |
| --- | --- | --- |
| GCS web app | User computer | Selects the repository, reads `monorepo.toml`, hashes files, calls deployment services, and displays progress |
| `pennair-agent` | Raspberry Pi | Hosts Foxglove deployment services, stages and verifies files, activates a deployment, builds ROS packages, and restarts autonomy |
| `pennair-protocol` | GCS and Pi | Defines the typed service payloads, authentication proofs, manifests, and binary file chunks |
| Foxglove SDK | GCS and Pi | Supplies the WebSocket connection, channel protocol, service discovery, and service calls |

The custom PennAir code defines deployment services on top of the Foxglove SDK.
Foxglove supplies the service mechanism, but it does not supply repository
deployment semantics.

## One connection

The Pi should expose one Foxglove server, normally at
`ws://<pi-hostname>:8765`. That server can advertise both telemetry channels and
the `/pennair/deployment/*` services. The GCS connects once, then uses the same
`NodeHandle` for subscriptions, ordinary ROS-facing service calls, and
deployment service calls.

Do not start `pennair-agent` and a separate `foxglove_bridge` on the same port.
The current agent is the Foxglove SDK server for deployment. A production image
that also needs ROS telemetry must register or forward those telemetry channels
through this same server instead of creating a second browser connection. That
ROS telemetry integration still requires Pi testing.

## Deployment sequence

1. The user enters the Pi URL and deployment token in Mission Control and connects.
2. The user selects the repository folder in the browser.
3. The GCS reads the selected `monorepo.toml` and retains only `[deploy].paths`.
4. The GCS hashes those files and requests the Pi's active inventory.
5. The GCS authenticates with a challenge and HMAC-SHA-256 proof.
6. The GCS sends a signed manifest and the Pi identifies missing files.
7. Missing files are sent in resumable binary chunks over Foxglove service calls.
8. The Pi verifies every SHA-256 digest before activating the staged tree.
9. The Pi runs the fixed colcon build command and returns its output.
10. After a successful build, the Pi optionally restarts `pennair-autonomy.service`.

Failed validation or builds do not mark the deployment as successfully built.
Activation keeps transaction backups so failed activation can restore replaced
or deleted files.

## Safety boundaries

- The token must contain at least 32 characters and is never sent directly.
- Paths are relative, must remain under the deployment root, and cannot enter
  the reserved `.pennair` directory.
- The Pi rejects symlinks and non-regular destination files during activation.
- Build and restart are fixed agent operations. A client cannot submit a shell
  command or executable path.
- File chunks are bounded to 64 KiB, acknowledged, resumable, and verified.
- Deletions come only from the difference between the authenticated Pi inventory
  and the selected deployment manifest.
- Plain `ws://` does not protect against an active network attacker. Production
  use and an HTTPS-hosted GCS require `wss://` with trusted certificates.

## Pi software requirements

The Pi does not need the entire monorepo's developer tooling. The current
build-on-Pi design does need:

- `pennair-agent`
- ROS 2 Jazzy runtime and development packages used by deployed packages
- `colcon`, a C++ toolchain, and required Python dependencies
- the narrow sudoers permission for restarting `pennair-autonomy.service`

The Pi does not need the GCS source or the Rust toolchain when a prebuilt ARM64
agent binary is installed. A future artifact deployment mode could move colcon
and compilation to CI or the user's computer, leaving only ROS runtime packages
and the agent on the Pi.

## Current checkpoint

The GCS-to-agent flow is implemented through Foxglove services and passes a
native end-to-end test using harmless build and restart substitutes. The test
authenticates, transfers files, activates them, builds, restarts, deploys the
same tree again, and confirms that unchanged files are not retransmitted.

The browser target compiles, but a real browser, ARM64 Pi, ROS environment,
systemd service, Wi-Fi interruption, TLS, and combined telemetry plus deployment
server have not yet been exercised.

Agent configuration:

| Variable | Default |
| --- | --- |
| `PENNAIR_AGENT_BIND` | `0.0.0.0:8765` |
| `PENNAIR_AGENT_ROOT` | `$HOME/monorepo` |
| `PENNAIR_AGENT_ROS_SETUP` | `/opt/ros/jazzy/setup.sh` |
| `PENNAIR_AGENT_COLCON` | `colcon` |
| `PENNAIR_AGENT_SUDO` | `sudo` |
| `PENNAIR_AGENT_SYSTEMCTL` | `systemctl` |
| `PENNAIR_AGENT_SERVICE` | `pennair-autonomy.service` |

Initial installation and recovery may still use an SD card image, local console,
or SSH. Routine source transfer no longer depends on SSH or rsync.
