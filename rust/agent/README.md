# PennAir deployment agent

`pennair-agent` runs on a Raspberry Pi and adds authenticated deployment
services to a Foxglove WebSocket server. The GCS web app connects to that one
server for Foxglove channels, ROS-facing services, and deployment services.

The agent does not expose an SSH or shell-command API. A local console, an SD
card image, or SSH is still needed to install the agent for the first time and
can be retained as a recovery path.

## Build and install

Build the agent for the Pi and install the binary:

```bash
cargo build --manifest-path rust/agent/Cargo.toml --release --locked
sudo install -m 0755 rust/agent/target/release/pennair-agent /usr/local/bin/pennair-agent
```

Install the service template and enable it for the Pi user, shown here as `penn`:

```bash
sudo install -m 0644 rust/agent/systemd/pennair-agent@.service /etc/systemd/system/
sudo install -d -m 0755 /etc/pennair
sudo install -m 0600 rust/agent/systemd/pennair-agent.env.example /etc/pennair/agent.env
sudo systemctl daemon-reload
sudo systemctl enable --now pennair-agent@penn.service
```

Replace the example token in `/etc/pennair/agent.env` with a random secret of at
least 32 characters. An authorized operator enters that token in the GCS when
deploying. The browser proves possession of the token using HMAC-SHA-256, so the
token is not sent over the connection.

The default address is `0.0.0.0:8765`. Only one Foxglove server may bind that
address. Do not run a separate `foxglove_bridge` on the same port. The intended
production arrangement is one Foxglove server that owns both telemetry channels
and these deployment services.

## Restart permission

The agent invokes one fixed privileged operation:

```text
sudo -n systemctl restart pennair-autonomy.service
```

Copy the example sudoers rule, replace `penn` if needed, and validate it before
installation:

```bash
sudo visudo -cf rust/agent/systemd/pennair-agent.sudoers
sudo install -m 0440 rust/agent/systemd/pennair-agent.sudoers /etc/sudoers.d/pennair-agent
```

The agent does not accept shell commands from clients. Deployment, build, and
restart are separate allowlisted Foxglove service calls.

## GCS deployment

In Mission Control:

1. Enter the Pi Foxglove URL, such as `ws://air-04.local:8765`.
2. Enter the deployment token and select **Connect**.
3. Select the local repository folder. The browser reads `monorepo.toml` and
   includes only its `[deploy].paths` entries.
4. Select **Deploy selected repository**.

The GCS compares the selected files with the Pi inventory, resumes partial
files, verifies SHA-256 hashes, activates the staged tree, runs the fixed colcon
build, and optionally restarts `pennair-autonomy.service`.

Plain `ws://` is suitable only for local development on a trusted network. An
HTTPS-hosted GCS also requires `wss://`; TLS termination and certificate
provisioning are not implemented in this checkpoint.
