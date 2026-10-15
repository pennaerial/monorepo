# PennAiR Ground Control Station (GCS)

A native desktop ground control station for PennAiR's UAVs, written in Rust with
[Dioxus](https://dioxuslabs.com/). It connects to the drone's ROS 2 graph over a
WebSocket using the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol),
subscribes to topics, and decodes the CDR-encoded ROS messages into strongly typed
Rust structs. It needs no ROS installation on the machine running the GCS.

> **Status: early development.** Right now the app has a single "Mission Control"
> page with a **Connect** button. The button connects to `ws://localhost:8765`,
> subscribes to a hard-coded channel (ID `4`) and prints each decoded
> `geometry_msgs/msg/Pose` to the terminal. Telemetry views, topic selection and
> commanding haven't been built yet.

---

## Contents

- [How it works](#how-it-works)
- [Why Rust?](#why-rust)
- [Repository layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Running the GCS](#running-the-gcs)
- [Connecting to ROS in a UTM VM (macOS)](#connecting-to-ros-in-a-utm-vm-macos)
- [Development guide](#development-guide)
  - [Adding UI](#adding-ui)
  - [Working with the Foxglove client](#working-with-the-foxglove-client)
  - [Adding ROS message types](#adding-ros-message-types)
  - [Tests, lint and formatting](#tests-lint-and-formatting)
- [Troubleshooting](#troubleshooting)
- [Roadmap / known gaps](#roadmap--known-gaps)

---

## How it works

```
┌──────────────── Drone / sim (Ubuntu + ROS 2) ───────────────┐        ┌─────────── GCS (this package) ───────────┐
│                                                              │        │                                           │
│  ROS 2 nodes ──topics──▶ foxglove_bridge  (ws://0.0.0.0:8765)│◀──WS──▶│ FoxgloveClient (tokio-tungstenite)        │
│                          subprotocol: foxglove.sdk.v1        │        │   │ JSON text  → serverInfo / advertise   │
│                                                              │        │   │ binary     → parse_binary_message()   │
└──────────────────────────────────────────────────────────────┘        │   ▼                                       │
                                                                        │ tokio broadcast channel (ServerMessage)   │
                                                                        │   ▼                                       │
                                                                        │ cdr::deserialize::<ros_interfaces::…>()   │
                                                                        │   ▼                                       │
                                                                        │ Dioxus UI (MissionControl view)           │
                                                                        └───────────────────────────────────────────┘
```

1. **`foxglove_bridge`** runs next to ROS on the drone or in the simulator VM.
   It advertises every ROS topic as a numbered *channel*.
2. **`FoxgloveClient`** (`src/ros/foxglove_client.rs`) opens a WebSocket using the
   `foxglove.sdk.v1` subprotocol. It splits the socket into a writer, which sends
   `subscribe` requests, and a reader loop that runs as a tokio task.
3. The reader loop handles two kinds of frames:
   - **Text frames** are JSON control messages such as `serverInfo` and `advertise`.
     At the moment they're only printed. `server_types.rs` already has serde types for
     `ServerInfo` and `Advertise`.
   - **Binary frames** go through `parse_binary_message` (`src/ros/parse.rs`).
     A `MessageData` frame looks like this:
     `[opcode: u8 = 1][subscription_id: u32 LE][timestamp: u64 LE][payload…]`.
4. Parsed `ServerMessage`s are sent out on a `tokio::sync::broadcast` channel.
   Any part of the UI can call `client.get_receiver()` to get its own receiver.
5. The payload is a ROS 2 **CDR** buffer. It's decoded with the `cdr` crate into
   structs from the generated **`ros_interfaces`** crate.
6. **Dioxus** renders the UI in a native webview window, styled with Tailwind.

The `ts/` directory holds the TypeScript Foxglove client from the GCS's earlier
Tauri + React version. It isn't built or used. It's kept as a reference for porting
the remaining protocol opcodes and message types to Rust.

## Why Rust?

The GCS started as a Tauri + React + TypeScript app and was rewritten in Rust/Dioxus.
The reasons:

- **One language, one process.** In Tauri, the UI is TypeScript and the backend is
  Rust, connected through IPC. With Dioxus, the UI, the networking and the message
  decoding are all Rust in one binary, so there's no JS↔Rust serialization boundary
  or second toolchain.
- **Performance for high-rate telemetry.** Decoding binary CDR frames from many
  topics, possibly images and point clouds, is cheap in native code. `Bytes` slices
  let the payload pass from the socket to the decoder without copying.
- **Type safety end to end.** ROS message definitions (`.idl`) are generated into
  Rust structs. If a message definition changes, a mismatch becomes a compile error
  instead of an `undefined` at runtime. Serde handles both JSON (the protocol) and
  CDR (the payloads).
- **Reliability.** A ground station that crashes mid-flight is a safety problem.
  Rust's ownership model, exhaustive `match` and explicit `Result` error handling
  rule out whole classes of runtime failures. `clippy.toml` adds a lint against
  holding Dioxus signal borrows across `.await`.
- **No ROS dependency on the operator's laptop.** The GCS only speaks WebSocket and
  CDR, so it builds and runs natively on macOS, Windows or Linux without ROS.

## Repository layout

```
gcs/
├─ Cargo.toml              # Workspace root + gcs crate (features: desktop [default], web, mobile)
├─ Cargo.lock
├─ Dioxus.toml             # dx CLI config (window/web title, resources)
├─ clippy.toml             # Lints: don't hold signal reads/writes across .await
├─ tailwind.css            # Tailwind input (dx compiles it automatically)
├─ assets/
│  ├─ tailwind.css         # Tailwind output; loaded in main.rs via asset!()
│  ├─ favicon.ico
│  └─ styling/, header.svg # Leftovers from the Dioxus template (unused)
├─ src/
│  ├─ main.rs              # Entry point; App root component
│  ├─ components.rs        # Shared UI components module
│  ├─ components/button.rs # Button (shadcn-style Tailwind classes)
│  ├─ views.rs             # Page-level views module
│  ├─ views/mission_control.rs  # MissionControl page + run_client() demo
│  ├─ ros.rs               # ROS/Foxglove module root
│  └─ ros/
│     ├─ foxglove_client.rs # WebSocket client, subscribe, broadcast of messages
│     ├─ parse.rs           # Binary frame parser
│     └─ server_types.rs    # Serde types for Foxglove server messages + opcodes
├─ crates/ros_interfaces/  # GENERATED crate: Rust structs for ROS messages (don't edit by hand)
├─ idl/                    # ROS .idl definitions that feed the generator
│  ├─ geometry_msgs/msg/{Point,Pose,Quaternion}.idl
│  └─ std_msgs/msg/{Float64,Int32,String}.idl
├─ generate_rs.py          # idl/ → crates/ros_interfaces generator (needs a ROS env)
├─ test/test_generate_rs.py# Generator regression tests
└─ ts/                     # Reference TypeScript Foxglove client (not built)
```

## Prerequisites

You need these on the machine that **runs the GCS**. On macOS that's normally the
host, not the VM.

1. **Rust** (stable) via [rustup](https://rustup.rs/):
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```
2. **Dioxus CLI (`dx`)**, matching the Dioxus 0.7 dependency:
   ```bash
   cargo install dioxus-cli --version "^0.7" --locked
   # or, faster, if you have cargo-binstall:
   cargo binstall dioxus-cli@0.7
   ```
   `dx` downloads Tailwind for you. You don't need Node or npm.
3. **Platform webview dependencies**:
   - **macOS**: nothing extra. It uses the system WebKit, and Xcode command line
     tools provide the toolchain (`xcode-select --install`).
   - **Windows**: WebView2, which ships with Windows 10/11.
   - **Linux / inside the Ubuntu VM**:
     ```bash
     sudo apt install libwebkit2gtk-4.1-dev build-essential curl wget file \
       libxdo-dev libssl-dev libayatana-appindicator3-dev librsvg2-dev
     ```

On the **ROS side**, either the drone or the sim VM:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

## Running the GCS

### 1. Start ROS and the Foxglove bridge (VM / drone)

```bash
# Terminal 1: your ROS stack, e.g. the SITL sim
cd $PENNAIR_SAE_WS_PATH
colcon build && source install/setup.bash
ros2 launch uav uav_sitl.launch.py

# Terminal 2: the bridge (listens on 0.0.0.0:8765 by default)
# One-time install: it isn't preinstalled with ROS or on the PennAiR VM image
sudo apt update && sudo apt install ros-$ROS_DISTRO-foxglove-bridge
# Source the workspace overlay, not just /opt/ros, so the bridge can find
# px4_msgs and the other PennAiR message packages built in sae_2025_ws
source $PENNAIR_SAE_WS_PATH/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Without a full sim, a test publisher gives the current demo something to decode:

```bash
ros2 topic pub -r 2 /test_pose geometry_msgs/msg/Pose \
  "{position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}"
```

### 2. Start the GCS (host)

```bash
cd gcs
dx serve            # builds and launches the desktop app with hot-reload
```

Other useful commands:

```bash
dx serve --release            # optimized build
dx bundle --release           # produce a distributable app bundle (.app / .msi / .deb …)
cargo run                     # plain cargo works too (no hot-reload, no auto-Tailwind)
```

> Only the **desktop** platform works right now. `tokio::net` and `tokio-tungstenite`
> don't compile to WebAssembly, so `--platform web` fails until the client gets a
> browser WebSocket backend.

### 3. Connect

Click **Connect**. The output appears in the terminal where you ran `dx serve`:

- `Connection Success with subprotocol: foxglove.sdk.v1`
- The JSON `serverInfo` and `advertise` messages. The `advertise` message lists every
  channel with its `id`, `topic` and `schemaName`.
- `SUCCESSFUL channel sub: 0`
- `successful cdr: Pose { position: Point { … }, … }` for each message, **if channel
  `4` is a `geometry_msgs/msg/Pose` topic.**

Channel IDs depend on the order in which the bridge advertised topics. To watch a
different topic, find its `id` in the printed `advertise` JSON. Then change
`client.subscribe(4)` and the `cdr::deserialize::<…>` type in
`src/views/mission_control.rs`. The URL is hard-coded there as well
(`ws://localhost:8765`).

## Connecting to ROS in a UTM VM (macOS)

On macOS, PennAiR runs ROS inside a UTM Ubuntu VM (see
[`docs/pages/installation/macos_utm.md`](../docs/pages/installation/macos_utm.md)).
The GCS should run on the **macOS host**, where it builds faster and gets a native
window. So the host has to reach port `8765` inside the VM. You have three options.
**Option A** works with the hard-coded `ws://localhost:8765` and leaves the VM's networking alone.

### SSH tunnel (recommended, no VM network changes)

Keep the VM on UTM's default **Shared Network** mode, so its internet access and
`git pull` are unaffected. Find the VM's IP inside the VM with `ip -4 addr`
(usually `192.168.64.x`). Then forward the port over SSH from macOS:

```bash
ssh -N -L 8765:localhost:8765 <user>@192.168.64.x
```

If you're on the default pennair UTM, <user> is "ubuntu"

While that runs, `ws://localhost:8765` on the host reaches the bridge in the VM.
You need `openssh-server` installed in the VM (`sudo apt install openssh-server`).
Check it from macOS with `nc -vz localhost 8765`.

### Running the GCS inside the VM instead

This also works. Install the Linux prerequisites above inside the VM and run
`dx serve` there. `localhost:8765` then works without any forwarding. Expect slower
builds and a less responsive UI than running natively on the host.

## Development guide

### Day-to-day loop

```bash
dx serve          # leave running; Rust changes rebuild, RSX/Tailwind changes hot-reload
```

`println!` output from the app shows up in the `dx serve` terminal. Tailwind classes
used anywhere in `src/` are picked up from `tailwind.css` and compiled into
`assets/tailwind.css` automatically.

### Adding UI

- **Views** (pages) go in `src/views/<name>.rs`. Declare them in `src/views.rs`
  (`mod <name>; pub use <name>::<Component>;`).
- **Reusable components** go in `src/components/<name>.rs` and are re-exported from
  `src/components.rs`.
- Components are functions with `#[component]` that return `rsx! { … }`. Style them
  with Tailwind classes. `Button` follows shadcn/ui conventions, so you can borrow
  class lists from shadcn when adding components.
- The `router` feature of Dioxus is enabled, but no `Routable` enum exists yet.
  `App` renders `MissionControl` directly. When you add a second page, introduce a
  `#[derive(Routable)] enum Route` in `main.rs` and render `Router::<Route> {}`.
- **Signals and async:** don't hold a signal `.read()` or `.write()` guard across an
  `.await`. `clippy.toml` enforces this. Clone or copy the value out first.

### Working with the Foxglove client

```rust
let mut client = FoxgloveClient::new();
client.connect("ws://localhost:8765").await?;   // spawns the reader task
let sub_id = client.subscribe(channel_id).await?;
let mut rx = client.get_receiver()?;            // one receiver per consumer
while let Ok(msg) = rx.recv().await {
    if let ServerMessage::MessageData(m) = msg {
        if m.subscription_id == sub_id {
            let pose: geometry_msgs::msg::Pose = cdr::deserialize(&m.data)?;
        }
    }
}
```

Things to know:

- The broadcast channel holds **100** messages. A slow receiver gets
  `RecvError::Lagged` and skips messages. It doesn't block other receivers.
- To support another **binary** opcode (`Time`, `ServiceCallResponse`,
  `FetchAssetResponse`), add a branch in `parse.rs` and a variant/struct in
  `server_types.rs`.
- To support **JSON** server messages, uncomment or add the variant on
  `ServerMessage` (it's `#[serde(tag = "op")]`). Then parse `Message::Text` in
  `on_message_loop` with `serde_json::from_str::<ServerMessage>` and broadcast it,
  the way binary frames are handled. That's how you'd get `Advertise` into the UI and
  resolve topic names to channel IDs instead of hard-coding them.
- The protocol is specified in the
  [Foxglove ws-protocol spec](https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md).
  `ts/` has a complete working client to port from.

### Adding ROS message types

`crates/ros_interfaces` is **generated**. Don't edit it by hand, because the generator
deletes and rewrites `src/`. To add a message, for example `sensor_msgs/msg/NavSatFix`:

1. **In an environment with ROS 2 sourced** (the VM): copy the `.idl` file, plus every
   type it references, into `idl/<package>/msg/`. ROS installs `.idl` files alongside
   the `.msg` files:
   ```bash
   cp /opt/ros/$ROS_DISTRO/share/sensor_msgs/msg/NavSatFix.idl    idl/sensor_msgs/msg/
   cp /opt/ros/$ROS_DISTRO/share/sensor_msgs/msg/NavSatStatus.idl idl/sensor_msgs/msg/
   cp /opt/ros/$ROS_DISTRO/share/std_msgs/msg/Header.idl          idl/std_msgs/msg/
   cp /opt/ros/$ROS_DISTRO/share/builtin_interfaces/msg/Time.idl  idl/builtin_interfaces/msg/
   ```
   The generator resolves referenced types from `idl/`. If one is missing it fails with
   `IDL for referenced type … not found`. For PennAiR's custom interfaces, run
   `colcon build` and copy them from `install/<pkg>/share/<pkg>/msg/`.
2. Regenerate:
   ```bash
   python3 generate_rs.py            # all packages under idl/ → crates/ros_interfaces
   python3 generate_rs.py --help     # subset packages, custom output dir, crate name, …
   ```
   The script needs `ROS_DISTRO` and `rosidl_parser`, which is why it has to run in
   the ROS environment.
3. Commit both the new `idl/` files and the regenerated `crates/ros_interfaces`.
   Then the GCS builds on machines without ROS, such as the macOS host.
4. Use the type: `ros_interfaces::sensor_msgs::msg::NavSatFix`.

Type mapping: primitives map to their Rust equivalents (`float64`→`f64`,
`uint8`→`u8`, …). `string` becomes `std::string::String`. Fixed arrays become
`[T; N]` and sequences become `Vec<T>`. Nested types are referenced by full path
(`crate::pkg::msg::Type`).

### Tests, lint and formatting

```bash
cargo fmt                       # format
cargo clippy --all-targets      # lint (includes clippy.toml rules)
cargo test                      # Rust tests (none yet; add #[cfg(test)] modules, e.g. for parse.rs)

# Generator tests: run from gcs/ in a ROS-sourced shell with cargo available
python3 -m unittest discover -s test -p test_generate_rs.py
```

> The docstring in `test/test_generate_rs.py` still uses the old
> `experimental/gcs/test` path. Use `-s test` from `gcs/` (or `-s gcs/test` from the
> monorepo root).

Before you open a PR, make sure `cargo fmt --check`, `cargo clippy` and `dx build` all
pass. CI doesn't build the GCS yet (see the TODO in `.github/workflows/mergebuild.yml`).

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `connection failed! … Connection refused` | The bridge isn't running, or port 8765 isn't reachable. Check with `nc -vz localhost 8765` on the host and `ss -ltnp \| grep 8765` in the VM. Recheck the SSH tunnel or UTM port forward. |
| `git pull` / internet stops working in the VM after enabling port forwarding | Emulated VLAN changed the VM's subnet. See [Fixing internet access](#fixing-internet-access-after-switching-to-emulated-vlan), or go back to Shared Network and use the SSH tunnel. |
| `SUBCRIBE ERROR: … NotConnected` | The connection failed first. See the line above it. |
| Connects, but nothing is decoded | Channel `4` isn't the topic you expect, or that topic has no publisher. Look up the right `id` in the printed `advertise` JSON. |
| `Failed cdr deserialization` | The Rust type passed to `cdr::deserialize` doesn't match the channel's `schemaName`. |
| `Warning: no active listeners to message …` | Nothing has called `get_receiver()` yet. This is harmless during startup. |
| Bridge logs `Failed to load schemaDefinition … package 'px4_msgs' not found` | The bridge's terminal only sourced `/opt/ros/$ROS_DISTRO`. Run `source $PENNAIR_SAE_WS_PATH/install/setup.bash` (after `colcon build`) and restart the bridge. |
| `os.environ["ROS_DISTRO"]` KeyError when running `generate_rs.py` | ROS isn't sourced. Run `source /opt/ros/$ROS_DISTRO/setup.bash` (in the VM). |
| `dx` version mismatch warnings | Install a `dioxus-cli` 0.7.x that matches the `dioxus` version in `Cargo.toml`. |
| Build fails with `--platform web` | Expected. Only desktop is supported (see above). |

## Roadmap / known gaps

- Parse JSON server messages (`serverInfo`, `advertise`, `status`, …) into
  `ServerMessage` and broadcast them.
- Pick topics by name, using `advertise`, instead of hard-coded channel IDs.
  Make the bridge URL configurable.
- Show telemetry in the UI (pose, battery, state) instead of `println!`.
- Client → server messages: `unsubscribe`, client publishing, service calls.
- Handle reconnects and surface connection state in the UI.
- Unit tests for `parse.rs`. Add the GCS to CI.
- Remove the unused Dioxus template assets (`assets/styling/*`, `assets/header.svg`).
