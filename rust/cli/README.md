# pennair CLI

`pennair` checks and installs the dependencies listed in `monorepo.toml`, and pushes onboard source to Pis so they can build it themselves.

You run every command from your laptop. Commands that take a host run their work on the Pi over ssh, so the Pi doesn't need the `pennair` binary.

## Building

```bash
cd rust/cli
cargo build --release        # binary at target/release/pennair
cargo install-local          # or install it to ~/.cargo/bin/pennair
```

The old ROS CLI in `ros_ws/src/common/pennair_cli` also installs a command named `pennair`. If both are on your `PATH`, the one in the earlier directory runs.

## Global option

`--manifest <path>` sets which `monorepo.toml` to use. Without it, `pennair` reads `$PENNAIR_MONOREPO_PATH/monorepo.toml`. `push` and `pull` treat the manifest's directory as the monorepo root, because `[deploy]` paths are relative to it.

## Commands

### `pennair config check`

Fails if `monorepo.toml` doesn't parse, or if a path in `[deploy]` doesn't exist. CI runs this on every change to `rust/` or `monorepo.toml`.

### `pennair deps check --dev | --prod [--host <ssh target>]`

Fails if anything the chosen environment needs isn't installed. `--prod` checks the `base` and `prod` groups, and `--dev` checks `base` and `dev`. You must pass one of them.

It checks three kinds of dependencies:

- apt packages, using `dpkg-query`
- Python packages, using `importlib.metadata` from `python3`. It checks only that each package is installed, not its version.
- shared libraries, using `ldconfig -p`. A name like `libmicroxrcedds_agent.so.2.4` only matches version 2.4, so listing the versioned name also checks the version.

`--host` runs the check on another machine over ssh, for example `--host penn@air-03.local`.

### `pennair deps sync --dev | --prod [--host <ssh target>]`

Installs what `deps check` reports as missing, then checks again. If nothing is missing, it stops after the check, and it doesn't need internet or sudo.

1. If a library is missing, it stops and tells you to install it by hand, because the listed libraries are built from source.
2. It installs missing apt packages with `sudo apt-get install`. sudo asks for the password in your terminal.
3. It installs missing Python packages with `pip install --user`. It pins numpy to the version apt installed, because pip would otherwise install numpy 2, which breaks ROS.

`--host` works the same as in `deps check`.

### `pennair push <ssh target>... [--no-restart]`

Copies the `[deploy]` paths to `~/monorepo` on each Pi, builds them there, and restarts `pennair-autonomy.service`. It handles one Pi at a time and stops at the first error.

For each Pi it:

1. Runs `deps check --prod` on the Pi. Push doesn't install anything, so run `deps sync` first if the check fails.
2. Sets the Pi's clock to your laptop's time. Pis have no hardware clock, and make compares file times to decide what to rebuild. This needs passwordless sudo. Without it, push prints a warning and continues.
3. Copies the `[deploy]` paths with `rsync -a --delete`, keeping file times, so unchanged packages don't rebuild. It skips `.git` and `__pycache__`.
4. Saves a push record in `~/monorepo/.pennair/push` (your commit, your name, the time) and a hash of every pushed file in `~/monorepo/.pennair/push.sha256`.
5. Runs `colcon build` in `~/monorepo/ros_ws`, limited to the pushed packages, with tests turned off.
6. Restarts `pennair-autonomy.service` if the build passed. `--no-restart` skips this step.

A push with no changes takes about 40 seconds on a Pi. The first push takes several minutes, mostly for `px4_msgs`.

### `pennair pull <ssh target> [--force]`

Copies the `[deploy]` paths from the Pi back into your checkout, then shows `git status` for them, so edits made on the Pi show up as a normal git diff.

Pull overwrites those paths, so it refuses if you have uncommitted changes in them. Commit or stash first. `--force` pulls anyway and overwrites your changes.

### `pennair status <ssh target>...`

Prints each Pi's push record, then lists pushed files that were edited or deleted on the Pi since the last push. It doesn't list files that were created on the Pi. `pull` still copies those back.

## What's on a Pi

```
~/monorepo/
  monorepo.toml
  ros_ws/src/...            pushed packages, same paths as in the repo
  ros_ws/build, install/    colcon output
  .pennair/push             last push record
  .pennair/push.sha256      hashes of the pushed files
```

The systemd unit from `ros_ws/scripts/hardware/bootstrap-pi.sh` runs the code built in `~/monorepo/ros_ws/install`.

## Pi requirements

- Your ssh key is in the Pi's `authorized_keys`.
- Passwordless sudo, so push can set the clock and restart the service. `provision-pi.sh` sets this up.
- Internet, only for `deps sync` when something is missing.
