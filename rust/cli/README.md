# pennair CLI

```bash
cargo build --release   # target/release/pennair
```

`--manifest <path>`: manifest to use (default `$PENNAIR_MONOREPO_PATH/monorepo.toml`).

| Command | Description |
|---|---|
| `config check` | Check that `monorepo.toml` parses and `[deploy]` paths exist. |
| `deps check --dev\|--prod [--host <ssh target>]` | Check that the environment's apt packages, Python packages, and libraries are installed. |
| `deps sync --dev\|--prod [--host <ssh target>]` | Install missing apt and Python packages. |
| `push <ssh target>... [--no-restart]` | Check deps, rsync `[deploy]` paths to `~/monorepo` on each Pi, `colcon build`, restart `pennair-autonomy.service`. |
| `pull <ssh target> [--force]` | Rsync `[deploy]` paths from a Pi back into the checkout. Refuses on uncommitted changes unless `--force`. |
| `status <ssh target>...` | Show the last push and files edited on the Pi since. |
| `flash <id> <device> [--image <file>]` | Write the Pi image to an SD card as `air-<id>` (mesh `10.10.0.<id>`, Ethernet `192.168.50.<id>`). Linux only. |
| `gateway on\|off` | Share this laptop's internet with Pis over Ethernet and the mesh. Linux only. |
