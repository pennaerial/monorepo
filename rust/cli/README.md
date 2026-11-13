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
