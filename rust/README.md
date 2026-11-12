# Rust workspace

Each package declares its own dependencies. The workspace shares `Cargo.lock` and the `target/` build directory.

## GCS development

Install the Dioxus CLI (`dx`) and the system dependencies required by Dioxus desktop before building. The current project uses Dioxus 0.7; use a compatible CLI version.

```bash
# Build and launch GCS with file watching and hot reload
dx serve -p gcs

# Build GCS and process its assets without starting the development server
dx build -p gcs

# Build an optimized GCS
dx build -p gcs --release
```

Alternatively, run `dx serve` directly from `rust/gcs/`.


## CLI

```bash
cargo build -p cli
cargo build -p cli --release

# Install the pennair executable into Cargo's bin directory
cargo install --path cli --locked
```

The package is named `cli`; the executable is named `pennair`. Regular build output is `target/debug/pennair` or `target/release/pennair`.

From `rust/cli/`, plain `cargo build` builds only the CLI and its dependencies.

## CLI cross-compilation (x86-64 → ARM64)

Requires Docker or Podman. Run from `rust/`:

```bash
# Install cross once
cargo install cross --git https://github.com/cross-rs/cross --locked

cross build -p cli --release --locked --target aarch64-unknown-linux-gnu
```

Output: `target/aarch64-unknown-linux-gnu/release/pennair`.

## Checks and tests

```bash
# Check every workspace package without producing final executables
cargo check --workspace

# Check only one package
cargo check --package cli
cargo check --package gcs

# Format all workspace source files
cargo fmt --all

# Check formatting without modifying files
cargo fmt --all -- --check

# Lint all packages, including tests and examples
cargo clippy --workspace --all-targets

# Run Rust tests across the workspace
cargo test --workspace
```

Plain `cargo build` from `rust/` builds all workspace members, including GCS. Prefer the package-specific CLI build and the Dioxus commands above tho

## Regenerate ROS interfaces

Run these commands from `rust/` in an environment with ROS and its Python IDL tooling available. Source your installed ROS distribution first (for example, `source /opt/ros/jazzy/setup.bash`).

```bash
# Regenerate from ros_interfaces/idl/
python3 ros_interfaces/generate_rs.py

# Run generator regression tests, including generated Rust compilation tests
python3 ros_interfaces/tests/test_generate_rs.py
```

The generator replaces `ros_interfaces/src/` and rewrites its `Cargo.toml`. Edit the IDLs or generator rather than the generated Rust files.
