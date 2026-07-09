# Installation & Build System

## Prerequisites

- [direnv](https://direnv.net/) — `payload_controller/.envrc` sources the ESP-IDF environment automatically when you `cd` into this directory
- Gazebo Transport/Msgs (`gz-transport13`, `gz-msgs10`) for the `linux` SITL target — the SITL drivers link against these via CMake's `find_package`

## Installing ESP-IDF

ESP-IDF v6.0.2 is installed via Espressif's IDE Installation Manager (`eim`), using the checked-in `eim_config.toml`:

```bash
echo "deb [trusted=yes] https://dl.espressif.com/dl/eim/apt/ stable main" | sudo tee /etc/apt/sources.list.d/espressif.list
sudo apt update
sudo apt install eim
cd payload_controller
eim install -c eim_config.toml
```

This installs the toolchain and ESP-IDF v6.0.2 to `~/.espressif` (per `eim_config.toml`'s `path`), and drops an activation script at `~/.espressif/tools/activate_idf_v6.0.2.sh`.

Once installed, allow direnv to load the environment: (you can also skip direnv setup and just manually source .envrc when you cd into payload_controller)

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc

direnv allow # in payload_controller directory
```

`.envrc` sources that activation script and puts `idf.py` on `PATH` — this happens automatically on `cd` from then on. It also re-prioritizes `/usr/bin` ahead of the ESP32 ULP toolchain's bin dir, since the ULP toolchain ships an unprefixed `as` that otherwise shadows the system assembler and breaks host (`linux` target) builds.

## Building

All esp targets are supported and can be built side by side through the `Makefile`, which is just a light wrapper around idf.py commands:

```bash
make esp32s3   # real hardware, for esp32s3 target
make linux     # SITL, runs on the host and talks to Gazebo
```

Each target gets its own out-of-tree build directory (`build/esp32s3/`, `build/linux/`) and its own generated `sdkconfig.<target>` file. On first run for a given target, the Makefile initializes that build directory with `idf.py set-target` (passing `--preview` for `linux`, since it's a preview IDF target); subsequent runs just build.

`sdkconfig.defaults` is checked in and applies to all targets. The per-target `sdkconfig.esp32s3`/`sdkconfig.linux` are also generated on build

## Component layout

The root `CMakeLists.txt` adds `src` and `src/drivers` as `EXTRA_COMPONENT_DIRS`, so ESP-IDF's component system discovers components there in addition to the default `main/`:

- **`main/`** — the app component (`payload_controller.cpp`, `app_main`). Depends on `imu` and `encoder`; also depends on `sitl` when building for `linux`.
- **`src/drivers/imu/`, `src/drivers/encoder/`** — driver components. Each defines a hardware-agnostic interface header, and conditionally compiles a `*_sitl.cpp` implementation only when `IDF_TARGET STREQUAL "linux"`, linking Gazebo Transport via `src/gz_deps.cmake` in that case. Real-hardware implementations are added the same way, gated on the real `IDF_TARGET`.
- **`src/sitl/`** — the `sitl` component (`linux`-only). Provides SITL runtime config (`SimConfig`, `get_config()`) — e.g. which Gazebo world/model to bind to — read from `GZ_MODEL`/`GZ_WORLD` env vars, consumed by the SITL drivers.
- **`src/state_estimator/`** — state estimation (EKF); currently a stub.
