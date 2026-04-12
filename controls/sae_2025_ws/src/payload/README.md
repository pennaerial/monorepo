# Payload Package

Payload runs the SAE ground payload in simulation and on Raspberry Pi hardware.

Payload for SAE Advanced Class 2026:
- two wheel diff drive payload
- front-mounted camera
- rear ball caster

## Parameters And Controllers

`payload` generates `payload/payload_parameters.hpp` from `config/payload_params.schema.yaml` at build time using `generate_parameter_library`.

If you are building manually without `rosdep`, install the generator first:

```bash
sudo apt-get install ros-humble-generate-parameter-library
```

Available controller plugin names:
- `SimController` for Gazebo-backed simulation
- `GPIOController` for Raspberry Pi hardware via `pigpio`

`config/payload_params.schema.yaml` defines the supported parameter surface, and `config/payload_params.yaml` provides the shipped runtime values. `payload_params.yaml` currently defaults `controller` to `GPIOController`. Simulation launch flows override that to `SimController`, or you can do it manually with `controller:=SimController`.

## Raspberry Pi Hardware Setup (`pigpiod`)

The `GPIOController` backend and the hardware test binaries (`test_motor`, `test_servo`) require the `pigpiod_if2` headers/library at build time and a running `pigpiod` daemon at runtime. This must be set up on every Pi before running the payload node in hardware mode.

### 1. Install pigpio from source

`pigpio` is not available via `apt` on Ubuntu 22.04. Build and install from source:

```bash
cd ~
git clone --depth 1 https://github.com/joan2937/pigpio.git
cd pigpio
cmake . -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2. Create the systemd service

```bash
sudo tee /etc/systemd/system/pigpiod.service > /dev/null << 'EOF'
[Unit]
Description=Daemon required to control GPIO pins via pigpio

[Service]
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill pigpiod
Type=forking

[Install]
WantedBy=multi-user.target
EOF
```

### 3. Enable and start the daemon

```bash
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
sudo systemctl status pigpiod
```

Look for `Active: active (running)`. The daemon will now start automatically on boot.

### 4. Verify

```bash
source install/setup.bash
ros2 run payload test_servo
```

`test_servo` should run without `sudo` and without `pigpio_start() failed`. `test_motor` uses the same pigpio runtime path when you are ready to validate motors on hardware.

#### Each-Session Setup

After the one-time setup, `pigpiod` should start automatically on boot. A normal hardware bringup looks like:

```bash
ssh penn@penn-desktop.local
cd ~/monorepo/controls/sae_2025_ws
python3 -m pip install --user "pydantic>=2,<3"
source install/setup.bash
ros2 launch uav main.launch.py params_file:=$(pwd)/src/uav/launch/launch_params_hardware.yaml
```

If `pigpiod` ever stops, restart it with:

```bash
sudo systemctl restart pigpiod
```

## Launch

`payload.launch.py` accepts these launch arguments:
- `vehicle_name`: payload ROS namespace prefix. In simulation, this should match the vehicle identity used by the orchestration layer. Default: `payload_0`
- `controller`: optional plugin override. Leave it empty to use `config/payload_params.yaml`

Examples:

```bash
# Hardware / default config
ros2 launch payload payload.launch.py

# Standalone simulation
ros2 launch payload payload.launch.py controller:=SimController

# Alternate vehicle namespace in simulation
ros2 launch payload payload.launch.py vehicle_name:=payload_1 controller:=SimController
```

Standalone `payload.launch.py` loads the installed `config/payload_params.yaml` from the package share directory. It launches payload ROS topics and services under `/<vehicle_name>/...` so multi-payload orchestration can target each payload independently. If you launch payload through `uav main.launch.py`, the UAV launch stack handles the mission and camera wiring. On hardware, make sure `pigpiod` is already running first.
