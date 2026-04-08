# Payload Package
Payload node responsible for receiving and converting drive commands to movement (in real and sim)

Payload for SAE Advanced Class 2026
- two wheel diff drive payload
- cam mounted in front
- ball caster in back

### Extra Dependencies
`sudo apt install ros-humble-generate-parameter-library`
- uses `generate_parameter_library` to load in node parameters as C++ structs

### Raspberry Pi Hardware Setup (pigpiod)

The GPIO controller backend uses the `pigpiod` daemon (pigpiod_if2 library) for hardware PWM and encoder interrupts. This must be set up on **every Pi** before running the payload node in hardware mode.

#### 1. Install pigpio from source
`pigpio` is not available via `apt` on Ubuntu 22.04. Build and install from source:

```bash
cd ~
git clone https://github.com/joan2937/pigpio.git
cd pigpio
make
sudo make install
sudo ldconfig
```

#### 2. Create the systemd service
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

#### 3. Enable and start the daemon
```bash
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
sudo systemctl status pigpiod
```

Look for `Active: active (running)`. The daemon will now start automatically on boot.

#### 4. Verify
```bash
source install/setup.bash
ros2 run payload test_servo
```

This should run without `sudo` and without errors. If you see `pigpio_start() failed`, check that pigpiod is running: `sudo systemctl status pigpiod`.

#### Each-session setup (after one-time setup is done)

pigpiod is enabled as a systemd service and starts automatically on boot — no manual steps are needed each session. Just SSH in and run your launch file normally:

```bash
ssh penn@penn-desktop.local
cd ~/monorepo/controls/sae_2025_ws
source install/setup.bash
ros2 launch uav main.launch.py params_file:=$(pwd)/src/uav/launch/launch_params_hardware.yaml
```

If the Pi was not rebooted and pigpiod somehow stopped, restart it with:

```bash
sudo systemctl restart pigpiod
```


If running multiple instances of payload, they should differ by launch arguments, but share the same node parameters

### Launch configuration:
The defined launch arguments are:
- `payload_name`: node name of the payload to launch. The node name is used to determine the name of the ros topics that the payload node listens to, and also the gazebo topics/services that the node sends messages to (sim controller). **This must match the name of the entity name of the gazebo payload model (sim mode)** \
DEFAULT VALUE: `payload_0`

### Payload Parameters
Defined payload parameters (defined in `config/payload_params.schema.yaml`):
- controller: defines the controller to use. The available options are `sim` and `gpio`. Use `sim` when testing in sitl and `gpio` when running on the pi \
DEFAULT VALUE: `sim`

### Launch:
Running the launch file will run one instance of the payload and also links payload_params
from ws directory, run 

```bash
ros2 launch payload payload.launch.py
```
- this will launch the payload node with the params found in `config/payload_params.yaml`
- this will also use the default launch configuration

You can override the launch configuration via command line \
Example: overriding the `payload_name` launch argument:
```bash
ros2 launch payload payload.launch.py payload_name:=payload_1
```
- This will launch a payload node called with name `payload_1`
- This will listen to a ros topic called `/payload_1/cmd_drive` for drive commands
- This will publish drive commands to a gazebo topic called `/model/payload_1/cmd_vel` to control the sim payload 



