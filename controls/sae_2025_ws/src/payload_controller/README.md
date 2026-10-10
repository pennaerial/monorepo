# Payload Package

Payload runs the SAE ground payload in simulation and on Raspberry Pi hardware.

Payload for SAE Advanced Class 2026:
- two wheel diff drive payload
- front-mounted camera
- rear ball caster

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

If `pigpiod` ever stops, restart it with:

```bash
sudo systemctl restart pigpiod
```
