# Flight Controller and Pi

This guide covers the **Raspberry Pi 4B and Compute Module 4 (CM4)**. Our
[CM4 base board](https://www.waveshare.com/wiki/CM4-NANO-B) exposes the same
**40-pin GPIO layout as the Pi 4B**, so the wiring below applies to both.

Both setups use two UART connections to the flight controller. Use your flight
controller's pinout to find TX, RX, and ground on two separate serial ports.

## Connections

MicroXRCEAgent connects the flight controller to ROS 2 on the Pi. MAVProxy
forwards MAVLink between the flight controller and QGC over Wi-Fi, and can also
pass flight logs to a logger on the Pi.

| Connection | Pi device in our Ubuntu 24.04 setup | Pi process | Launch baud rate |
| --- | --- | --- | --- |
| ROS 2 messages over XRCE-DDS | UART0: `/dev/ttyAMA0` | `MicroXRCEAgent` | 921600 |
| MAVLink for QGC and optional ULog collection | UART5: `/dev/ttyAMA5` | `mavproxy.py` | 921600 |

Both ends of each UART must use the same baud rate and protocol. Set the matching
flight-controller port to **921600** as well, and reboot after changing its baud
rate.

## Pi pin connections

**Physical pin numbers are positions on the header, not GPIO numbers.**
For example, physical pin 32 is GPIO12.

```{figure} ../../_static/raspberry-pi-40-pin-pinout.png
:alt: 40-pin GPIO layout shared by the Pi 4B and our CM4 base board, showing physical pin numbers, GPIO labels, power, and ground.
:width: 100%

40-pin GPIO layout used by the Pi 4B and our CM4 base board. Image: [Raspberry Pi](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio), [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
```

| Connection | Pi physical pin | Pi signal | Connect to flight controller |
| --- | --- | --- | --- |
| XRCE-DDS | **8** | GPIO14 / UART0 TX | RX on the XRCE-DDS port |
| XRCE-DDS | **10** | GPIO15 / UART0 RX | TX on the XRCE-DDS port |
| XRCE-DDS | **9** | Ground | Ground |
| MAVLink | **32** | GPIO12 / UART5 TX | RX on the MAVLink port |
| MAVLink | **33** | GPIO13 / UART5 RX | TX on the MAVLink port |
| MAVLink | **34** | Ground | Ground |

The image labels GPIO12/13 as PWM. In our setup, enabling UART5 uses these
pins for serial TX/RX, as shown in the table.

For each UART, cross **TX to RX**, connect a common ground, and use **3.3 V
signals**. Leave the flight controller's UART power and CTS/RTS pins unconnected.

`/dev/ttyAMA0` and `/dev/ttyAMA5` are Linux device names for the Pi's UARTs, not
names assigned to the attached flight controller. On our image, `/dev/serial0`
is an alias for `/dev/ttyAMA0`. Device numbering can differ on other images.

## Power

Power the Pi from the **LiPo battery through a 5 V step-down regulator**:

- Regulator **+5 V output → Pi physical pin 4**.
- Regulator **ground output → Pi physical pin 6**.

Check the regulator's output voltage and polarity before connecting it. Never
connect the LiPo directly to the Pi.

## Enable UART5

In `/boot/firmware/config.txt`, add this once under an active `[all]` section:

```ini
dtoverlay=uart5
```

This enables UART5 on GPIO12/13 (physical pins 32/33), selecting their ALT4
function. Use the plain overlay: adding `,ctsrts` would also use GPIO14/15,
which are already assigned to UART0. See the
[Raspberry Pi UART overlay reference](https://github.com/raspberrypi/linux/blob/rpi-6.6.y/arch/arm/boot/dts/overlays/README#L4689-L4692).

Keep the existing UART0 settings (`enable_uart=1` and `dtoverlay=disable-bt` on
our image), then reboot. The overlay enables hardware; it does not start
MAVProxy or set the baud rate.

After rebooting, check the device names:

```bash
ls -l /dev/serial0 /dev/ttyAMA*
```

Expect `/dev/serial0` to point to `/dev/ttyAMA0`, with `/dev/ttyAMA5` also present.
This confirms the device files exist; receiving a heartbeat in MAVProxy checks
the MAVLink connection to the powered flight controller.
