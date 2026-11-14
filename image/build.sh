#!/usr/bin/env bash
# Builds the Pi base image from the official Ubuntu 24.04 server image.
# Run as root on arm64 Ubuntu:
#   image/build.sh <pennair binary> <output.img.xz>
set -euo pipefail

UBUNTU="ubuntu-24.04.5-preinstalled-server-arm64+raspi.img"
UBUNTU_SHA256="b23371a5c8d02f612c26e7c18cacb81c5a48f968c7c66a2645391320aa94351e"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PENNAIR="$(realpath "$1")"
OUT="$(realpath "$2")"

work="$(mktemp -d)"
cd "$work"

curl -fL -o "$UBUNTU.xz" "https://cdimage.ubuntu.com/releases/24.04/release/$UBUNTU.xz"
echo "$UBUNTU_SHA256  $UBUNTU.xz" | sha256sum -c
xz -d "$UBUNTU.xz"

# The stock image has almost no free space, so grow it to fit ROS and the build tools.
truncate -s +6G "$UBUNTU"
loop="$(losetup --find --show --partscan "$UBUNTU")"
growpart "$loop" 2
e2fsck -fy "${loop}p2"
resize2fs "${loop}p2"

root="$work/root"
mkdir "$root"
mount "${loop}p2" "$root"
# the stock root filesystem doesn't have this folder until first boot
mkdir -p "$root/boot/firmware"
mount "${loop}p1" "$root/boot/firmware"
for dir in dev dev/pts proc sys; do
	mount --bind "/$dir" "$root/$dir"
done

# DNS for the chroot. The image's resolv.conf is a symlink to this path.
mkdir -p "$root/run/systemd/resolve"
cp /etc/resolv.conf "$root/run/systemd/resolve/stub-resolv.conf"

# Stop apt from starting services inside the chroot.
printf '#!/bin/sh\nexit 101\n' >"$root/usr/sbin/policy-rc.d"
chmod +x "$root/usr/sbin/policy-rc.d"

# The stock image only lists noble and noble-security until cloud-init's first boot,
# but it ships packages from noble-updates, so apt needs that suite to install anything.
: >"$root/etc/apt/sources.list"
cat >"$root/etc/apt/sources.list.d/ubuntu.sources" <<'SOURCES'
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports
Suites: noble noble-updates noble-backports noble-security
Components: main universe restricted multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
SOURCES

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
	gpg --dearmor -o "$root/usr/share/keyrings/ros2-archive-keyring.gpg"
echo "deb [signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
	>"$root/etc/apt/sources.list.d/ros2.list"

install -m 755 "$PENNAIR" "$root/usr/local/bin/pennair"
cp "$HERE/../monorepo.toml" "$root/tmp/monorepo.toml"

# Networking tools the image needs, then everything monorepo.toml lists for prod.
chroot "$root" /bin/bash -e <<'EOF'
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y network-manager avahi-daemon dnsmasq iw batctl
pennair --manifest /tmp/monorepo.toml deps sync --prod
apt-get clean
EOF

cp -a "$HERE/rootfs/." "$root/"

# Free the serial port for PX4, and turn on the camera.
sed -i 's/console=serial0,115200 //' "$root/boot/firmware/cmdline.txt"
cat >>"$root/boot/firmware/config.txt" <<'EOF'

# pennair
enable_uart=1
dtoverlay=disable-bt
camera_auto_detect=0
dtoverlay=imx219
EOF

rm -rf "$root/usr/sbin/policy-rc.d" "$root/run/systemd" "$root/root/.cache" \
	"$root"/tmp/* "$root"/var/lib/apt/lists/*
# Deleted files leave their data in the free space. Trimming zeroes it so it compresses away.
fstrim "$root"
umount -R "$root"
losetup -d "$loop"

xz -T0 -c "$UBUNTU" >"$OUT"
(cd "$(dirname "$OUT")" && sha256sum "$(basename "$OUT")" >"$OUT.sha256")
ls -lh "$OUT"
