#!/bin/bash
# One-time setup of the Jetson Nano (JetPack 4.6) for this repo. Run it as yourself: it asks
# for sudo where it has to, and skips whatever is already done.
#   - checks for JetPack 4 (L4T r32) and Docker's NVIDIA runtime
#   - lets you run Docker without sudo
#   - adds a 4 GB swap file, the usual advice for compiling on a Nano, whose 4 GB of memory
#     is shared with the GPU and the desktop
#   - gives the arm's and the mirror's USB serial ports stable names (ros2/udev)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

if ! grep -q '^# R32 ' /etc/nv_tegra_release 2>/dev/null; then
    echo "This is for a Jetson on JetPack 4 (L4T r32). /etc/nv_tegra_release says:" >&2
    cat /etc/nv_tegra_release >&2 2>/dev/null || echo "(missing, so this isn't a Jetson)" >&2
    exit 1
fi
echo "L4T: $(head -n1 /etc/nv_tegra_release)"

if ! command -v docker >/dev/null || ! sudo docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
    echo "Docker or its NVIDIA runtime is missing. JetPack's SD card image has both; to add them:" >&2
    echo "  sudo apt-get update && sudo apt-get install nvidia-jetpack" >&2
    exit 1
fi
echo "Docker: $(sudo docker version --format '{{.Server.Version}}'), with the NVIDIA runtime"

if id -nG "$USER" | grep -qw docker; then
    echo "Docker without sudo: already set up"
else
    sudo usermod -aG docker "$USER"
    echo "Docker without sudo: added $USER to the docker group. Log out and back in for it to apply."
fi

if [ -f /swapfile ]; then
    echo "Swap file: already there"
else
    sudo fallocate -l 4G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile >/dev/null
    sudo swapon /swapfile
    grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
    echo "Swap file: added 4 GB at /swapfile"
fi

RULES="$REPO/ros2/udev/99-so101-scan.rules"
if cmp -s "$RULES" /etc/udev/rules.d/99-so101-scan.rules; then
    echo "USB port names: already installed"
else
    sudo cp "$RULES" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "USB port names: installed; with the arm and the ESP32 plugged in, ls -l /dev/so101 /dev/scan_mirror"
fi

echo "Power mode: $(sudo nvpmodel -q 2>/dev/null | grep -m1 'NV Power Mode' || echo unknown)"
echo "Done. Next: jetson/build_all.sh"
