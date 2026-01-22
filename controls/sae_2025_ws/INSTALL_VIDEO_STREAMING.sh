#!/bin/bash
# Install GStreamer and dependencies for video streaming to QGroundControl
# Run this on your Ubuntu VM

set -e

echo "=========================================="
echo "Installing Video Streaming Dependencies"
echo "=========================================="

# Update package list
echo "Updating package list..."
sudo apt update

# Install GStreamer core and plugins
echo "Installing GStreamer..."
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-x \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

# Install Python GStreamer bindings
echo "Installing Python GStreamer bindings..."
sudo apt install -y \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0

# Install cv_bridge if not already installed
echo "Installing ROS 2 cv_bridge..."
sudo apt install -y ros-humble-cv-bridge

# Verify installation
echo ""
echo "=========================================="
echo "Verifying Installation"
echo "=========================================="

# Check GStreamer version
echo "GStreamer version:"
gst-launch-1.0 --version

# Check Python bindings
echo ""
echo "Testing Python GStreamer bindings..."
python3 -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; print('✓ Python GStreamer bindings OK')" || echo "✗ Python GStreamer bindings FAILED"

# Check cv_bridge
echo ""
echo "Testing cv_bridge..."
python3 -c "from cv_bridge import CvBridge; print('✓ cv_bridge OK')" || echo "✗ cv_bridge FAILED"

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. cd ~/Desktop/PennAiR/monorepo/controls/sae_2025_ws"
echo "2. colcon build --packages-select uav"
echo "3. source install/setup.bash"
echo "4. ros2 launch uav main.launch.py"
echo ""
echo "Make sure to configure QGroundControl:"
echo "- Video Source: UDP h.264 Video Stream"
echo "- Port: 5600"
echo ""
