#!/usr/bin/env bash
set -euo pipefail

# Installer for system and Python dependencies used by the workspace
# Places: workspace root: install_dependencies.sh
# Usage: sudo ./install_dependencies.sh [-d DISTRO]

DISTRO_DEFAULT="jazzy"
DISTRO="${1:-${ROS_DISTRO:-$DISTRO_DEFAULT}}"

echo "Using ROS distro: $DISTRO"

if [ "$(id -u)" -ne 0 ]; then
  echo "This script needs sudo for apt and rosdep. Re-run with sudo or as root."
  exit 1
fi

apt update

echo "Installing rosdep, colcon and common build tools..."
apt install -y python3-rosdep python3-colcon-common-extensions build-essential git curl

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep not found after install. Aborting."
  exit 1
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi
rosdep update || true

WORKSPACE_DIR="$(pwd)"
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Run this script from the workspace root (the folder that contains src/)."
  exit 1
fi

echo "Running rosdep to install package system dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true

echo "Attempting to install common ROS packages via apt (best-effort)..."
apt install -y \
  ros-${DISTRO}-moveit \
  ros-${DISTRO}-moveit-ros \
  ros-${DISTRO}-moveit-ros-planning \
  ros-${DISTRO}-tf2-ros \
  ros-${DISTRO}-ros2-control \
  ros-${DISTRO}-ros2-controllers \
  ros-${DISTRO}-robot-state-publisher \
  ros-${DISTRO}-xacro \
  ros-${DISTRO}-rviz2 \
  ros-${DISTRO}-ros-gz-bridge \
  ros-${DISTRO}-ros-gz || true

echo "Installing Python requirements (if present)..."
if [ -f requirements.txt ]; then
  pip3 install -r requirements.txt
else
  echo "No requirements.txt found at workspace root; creating minimal requirements.txt"
  cat > requirements.txt <<'EOF'
setuptools
EOF
  pip3 install -r requirements.txt
fi

echo "Done. You may still need to install distribution-specific packages manually if apt failed for some names."
echo "Recommended next steps:"
echo "  1) source /opt/ros/$DISTRO/setup.bash"
echo "  2) cd $WORKSPACE_DIR && rosdep install --from-paths src --ignore-src -r -y"
echo "  3) colcon build --symlink-install"

exit 0
