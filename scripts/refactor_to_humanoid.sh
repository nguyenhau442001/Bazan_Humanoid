#!/usr/bin/env bash
# ==============================================================================
# Synapse Humanoid - Safe Refactoring & Cleanup Script
# Transforms legacy mobile robot repository into Synapse Humanoid stack.
# ==============================================================================
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_ROOT}"

echo ">>> [1/4] Cleaning legacy wheeled robot packages..."
LEGACY_DIRS=(
  "mobile_robot"
  "mobile_robot_control"
  "mobile_robot_custom_nav"
  "mobile_robot_description"
  "mobile_robot_digital_twin"
  "mobile_robot_gazebo"
  "mobile_robot_genesis"
  "mobile_robot_interfaces"
  "mobile_robot_monitor"
  "mobile_robot_mujoco"
  "mobile_robot_multi"
  "mobile_robot_navigation2"
  "mobile_robot_slam"
  "mobile_robot_teleop"
  "mobile_robot_web"
  "docs"
)

for dir in "${LEGACY_DIRS[@]}"; do
  if [ -d "$dir" ]; then
    echo "  Removing: $dir"
    rm -rf "$dir"
  fi
done

echo ">>> [2/4] Verifying src/ packages for Synapse Humanoid..."
PACKAGES=(
  "src/humanoid_description"
  "src/humanoid_control"
  "src/humanoid_perception"
  "src/humanoid_infrastructure"
  "src/humanoid_interfaces"
)

for pkg in "${PACKAGES[@]}"; do
  if [ -d "$pkg" ]; then
    echo "  Found: $pkg [OK]"
  else
    echo "  Warning: $pkg missing!"
  fi
done

echo ">>> [3/4] Updating Git tracking..."
git add -A

echo ">>> [4/4] Clean up complete! Run 'colcon build' to verify."
