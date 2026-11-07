#!/usr/bin/env bash
# Nuclear cleanup script for Mico ROS 2 and MuJoCo simulation.
# Forcefully kills all ROS 2 nodes, launch parents, bridge, rviz, and resets daemons.
#
# Usage:
#   ./kill_fresh.sh

set -euo pipefail

echo "================================================================="
echo "  [NUCLEAR CLEANUP] TERMINATING ALL SIMULATION & ROS 2 PROCESSES"
echo "================================================================="

# Target process patterns to purge completely
PATTERNS=(
  "mico_mujoco_bridge"
  "mico_rl_node"
  "mico_gait"
  "mico_playback"
  "robot_state_publisher"
  "rviz2"
  "mjpython"
  "ros2 launch"
  "ros2 topic pub"
  "ros2-daemon"
)

# Force-kill everything matching target patterns
for pat in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pat" 2>/dev/null || true)
  if [ -n "$pids" ]; then
    echo "[-] Force-killing $pat (PIDs: $pids)..."
    kill -9 $pids 2>/dev/null || true
  fi
done

# Stop ros2 daemon cleanly if ros2 is available
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

# Clean up FastDDS shared memory segments on macOS
echo "[-] Purging shared memory segments..."
rm -f /tmp/boost_interprocess* /tmp/fastdds* /tmp/shm* 2>/dev/null || true

# Verification
sleep 0.3
REMAINING=$(pgrep -f "mico_|rviz2|robot_state_publisher|ros2 launch" 2>/dev/null || true)
if [ -n "$REMAINING" ]; then
  echo "[!] Warning: Stubborn PIDs still alive: $REMAINING. Killing with extreme prejudice..."
  kill -9 $REMAINING 2>/dev/null || true
fi

echo "================================================================="
echo "  [DONE] SYSTEM IS 100% CLEAN. NO LINGERING DAEMONS OR PROCESSES."
echo "================================================================="
echo ""
echo "Next run in 2 clean terminals:"
echo ""
echo "  [Terminal 1]:"
echo "    conda activate ros2_jazzy && source .mac/install/setup.zsh"
echo "    ros2 launch humanoid_description mico_mujoco.launch.py rl:=true viewer:=true"
echo ""
echo "  [Terminal 2]:"
echo "    conda activate ros2_jazzy && source .mac/install/setup.zsh"
echo "    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\" -1"
echo "================================================================="
