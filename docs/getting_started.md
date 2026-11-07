# Getting Started with Synapse Humanoid

This guide covers environment setup, running the MuJoCo physics simulation, kinematics verification, and running tests.

---

## 🚀 Quick Setup Overview

Physics runs in MuJoCo. The bridge publishes `/joint_states`, `/tf`, `/odom`, `/imu`, `/clock` and subscribes to
`/joint_trajectory_controller/joint_trajectory`, the topic the gait and the RL policy send to:

| Feature | MuJoCo |
| :--- | :--- |
| **Execution** | Natively on macOS (RoboStack Conda env) |
| **Visualization** | RViz2 on macOS, native MuJoCo viewer |
| **Bridge** | `mico_mujoco_bridge.py` (rclpy, 1 ms steps) |
| **Gait Demo** | Stays upright, travels 186 mm, +62.7° |

---

## 🍎 1. MuJoCo + RViz2 (Native macOS)

### Initial Setup
Run once to create the `ros2_jazzy` conda environment and compile the Mac workspace into `.mac/`:
```bash
./setup_macos.sh
```

### Launching Simulation & Visualization
In every new terminal:
```bash
# 1. Activate environment
conda activate ros2_jazzy && source .mac/install/setup.zsh

# 2. Launch MuJoCo bridge with 8-step walking demo in RViz2
ros2 launch humanoid_description mico_mujoco.launch.py gait:=true

# 3. (Optional) Open native MuJoCo interactive viewer
ros2 launch humanoid_description mico_mujoco.launch.py viewer:=true

# 4. (Optional) Kinematic playback without physics
ros2 launch humanoid_description mico_view.launch.py
```

### Physics Checks Without ROS
```bash
python3 sim/mujoco/check_mujoco.py             # physics verification: distance, heading, servo loads
mjpython sim/mujoco/mico_viewer.py --physics   # interactive simulation in MuJoCo viewer
```

---

## 📐 2. Kinematics and Load Tables
```bash
python3 sim/mujoco/check_mujoco.py       # gait in MuJoCo: fell or not, distance, heading, writes servo loads
python3 sim/mini_humanoid/kin_dyn.py     # check DH tables against URDF, static loads, merge the MuJoCo loads
```

---

## 🧪 Testing & Verification

Run the full pytest suite for kinematics and bridge physics:
```bash
python3 -m pytest src/humanoid_control/test
```
Validate the URDF and xacro (links, joints, limits, frames, gait joints):
```bash
python3 sim/mini_humanoid/check_urdf.py
```
