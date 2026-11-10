# Mico in MuJoCo — Physics Simulation & RViz2 on macOS

MuJoCo runs natively on Apple Silicon macOS, ROS 2 Jazzy is provided via RoboStack (conda), and RViz2 runs natively. Both physics setups subscribe to identical `/joint_trajectory_controller/joint_trajectory` topics, allowing the same `mico_gait.py` script to run seamlessly on both.

```text
mico_gait.py ──JointTrajectory──▶ mico_mujoco_bridge.py ──/clock /joint_states /tf /odom /imu──▶ robot_state_publisher, RViz2
                                   (MuJoCo, 1 ms substeps)
```

The physics core is located at `src/humanoid_control/scripts/mico_mujoco.py`, shared between the ROS 2 bridge and standalone simulation scripts:

| Component | Model Representation |
| --- | --- |
| Kinematics, Limits, Damping | Loaded directly from URDF (`mico.urdf.xacro`, `look:=mico camera:=false mass:=sim`) |
| Visuals | Blender meshes `meshes/visual/mico` (geom group 1); collision boxes in group 0, floor in group 2 |
| Servo Actuation | `τ = kp (q* − q) − kv q̇`, clipped at stall torque = `effort` in URDF; `kp` = 6 N·m/rad, `kv` = 0.03, armature = 2e-5 kg·m² |
| Contacts | Robot-to-floor and foot-to-foot contacts |
| IMU | Pelvis site on `imu_link`: accelerometer, gyroscope, quaternion orientation |
| Trajectory Interpolation | Linear time interpolation matching `joint_trajectory_controller`; stamp 0 executes immediately |

## Running Simulation

```zsh
./setup_macos.sh                                     # One-time setup: ros2_jazzy env + MuJoCo, builds to .mac/

conda activate ros2_jazzy && source .mac/install/setup.zsh
ros2 launch humanoid_description mico_mujoco.launch.py gait:=true      # Physics + RViz2 + gait demo
ros2 launch humanoid_description mico_mujoco.launch.py viewer:=true    # Adds MuJoCo viewer (launched via mjpython)
ros2 launch humanoid_description mico_view.launch.py                   # Kinematic playback without physics
```

| Parameter `mico_mujoco.launch.py` | Default | Description |
| --- | --- | --- |
| `gait` | `false` | Launch `mico_gait.py` (sends trajectory after 2 s) |
| `rviz` | `true` | Open RViz2 with `rviz/mico.rviz` (fixed frame `odom`, 5 cm grid) |
| `viewer` | `false` | Open interactive MuJoCo viewer (automatically invokes `mjpython` on macOS) |
| `look` | `mico` | `mico`: Blender outer shell, `boxes`: collision servo boxes |
| `mass` | `sim` | `sim`: tuned gait simulation mass, `blender`: full CAD shell mass budget |

Topics: `/clock`, `/joint_states` (position, velocity, effort), `/tf` (`odom → base_link`), `/odom`, `/imu`.
Commands: `/joint_trajectory_controller/joint_trajectory`, or 20 joint angles on `/position_controller/commands`.

Without ROS (Standalone Python):

```zsh
python3 sim/mujoco/check_mujoco.py                   # Headless verification: balance check, distance, heading, loads
mjpython sim/mujoco/mico_viewer.py --physics         # Interactive viewer with physics
mjpython sim/mujoco/mico_viewer.py                   # Kinematic playback (standalone)
python3 sim/mujoco/mico_viewer.py --png mico.png     # Render standing/walking/turn/wave snapshots
python3 sim/mujoco/mico_mjcf.py                      # Regenerate mico.xml after modifying gen_robot.py
```

`mico.xml` is a pre-generated MJCF file (relative mesh paths to `src/humanoid_description`), runnable directly via `pip install mujoco` without ROS 2 installed.

## Validation Results (MuJoCo 3.9+, macOS, 1 ms step)

- `check_mujoco.py` full gait evaluation: zero falls, forward distance 186 mm, lateral drift +11 mm, yaw turn +62.7° (target 60°), maximum base tilt 5.6°, minimum pelvis height 111 mm.
- Servo loading (P99 peak vs stall torque): hip roll 35%, knee 29%, ankle pitch 29%.
- Robustness across gain sweeps: `kp` in 3–12, `kv` in 0.01–0.1, armature in 0–1e-4 maintain stable dynamic balance and 61–63° yaw turns.
- ROS 2 execution (`mico_mujoco.launch.py gait:=true`): `/joint_states` published at ~100 Hz at 1.0× real-time; stationary IMU measures +9.79 m/s² on the z-axis.
