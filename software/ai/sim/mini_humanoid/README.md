# Mini Humanoid 20 DoF (Mico) — Simulation Model v0.7

Box model constructed strictly from physical dimensions and mass budgets specified in mechanical design docs:
Height 303 mm, mass ~536 g, center of mass height ~138 mm. Actuators selected based on joint loads:

| Servo | Joint | Stall Torque (sim limit) |
| --- | --- | --- |
| TowerPro MG92B × 10 | hip roll, hip pitch, knee, ankle pitch, ankle roll | 3.5 kg·cm = 0.343 N·m |
| MG90S × 4 | hip yaw, shoulder pitch | 2.2 kg·cm = 0.216 N·m |
| SG90 × 6 | shoulder roll, elbow, neck yaw, head pitch | 1.8 kg·cm = 0.177 N·m |

Physics runs in **MuJoCo**, natively on macOS, viewed in **RViz2** (see [`sim/mujoco`](../mujoco/README.md)).

| File | Purpose |
| --- | --- |
| `gen_robot.py` | Single source of truth: edit dimensions/masses here to regenerate the URDF and the xacro |
| `humanoid.urdf` | Standalone URDF for RViz2 / ROS 2 (`robot_description`) |
| `../../src/humanoid_description/urdf/mico.urdf.xacro` | Full robot description (RViz2, MuJoCo): 20 joints, IMU, camera and ToF frames |
| `../../src/humanoid_control/scripts/mico_gait.py` | Demo trajectory: 8 steps → 60° left turn → wave (`JointTrajectory`) |
| `kin_dyn.py` | 5-chain DH table, FK verification against URDF, link tables, static loads → `tables/*.csv` |
| `check_urdf.py` | Cross-validates URDF, xacro and the gait joints (names, limits, frames) |

## Checks

```zsh
python3 sim/mujoco/check_mujoco.py        # ~36 s gait in MuJoCo, writes tables/mj_servo_loads.csv
python3 sim/mini_humanoid/kin_dyn.py      # merges the MuJoCo loads into tables/joints_kin_dyn.csv
python3 sim/mini_humanoid/check_urdf.py   # pure Python URDF/xacro validator
```

## Coordinate Frames

`gen_robot.py` generates both models from a unified kinematic tree, ensuring exact frame alignment (`check_urdf.py` verifies limits and frames).

Standard: REP-103 (x forward, y left, z up), link naming following REP-120. `base_link` is positioned at hip center H (115 mm above ground in default stance). Sensor frames:

| Frame | Parent Link | Position relative to parent (mm) | Notes |
| --- | --- | --- | --- |
| `imu_link` | `base_link` | 0, 0, 30 (145 mm above floor) | MPU6050 on pelvis top plate |
| `camera_link` → `camera_optical_frame` | `head_pitch_link` | 31, 0, 42, pitched 15° down | Optical frame: z forward, x right, y down |
| `tof_link` | `head_pitch_link` | 31, 8, 45, pitched 20° down | VL53L0X distance sensor |
| `l_mic_link`, `r_mic_link` | `head_pitch_link` | 2, ±38.5, 24 | Microphones facing laterally |
| `l_sole`, `r_sole` | `*_ankle_roll_link` | 0, 0, −25 | Sole plane, directly below ankle joint A |
| `l_gripper`, `r_gripper` | `*_elbow_link` | 0, 0, −44 | Gripper centerpoint |

Inspecting the robot model without physics:

```zsh
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui   # 20-joint sliders
rviz2 -d ../../src/humanoid_description/rviz/mico.rviz
```

## Verification Results

- `check_mujoco.py` full gait evaluation: zero falls, forward travel 186 mm, yaw turn +62.7° (target 60°), base tilt 5.6°, minimum pelvis height 111 mm.
- Stationary IMU reading: +9.80 m/s² on the z-axis.
- Servo load during gait in MuJoCo (P99 peak): hip roll 35% of stall, knee 29%, ankle pitch 27–29%. Hip roll is above the continuous rating (~33% of stall) for about 1.5 s of the 36 s gait, during the single-support lateral leans.
- `kin_dyn.py`: Forward kinematics from DH matches URDF with 0.000 mm maximum error across 300 random joint configurations.

## Joint Sign Conventions

Coordinates: x forward, y left, z up. Positive angles follow the right-hand rule:
- Hip yaw positive: toe turns left.
- Knee positive: knee flexion.
- Hip pitch negative: leg swings forward.
- Left shoulder roll positive: arm abducts outward (right arm opposite).
- Elbow negative: elbow flexes forward.

STM32 firmware strictly adheres to these sign conventions to ensure trajectory compatibility between simulation and physical hardware.
