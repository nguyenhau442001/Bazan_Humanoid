# Bazan in MuJoCo — physics on the robot generated from the CAD

One robot description: `software/ros2/src/humanoid_description/urdf/bazan_cad.urdf`, written by
`hardware/mechanical/gen_urdf.py` from the printable CAD (joint axes where the parts put them, joint limits the
parts are checked for, every link's mass, CoM and inertia from its solids, servo ratings from `hardware.py`).
MuJoCo, the gait, RViz2, MoveIt, the RL environment and the firmware joint table are all built from it.

```text
hardware/mechanical (CAD) ──gen_urdf.py──▶ bazan_cad.urdf ──bazan_mujoco.cad_mjcf()──▶ bazan.xml / bridge / RL env
                                                  │
                                                  └──▶ bazan_gait.py (leg geometry, masses) ──JointTrajectory──▶ MuJoCo
```

| Component | Model |
| --- | --- |
| Kinematics, limits, inertials | `bazan_cad.urdf` (1083 g, hip pitch axes 137 mm above the floor) |
| Collision | Each link's bounding box (group 0); legs collide with the floor and with each other |
| Servo | `τ = kp (q* − q) − kv q̇`, clipped at the URDF effort (catalogue stall torque); `kp` = `bazan_gait.SERVO_KP` = 6 N·m/rad, `kv` = 0.03, armature 2e-5 kg·m² |
| IMU | Site on `imu_link` (MPU6050 on deck 1): accelerometer, gyro, orientation |
| Camera | `head_camera` on `camera_link` (camera mount on the head, 15° down) |
| Trajectories | Linear interpolation like `joint_trajectory_controller`; stamp 0 executes immediately |

The gait (`bazan_gait.py`) plans on the same URDF: the pelvis is placed by whole-body IK so the CoM of the CAD masses
sits over the stance foot, the pelvis rolls toward the loaded foot (`LEAN`) to unload the stance hip roll, and each
command carries the static servo sag (hold torque / `SERVO_KP`, ×`SAG_FF`).

## Running

```zsh
python3 sim/mujoco/bazan_mjcf.py            # regenerate bazan.xml after gen_urdf.py changed the URDF (--check in CI)
python3 sim/mujoco/check_mujoco.py          # headless demo: fell or not, distance, heading, servo loads -> servo_loads.csv
mjpython sim/mujoco/bazan_viewer.py --physics
python3 sim/mujoco/bazan_viewer.py --png bazan.png

conda activate ros2_jazzy && source .mac/install/setup.zsh
ros2 launch humanoid_description bazan_mujoco.launch.py gait:=true      # physics + RViz2 + demo gait
ros2 launch humanoid_description bazan_view.launch.py                   # kinematic playback, no physics
```

`bazan.xml` runs with `pip install mujoco` alone (no ROS, no xacro).

## Results on the CAD robot (MuJoCo 4.0, 1 ms step)

`check_mujoco.py`: no fall, 170 mm forward, heading +63° (target +60°), max tilt 15° (the planned 11.5° pelvis roll
plus servo sag). Loads (99th percentile of stall): knee 76 %, hip roll 51 %, ankle roll 33 %; knee and hip roll spend
most of the demo above 1/3 stall, see `docs/engineering/torque_validation.md`.

The same open-loop trajectory was also run against servo and floor variations (not in CI):

| Variation | Result |
| --- | --- |
| 50 Hz servo frame + 15 ms delay | walks |
| Robot mass ±10 % (gait planned on the nominal mass) | walks |
| Servo stiffness 5 or 8 N·m/rad (gait assumes 6) | walks |
| Floor friction 0.42 | walks |
| Servo deadband ±0.25° | walks |
| Floor friction 0.35 | falls |
| Servo deadband ±0.5° | falls |

There is no feedback in this gait. Measure deadband, stiffness and stall on the P0 bench before trusting it on the
real robot, and keep a TPU sole for friction.
