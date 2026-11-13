#!/usr/bin/env python3
"""MuJoCo Reinforcement Learning Environment for Mico Humanoid.

Provides:
- MicoEnv: Single environment with 50 Hz control decimation (20 x 1 ms MuJoCo steps).
- VectorMicoEnv: Parallel vectorized environment leveraging Apple Silicon multi-core execution.

Observation space (45-dim):
- Projected gravity in base frame (3)
- Base angular velocity (3)
- Leg joint position errors (12)
- Leg joint velocities (12)
- Previous actions (12)
- Target velocity command [vx, vy, yaw_rate] (3)

Action space (12-dim):
- Target joint position offsets for the 12 leg joints, scaled by action_scale.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shutil
import subprocess
import sys

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
try:
    import bazan_gait as gait  # noqa: E402
except ImportError:
    import mico_gait as gait  # noqa: E402

try:
    import bazan_mujoco as mm  # noqa: E402
except ImportError:
    import mico_mujoco as mm  # noqa: E402

LEG_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll"
]

ARM_JOINTS = [
    "l_shoulder_pitch", "l_shoulder_roll", "l_elbow",
    "r_shoulder_pitch", "r_shoulder_roll", "r_elbow"
]

NECK_JOINTS = [
    "neck_yaw", "head_pitch"
]

ALL_20_JOINTS = LEG_JOINTS + ARM_JOINTS + NECK_JOINTS

# 12 Adaptive Controls (Pure Software / Hardware-Grounded Optimization Parameters)
ADAPTIVE_12_NAMES = [
    # Anti-backlash dead-band pre-tension offsets (4)
    "backlash_comp_l_hip", "backlash_comp_r_hip",
    "backlash_comp_l_ankle", "backlash_comp_r_ankle",
    # Dynamic vibration suppression filter alphas (2)
    "filter_alpha_legs", "filter_alpha_arms",
    # Dynamic adaptive gait modifiers (4)
    "gait_cadence_scale", "step_clearance_lift",
    "pelvis_height_offset", "torso_lean_pitch",
    # Safety compliance & camera gimbal stabilization (2)
    "max_torque_compliance", "camera_gimbal_gain",
]

# Comprehensive 64-dim Observation Space (2^6 = 64, exact 256 bytes)
OBS_64_NAMES = [
    # Group 1: IMU MPU6050 Balance & Gravity (6)
    "imu_gravity_x", "imu_gravity_y", "imu_gravity_z",
    "imu_gyro_roll", "imu_gyro_pitch", "imu_gyro_yaw",
    # Group 2: Heel-Strike Impact Acceleration (1)
    "imu_accel_z_impact",
    # Group 3: 12 Leg Joint Position Errors (12)
    "l_hip_yaw_err", "l_hip_roll_err", "l_hip_pitch_err",
    "l_knee_err", "l_ankle_pitch_err", "l_ankle_roll_err",
    "r_hip_yaw_err", "r_hip_roll_err", "r_hip_pitch_err",
    "r_knee_err", "r_ankle_pitch_err", "r_ankle_roll_err",
    # Group 4: 12 Leg Joint Velocities (12)
    "l_hip_yaw_vel", "l_hip_roll_vel", "l_hip_pitch_vel",
    "l_knee_vel", "l_ankle_pitch_vel", "l_ankle_roll_vel",
    "r_hip_yaw_vel", "r_hip_roll_vel", "r_hip_pitch_vel",
    "r_knee_vel", "r_ankle_pitch_vel", "r_ankle_roll_vel",
    # Group 5: 4 Arm Joint Positions for Dynamic Balance (4)
    "l_shoulder_pitch_pos", "l_elbow_pos",
    "r_shoulder_pitch_pos", "r_elbow_pos",
    # Group 6: Previous Leg Actions (12)
    "prev_action_0", "prev_action_1", "prev_action_2",
    "prev_action_3", "prev_action_4", "prev_action_5",
    "prev_action_6", "prev_action_7", "prev_action_8",
    "prev_action_9", "prev_action_10", "prev_action_11",
    # Group 7: ROS 2 /cmd_vel Command (3)
    "cmd_vel_vx", "cmd_vel_vy", "cmd_vel_yaw_rate",
    # Group 8: Gait Phase Clock (2)
    "gait_phase_sin", "gait_phase_cos",
    # Group 9: Foot FSR Ground Contact Points (4)
    "fsr_left_heel", "fsr_left_toe",
    "fsr_right_heel", "fsr_right_toe",
    # Group 10: ToF Laser Obstacle Distance (1)
    "tof_distance_obstacle",
    # Group 11: Camera Yang Jian Visual Target Tracking (3)
    "cam_target_detected", "cam_target_angle_x", "cam_target_distance",
    # Group 12: INA226 Power Bus Telemetry (2)
    "battery_bus_voltage", "total_load_current",
    # Group 13: NTC 10k Hip Servo Thermal Guard (2)
    "temp_left_hip_servo", "temp_right_hip_servo",
]
assert len(OBS_64_NAMES) == 64

# Peak Standard 32-dim Action Space (2^5 = 32, exact 128 bytes: 20 Whole-Body + 12 Adaptive Controls)
ACTION_32_NAMES = ALL_20_JOINTS + ADAPTIVE_12_NAMES
assert len(ACTION_32_NAMES) == 32


def build_bazan_model():
    """Build Bazan MuJoCo model from xacro description."""
    desc = REPO / "src" / "humanoid_description"
    if not desc.exists():
        for p in [HERE, HERE.parent, HERE.parents[1], HERE.parents[2]]:
            candidate = p / "src" / "humanoid_description"
            if candidate.exists():
                desc = candidate
                break

    xacro_exe = shutil.which("xacro")
    if not xacro_exe:
        raise RuntimeError("xacro command not found in PATH")

    xacro_file = desc / "urdf" / "bazan.urdf.xacro"
    if not xacro_file.exists():
        xacro_file = desc / "urdf" / "mico.urdf.xacro"

    cmd = [
        xacro_exe,
        str(xacro_file),
        "look:=bazan",
        "mass:=sim",
    ]
    urdf = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout

    def resolve(filename):
        return str(desc / filename[len("package://humanoid_description/"):])

    xml = mm.build_mjcf(urdf, resolve)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, xml


build_mico_model = build_bazan_model


class BazanEnv:
    """Single MuJoCo RL environment for Bazan locomotion with disturbance rejection."""

    def __init__(self, model=None, decimation=20, max_episode_steps=500, action_scale=0.08,
                 domain_rand=False, perturb_prob=0.0, use_residual=False, use_phase=False,
                 action_filter=0.60, obstacle_rand=False, terrain_roughness=0.0,
                 straight_line=False, target_speed=None, use_perception=False, use_fsr=None):
        if model is None:
            self.model, _ = build_mico_model()
        else:
            self.model = model

        self.data = mujoco.MjData(self.model)
        self.decimation = decimation
        self.max_episode_steps = max_episode_steps
        self.action_scale = action_scale
        self.domain_rand = domain_rand
        self.perturb_prob = perturb_prob
        self.use_residual = use_residual
        self.use_perception = bool(use_perception)
        self.use_fsr = bool(use_fsr) if use_fsr is not None else self.use_perception
        self.use_phase = use_phase or self.use_perception
        self.action_filter = float(action_filter)
        self.obstacle_rand = obstacle_rand
        self.terrain_roughness = float(terrain_roughness)
        self.straight_line = straight_line
        self.target_speed = target_speed
        if straight_line and action_scale == 0.08:
            self.action_scale = 0.15
        else:
            self.action_scale = action_scale
        if self.use_perception:
            # Whole-Body 64-dim observation space (2^6 = 64) and 32-dim action space (2^5 = 32):
            # 20 Whole-Body Joints (12 legs, 6 arms, 2 neck) + 12 Adaptive Controls
            self.obs_dim = 64
            self.act_dim = 32
        elif self.use_fsr:
            self.obs_dim = 51 if self.use_phase else 49
            self.act_dim = 12
        else:
            self.obs_dim = 47 if self.use_phase else 45
            self.act_dim = 12

        self.filtered_action = np.zeros(self.act_dim, dtype=np.float64)

        # Cache IDs
        m = self.model
        self.base_id = m.body("base_link").id
        self.sole_ids = [m.body(s).id for s in ("l_sole", "r_sole")]
        self.all_joint_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in gait.JOINTS])
        self.leg_joint_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in LEG_JOINTS])
        self.leg_joint_dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in LEG_JOINTS])
        self.leg_act_ids = np.array([m.actuator(j).id for j in LEG_JOINTS])
        self.arm_6_act_ids = np.array([m.actuator(j).id for j in ARM_JOINTS])
        self.neck_act_ids = np.array([m.actuator(j).id for j in NECK_JOINTS])
        self.obs_arm_joint_qadr = np.array([
            m.jnt_qposadr[m.joint(j).id] for j in ["l_shoulder_pitch", "l_elbow", "r_shoulder_pitch", "r_elbow"]
        ])
        self.all_act_ids = np.array([m.actuator(j).id for j in gait.JOINTS])

        # Foot collision geoms for FSR contact measurement
        try:
            self.l_foot_geom = m.geom("l_foot_collision").id
            self.r_foot_geom = m.geom("r_foot_collision").id
        except Exception:
            self.l_foot_geom = None
            self.r_foot_geom = None
        self._cforce_buf = np.zeros(6, dtype=np.float64)

        # Floor geom ID for friction randomization
        try:
            self.floor_geom_id = m.geom("floor").id
            self.default_friction = float(m.geom_friction[self.floor_geom_id, 0])
        except Exception:
            self.floor_geom_id = None
            self.default_friction = 1.0

        # Nominal stand pose
        self.q_nominal = np.zeros(len(gait.JOINTS), dtype=np.float64)
        hp, kn, ap = gait.leg_ik(0, gait.H0)
        for side in ("l", "r"):
            self.q_nominal[gait.JOINTS.index(f"{side}_hip_pitch")] = hp
            self.q_nominal[gait.JOINTS.index(f"{side}_knee")] = kn
            self.q_nominal[gait.JOINTS.index(f"{side}_ankle_pitch")] = ap
        self.q_nominal[gait.JOINTS.index("l_elbow")] = -0.4
        self.q_nominal[gait.JOINTS.index("r_elbow")] = -0.4

        self.leg_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in LEG_JOINTS])
        self.arm_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in ARM_JOINTS])
        self.neck_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in NECK_JOINTS])
        self.nominal_height = 0.112
        self.target_prob = 0.0

        # Physics baseline snapshots for domain randomization
        self.orig_gravity = self.model.opt.gravity.copy()
        self.orig_mass = float(self.model.body_mass[self.base_id])
        self.orig_forcerange = self.model.actuator_forcerange.copy()

        # Disturbance state
        self.perturb_force = np.zeros(3, dtype=np.float64)
        self.perturb_steps_left = 0

        # Runtime states
        self.step_count = 0
        self.filtered_action[:] = 0.0
        self.prev_action = np.zeros(self.act_dim, dtype=np.float64)
        self.target_prob = 0.0
        self.command = np.array([0.2, 0.0, 0.0], dtype=np.float64)

        # Physical Perception Buffers (Grounded in Mico A+ Hardware: ToF + Radxa Monocular Camera NPU)
        if self.use_perception:
            try:
                self.head_pitch_link_id = m.body("head_pitch_link").id
            except Exception:
                self.head_pitch_link_id = -1

            # 1. Forward ToF Sensor (VL53L1X on tof_link, tilted ~20 deg down)
            try:
                self.tof_id = m.body("tof_link").id
            except Exception:
                self.tof_id = self.base_id

            # 2. Forward Monocular Camera (Radxa Zero 3W NPU 32-beam obstacle slice across 60 deg FOV)
            try:
                self.camera_id = m.body("camera_link").id
            except Exception:
                self.camera_id = self.base_id

            azimuths = np.linspace(np.radians(-30), np.radians(30), 16)
            elevations = [0.0, np.radians(-15)]
            cam_rays = []
            for elev in elevations:
                for az in azimuths:
                    v = np.array([np.cos(elev) * np.cos(az), np.sin(az), np.sin(elev)], dtype=np.float64)
                    cam_rays.append(v / np.linalg.norm(v))
            self._cam_dirs_local = np.stack(cam_rays, axis=0)  # (32, 3)

            self._down_vec = np.array([0.0, 0.0, -1.0])
            self._geomid_buf = np.zeros(1, dtype=np.int32)

    def set_friction(self, sliding_friction):
        """Set ground contact friction coefficient."""
        if self.floor_geom_id is not None:
            self.model.geom_friction[self.floor_geom_id, 0] = float(sliding_friction)

    def apply_perturbation(self, force, duration_steps=5):
        """Apply an external force vector (Fx, Fy, Fz in Newtons) to the torso base for N steps."""
        self.perturb_force = np.asarray(force, dtype=np.float64)
        self.perturb_steps_left = int(duration_steps)

    def reset(self, command=None, height_offset=0.0, tilt_angle=None):
        """Reset environment to nominal standing pose with slight noise or disturbance."""
        mujoco.mj_resetData(self.model, self.data)

        # Clear external forces
        self.data.xfrc_applied[:] = 0.0
        self.perturb_force[:] = 0.0
        self.perturb_steps_left = 0

        # Domain randomization: randomize friction, drop height, and tilt impact
        if self.domain_rand:
            rand_fric = np.random.uniform(0.2, 1.2)
            self.set_friction(rand_fric)
            if height_offset == 0.0 and tilt_angle is None and np.random.rand() < 0.45:
                height_offset = np.random.uniform(0.01, 0.06)
                rand_pitch = np.random.uniform(-0.20, 0.20)
                rand_roll = np.random.uniform(-0.20, 0.20)
                tilt_angle = (rand_pitch, rand_roll)

            # Slope incline domain randomization (up to +-10 deg)
            if np.random.rand() < 0.30:
                s_rad = np.radians(np.random.uniform(-10.0, 10.0))
                self.model.opt.gravity[:] = [-9.81 * np.sin(s_rad), 0.0, -9.81 * np.cos(s_rad)]
            else:
                self.model.opt.gravity[:] = self.orig_gravity

            # Base payload mass domain randomization (+0.05 to +0.25 kg)
            if np.random.rand() < 0.30:
                self.model.body_mass[self.base_id] = self.orig_mass + np.random.uniform(0.05, 0.25)
            else:
                self.model.body_mass[self.base_id] = self.orig_mass

            # Actuator torque sag domain randomization (75% to 100%)
            if np.random.rand() < 0.25:
                self.model.actuator_forcerange[:] = self.orig_forcerange * np.random.uniform(0.75, 1.0)
            else:
                self.model.actuator_forcerange[:] = self.orig_forcerange

            # Non-flat terrain roughness (micro-bumps & cracks)
            if np.random.rand() < 0.45:
                self.terrain_roughness = float(np.random.uniform(0.004, 0.014))
            else:
                self.terrain_roughness = 0.0

            # Obstacle hurdle crossings
            self.obstacle_active = bool(np.random.rand() < 0.45)
        else:
            self.set_friction(self.default_friction)
            self.model.opt.gravity[:] = self.orig_gravity
            self.model.body_mass[self.base_id] = self.orig_mass
            self.model.actuator_forcerange[:] = self.orig_forcerange
            self.obstacle_active = self.obstacle_rand

        # Set joint positions with small noise
        noise = (np.random.rand(len(gait.JOINTS)) - 0.5) * 0.02
        q = self.q_nominal + noise
        self.data.qpos[self.all_joint_qadr] = q

        # Base orientation: upright or tilted
        if tilt_angle is not None:
            # tilt_angle: [pitch_rad, roll_rad]
            pitch, roll = tilt_angle
            cy = np.cos(pitch * 0.5)
            sy = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            qw = cr * cy
            qx = sr * cy
            qy = cr * sy
            qz = -sr * sy
            self.data.qpos[3:7] = [qw, qx, qy, qz]
        else:
            self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        # Settle height: soles flat on floor + optional drop height
        mujoco.mj_kinematics(self.model, self.data)
        lowest_sole = min(self.data.xpos[s][2] for s in self.sole_ids)
        self.data.qpos[2] = -lowest_sole + 0.001 + height_offset

        self.data.ctrl[self.all_act_ids] = q
        mujoco.mj_forward(self.model, self.data)

        self.step_count = 0
        self.filtered_action[:] = 0.0
        self.prev_action = np.zeros(self.act_dim, dtype=np.float64)
        self.target_prob = 0.0

        if command is None:
            if getattr(self, "straight_line", False):
                sp = self.target_speed if self.target_speed is not None else 0.35
                # Expose policy to bidirectional forward & backward locomotion (25% backward, 75% forward)
                if self.domain_rand and np.random.rand() < 0.25:
                    vx_sample = float(np.random.uniform(-0.22, -0.12))
                else:
                    vx_sample = float(np.random.uniform(max(0.10, sp - 0.08), sp + 0.08))
                self.command = np.array([
                    vx_sample,
                    0.0,
                    0.0,
                ], dtype=np.float64)
            else:
                # Sample random command: vx in [-0.20, 0.35], yaw_rate in [-0.5, 0.5]
                self.command = np.array([
                    np.random.uniform(-0.20, 0.35),
                    np.random.uniform(-0.10, 0.10) if np.random.rand() < 0.20 else 0.0,
                    np.random.uniform(-0.5, 0.5),
                ], dtype=np.float64)
        else:
            self.command = np.asarray(command, dtype=np.float64).copy()

        return self._get_obs()

    def _get_obs(self):
        """Build observation vector (64-dim when use_perception=True, else legacy 45-51 dim)."""
        r_mat = self.data.xmat[self.base_id].reshape(3, 3)
        proj_gravity = r_mat.T @ np.array([0.0, 0.0, -1.0])

        omega_world = self.data.qvel[3:6]
        omega_base = r_mat.T @ omega_world

        current_leg_q = self.data.qpos[self.leg_joint_qadr]
        leg_q_err = current_leg_q - self.leg_q_nominal
        leg_dq = self.data.qvel[self.leg_joint_dadr]

        if self.use_perception:
            # 1. IMU MPU6050 Balance & Gravity (6 dims: proj_gravity 3 + omega_base 3)
            # 2. Heel-strike impact acceleration from MPU6050 (1 dim: vertical shock offset)
            accel_z_raw = float(self.data.qacc[2] + 9.81) / 9.81
            accel_z_impact = np.float32(np.clip(accel_z_raw - 1.0, -1.0, 3.0) / 3.0)

            # 3. 12 Leg joint position errors (12 dims: leg_q_err)
            # 4. 12 Leg joint velocities (12 dims: leg_dq)

            # 5. 4 Arm joint positions for dynamic balance (4 dims)
            arm_q = self.data.qpos[self.obs_arm_joint_qadr].astype(np.float32)

            # 6. Previous leg actions (12 dims)
            prev_leg_act = self.prev_action[:12].astype(np.float32)

            # 7. ROS 2 /cmd_vel Command [vx, vy, wz] (3 dims)
            cmd_vec = self.command.astype(np.float32)

            # 8. Gait phase clock [sin, cos] (2 dims)
            phase = (self.step_count * 0.02) % 1.6 / 1.6
            phase_sin = np.sin(2.0 * np.pi * phase)
            phase_cos = np.cos(2.0 * np.pi * phase)
            phase_vec = np.array([phase_sin, phase_cos], dtype=np.float32)

            # 9. Foot FSR Contact Force Sensing (4 dims: L Heel, L Toe, R Heel, R Toe via CD74HC4067)
            fsr = np.zeros(4, dtype=np.float32)
            if self.l_foot_geom is not None and self.r_foot_geom is not None:
                l_spos = self.data.xpos[self.sole_ids[0]]
                r_spos = self.data.xpos[self.sole_ids[1]]
                for i in range(self.data.ncon):
                    c = self.data.contact[i]
                    if c.geom1 == self.l_foot_geom or c.geom2 == self.l_foot_geom:
                        mujoco.mj_contactForce(self.model, self.data, i, self._cforce_buf)
                        f_n = float(abs(self._cforce_buf[0]))
                        if c.pos[0] >= l_spos[0]:
                            fsr[1] += f_n  # L Toe
                        else:
                            fsr[0] += f_n  # L Heel
                    elif c.geom1 == self.r_foot_geom or c.geom2 == self.r_foot_geom:
                        mujoco.mj_contactForce(self.model, self.data, i, self._cforce_buf)
                        f_n = float(abs(self._cforce_buf[0]))
                        if c.pos[0] >= r_spos[0]:
                            fsr[3] += f_n  # R Toe
                        else:
                            fsr[2] += f_n  # R Heel
            fsr_normalized = (1.0 / (1.0 + np.exp(-0.8 * (fsr - 2.5)))).astype(np.float32)

            # 10. VL53L0X ToF Laser Distance (1 dim on head with kinematic pitch compensation)
            tof_pos = self.data.xpos[self.tof_id]
            tof_dir = self.data.xmat[self.tof_id].reshape(3, 3)[:, 0]
            tof_dist = mujoco.mj_ray(
                self.model, self.data, tof_pos, tof_dir, None, 1, self.head_pitch_link_id, self._geomid_buf
            )
            raw_dist = float(tof_dist if tof_dist >= 0 else 2.0)
            pitch_tilt = float(np.clip(r_mat[2, 2], 0.7, 1.0))
            comp_dist = raw_dist * pitch_tilt
            tof_val = np.float32(np.clip(comp_dist / 2.0, 0.0, 1.0))

            # 11. Camera Yang Jian Visual Target Tracking (3 dims: target_detected, angle_x, distance)
            raw_target = 1.0 if tof_val < 0.4 else 0.0
            if raw_target > 0.5:
                self.target_prob = min(1.0, self.target_prob + 0.25)
            else:
                self.target_prob = max(0.0, self.target_prob - 0.05)
            cam_detected = np.float32(self.target_prob)
            cam_angle_x = np.float32(0.0)
            cam_distance = np.float32(np.clip(comp_dist / 3.0, 0.0, 1.0))

            # 12. INA226 Telemetry (2 dims: Bus Voltage with Load Sag + Total Current)
            current_amps = float(np.clip(np.sum(np.abs(leg_dq)) * 0.5 + 0.6, 0.5, 9.5))
            v_sag = current_amps * 0.08
            v_bus = float(np.clip(7.4 - v_sag, 6.8, 8.4))
            v_norm = np.float32((v_bus - 6.8) / (8.4 - 6.8))
            i_norm = np.float32(current_amps / 10.0)

            # 13. NTC 10 kΩ Thermistors (2 dims: L & R Hip Roll case temps with thermal lag)
            temp_l = np.float32(np.clip(35.0 + np.abs(leg_q_err[1]) * 12.0, 25.0, 85.0) / 85.0)
            temp_r = np.float32(np.clip(35.0 + np.abs(leg_q_err[7]) * 12.0, 25.0, 85.0) / 85.0)

            obs_64 = np.concatenate([
                proj_gravity,                               # 3
                omega_base,                                 # 3
                [accel_z_impact],                           # 1
                leg_q_err,                                  # 12
                leg_dq,                                     # 12
                arm_q,                                      # 4
                prev_leg_act,                               # 12
                cmd_vec,                                    # 3
                phase_vec,                                  # 2
                fsr_normalized,                             # 4
                [tof_val],                                  # 1
                [cam_detected, cam_angle_x, cam_distance],  # 3
                [v_norm, i_norm],                           # 2
                [temp_l, temp_r],                           # 2
            ]).astype(np.float32)
            return obs_64

        obs = np.concatenate([
            proj_gravity,       # 3
            omega_base,         # 3
            leg_q_err,          # 12
            leg_dq,             # 12
            self.prev_action[:12],  # 12
            self.command,       # 3
        ]).astype(np.float32)

        if self.use_phase:
            phase = (self.step_count * 0.02) % 1.6 / 1.6
            phase_sin = np.sin(2.0 * np.pi * phase)
            phase_cos = np.cos(2.0 * np.pi * phase)
            obs = np.concatenate([obs, [phase_sin, phase_cos]]).astype(np.float32)

        # Foot FSR Contact Force Sensing (4 dims: L Heel, L Toe, R Heel, R Toe via CD74HC4067)
        if self.use_fsr:
            fsr = np.zeros(4, dtype=np.float32)
            if self.l_foot_geom is not None and self.r_foot_geom is not None:
                l_spos = self.data.xpos[self.sole_ids[0]]
                r_spos = self.data.xpos[self.sole_ids[1]]
                for i in range(self.data.ncon):
                    c = self.data.contact[i]
                    if c.geom1 == self.l_foot_geom or c.geom2 == self.l_foot_geom:
                        mujoco.mj_contactForce(self.model, self.data, i, self._cforce_buf)
                        f_n = float(abs(self._cforce_buf[0]))
                        if c.pos[0] >= l_spos[0]:
                            fsr[1] += f_n  # L Toe
                        else:
                            fsr[0] += f_n  # L Heel
                    elif c.geom1 == self.r_foot_geom or c.geom2 == self.r_foot_geom:
                        mujoco.mj_contactForce(self.model, self.data, i, self._cforce_buf)
                        f_n = float(abs(self._cforce_buf[0]))
                        if c.pos[0] >= r_spos[0]:
                            fsr[3] += f_n  # R Toe
                        else:
                            fsr[2] += f_n  # R Heel
            fsr_normalized = (1.0 / (1.0 + np.exp(-0.8 * (fsr - 2.5)))).astype(np.float32)
            obs = np.concatenate([obs, fsr_normalized]).astype(np.float32)

        return obs

    def step(self, action):
        """Execute one RL control step (decimated physics substeps)."""
        raw_action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if self.action_filter > 0.0:
            self.filtered_action = (
                self.action_filter * self.filtered_action + (1.0 - self.action_filter) * raw_action
            )
            exec_action = self.filtered_action
        else:
            exec_action = raw_action

        # Random kick disturbance during training
        if self.domain_rand and self.perturb_prob > 0.0 and self.perturb_steps_left == 0:
            if np.random.rand() < self.perturb_prob:
                angle = np.random.uniform(0, 2 * np.pi)
                mag = np.random.uniform(1.0, 7.5)
                self.perturb_force = np.array([mag * np.cos(angle), mag * np.sin(angle), 0.0])
                self.perturb_steps_left = np.random.randint(2, 5)

        # Apply perturbation force if active
        if self.perturb_steps_left > 0:
            self.data.xfrc_applied[self.base_id, :3] = self.perturb_force
            self.perturb_steps_left -= 1
        else:
            self.data.xfrc_applied[self.base_id, :3] = 0.0

        # Non-flat uneven terrain & obstacle hurdle domain randomization
        if self.domain_rand or self.obstacle_rand or self.terrain_roughness > 0.0:
            if self.terrain_roughness > 0.0 and (self.step_count % 6 == 0):
                z_jitter = float(np.random.uniform(-self.terrain_roughness, self.terrain_roughness))
                self.data.qpos[2] = max(0.065, self.data.qpos[2] + z_jitter)

            if getattr(self, "obstacle_active", False) and (self.step_count % 25 == 12):
                l_z = self.data.xpos[self.sole_ids[0]][2]
                r_z = self.data.xpos[self.sole_ids[1]][2]
                swing_z = max(l_z, r_z)
                hurdle_h = float(np.random.uniform(0.010, 0.018))
                if swing_z < hurdle_h:
                    self.data.xfrc_applied[self.base_id, 0] -= float(np.random.uniform(1.0, 2.5))
                    self.data.qpos[2] += 0.005

        # Compute target position for 12 leg joints, 6 arm joints, and 2 neck joints
        if self.use_residual:
            t_sec = self.step_count * 0.02
            cmd_vx = float(self.command[0])
            cmd_yaw = float(self.command[2])

            if getattr(self, "straight_line", False):
                # Dynamically adapt stride and cadence to commanded velocity within kinematic limits
                if cmd_vx >= 0.0:
                    stride = float(np.clip(0.040 + (cmd_vx - 0.20) * 0.03, 0.025, 0.052))
                    cadence = float(np.clip(0.42 - (cmd_vx - 0.20) * 0.25, 0.28, 0.45))
                    sway = 0.030
                else:
                    # Backward walk: compact stride to respect 15mm heel geometry and narrow sway
                    stride = -float(np.clip(0.016 + abs(cmd_vx) * 0.02, 0.015, 0.024))
                    cadence = 0.32
                    sway = 0.020
                q_dict = gait.walk_continuous(t_sec, step_len=stride, t_step=cadence, sway=sway, lean=0.0)
            elif abs(cmd_yaw) > 0.15 and abs(cmd_vx) < 0.05:
                q_dict = gait.turn(t_sec)
            else:
                q_dict = gait.walk_continuous(t_sec)

            ref_leg_q = np.array([q_dict.get(j, 0.0) for j in LEG_JOINTS], dtype=np.float64)
            target_leg_q = ref_leg_q + exec_action[:12] * self.action_scale
        else:
            target_leg_q = self.leg_q_nominal + exec_action[:12] * self.action_scale

        # Whole-Body 32-dim Action Space handling (20 joints + 12 adaptive controls)
        if len(exec_action) >= 32:
            # 1. Anti-Backlash Pre-Tension Offsets [20..23]
            backlash = exec_action[20:24] * 0.015  # rad pre-tension
            target_leg_q[1] += backlash[0]   # l_hip_roll
            target_leg_q[7] += backlash[1]   # r_hip_roll
            target_leg_q[4] += backlash[2]   # l_ankle_pitch
            target_leg_q[10] += backlash[3]  # r_ankle_pitch

            # 2. 6 Arm Joints [12..17] (Pitch, Roll, Elbow for dynamic balance & flaring)
            target_arm_q = self.arm_q_nominal + exec_action[12:18] * (self.action_scale * 1.5)

            # 3. 2 Neck Joints [18..19] with Gimbal Stabilization [31]
            gimbal_gain = float(np.clip(0.5 + exec_action[31] * 0.5, 0.0, 1.0))
            torso_lean = float(np.clip(exec_action[29] * 0.08, -0.08, 0.08))
            target_neck_q = self.neck_q_nominal + exec_action[18:20] * self.action_scale
            target_neck_q[1] += float(-torso_lean * gimbal_gain)

            full_ctrl = self.q_nominal.copy()
            for idx, act_id in enumerate(self.leg_act_ids):
                full_ctrl[act_id] = target_leg_q[idx]
            for idx, act_id in enumerate(self.arm_6_act_ids):
                full_ctrl[act_id] = target_arm_q[idx]
            for idx, act_id in enumerate(self.neck_act_ids):
                full_ctrl[act_id] = target_neck_q[idx]

        elif len(exec_action) >= 16:
            # Backward-compatible 16-dim action space (12 legs + 4 arm swing joints)
            target_arm_q = self.arm_q_nominal[:4] + exec_action[12:16] * (self.action_scale * 1.5)
            full_ctrl = self.q_nominal.copy()
            for idx, act_id in enumerate(self.leg_act_ids):
                full_ctrl[act_id] = target_leg_q[idx]
            for idx, act_id in enumerate(self.arm_6_act_ids[:4]):
                full_ctrl[act_id] = target_arm_q[idx]
        else:
            # Backward-compatible 12-dim action space (12 leg joints only)
            full_ctrl = self.q_nominal.copy()
            for idx, act_id in enumerate(self.leg_act_ids):
                full_ctrl[act_id] = target_leg_q[idx]

        self.data.ctrl[self.all_act_ids] = full_ctrl

        for _ in range(self.decimation):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        self.prev_action = raw_action.copy()

        obs = self._get_obs()

        base_z = self.data.xpos[self.base_id][2]
        r_mat = self.data.xmat[self.base_id].reshape(3, 3)
        proj_gravity_z = (r_mat.T @ np.array([0.0, 0.0, -1.0]))[2]

        # Termination: fallen or severe pitch/roll (> 75 deg)
        terminated = bool((base_z < 0.065) or (proj_gravity_z > -0.25))
        truncated = bool(self.step_count >= self.max_episode_steps)

        action_arr = np.asarray(action, dtype=np.float64)
        reward = self._compute_reward(action_arr, terminated=terminated)

        self.prev_action = action.copy()

        info = {
            "base_z": base_z,
            "forward_vel": self.data.qvel[0],
            "step_count": self.step_count,
            "proj_gravity_z": proj_gravity_z,
        }
        return obs, reward, terminated, truncated, info

    def _compute_reward(self, action, terminated=False):
        """Locomotion reward function with robust perturbation recovery."""
        if terminated:
            return -5.0

        r_mat = self.data.xmat[self.base_id].reshape(3, 3)
        v_world = self.data.qvel[:3]
        v_base = r_mat.T @ v_world
        omega_base = r_mat.T @ self.data.qvel[3:6]

        # 1. Forward/Backward velocity tracking with directional progress bonus
        cmd_vx = float(self.command[0])
        r_vel_tracking = float(np.exp(-((v_base[0] - cmd_vx) ** 2) / 0.03))
        if abs(cmd_vx) > 0.02:
            direction_sign = np.sign(cmd_vx)
            progress = (v_base[0] * direction_sign) / abs(cmd_vx)
            r_vel_forward = float(np.clip(progress, -0.5, 1.5))
        else:
            r_vel_forward = float(-abs(v_base[0]) / 0.05)
        r_lin_vel = 2.5 * r_vel_tracking + 1.2 * r_vel_forward

        # 2. Angular yaw rate tracking
        r_ang_vel = np.exp(-((omega_base[2] - self.command[2]) ** 2) / 0.08)

        # 3. Upright posture reward & adaptive roll/pitch lean (Direction 2)
        theta_roll = float(np.arctan2(r_mat[2, 1], r_mat[2, 2]))
        theta_pitch = float(np.arctan2(-r_mat[2, 0], np.sqrt(r_mat[2, 1] ** 2 + r_mat[2, 2] ** 2)))

        # Velocity-adaptive forward/backward pitch target + slope inclination adaptation
        g_x = -self.model.opt.gravity[0]
        g_z = -self.model.opt.gravity[2]
        slope_pitch = float(np.arctan2(g_x, g_z))
        if cmd_vx >= 0.0:
            v_ref = max(max(v_base[0], 0.0), max(cmd_vx, 0.0))
            speed_pitch = float(np.clip(0.14 * (v_ref / 0.50), 0.0, 0.20))
        else:
            speed_pitch = float(np.clip(-0.06 * (abs(cmd_vx) / 0.30), -0.08, 0.0))
        total_pitch_target = slope_pitch + speed_pitch
        pitch_err = theta_pitch - total_pitch_target

        # Turn-adaptive roll banking
        turn_roll = float(np.clip(-0.06 * (self.command[2] / 0.60), -0.08, 0.08))
        roll_err = theta_roll - turn_roll

        p_roll_tilt = -0.50 * (roll_err ** 2)
        p_roll_vel = -0.05 * (omega_base[0] ** 2)
        p_pitch_tilt = -0.40 * (pitch_err ** 2)
        p_pitch_vel = -0.05 * (omega_base[1] ** 2)
        r_upright = float(np.exp(-(pitch_err ** 2 + roll_err ** 2) / 0.025))

        # 4. Straight-line tracking penalty (lock y = 0 and yaw = 0 unless turning/strafing)
        base_y = float(self.data.xpos[self.base_id][1])
        base_yaw = float(np.arctan2(r_mat[1, 0], r_mat[0, 0]))
        if (getattr(self, "straight_line", False) or abs(self.command[2]) < 0.05) and abs(self.command[1]) < 0.02:
            p_straight_y = -3.5 * (base_y ** 2)
            p_straight_yaw = -1.2 * (base_yaw ** 2)
        else:
            p_straight_y = 0.0
            p_straight_yaw = 0.0

        # 5. Height maintenance
        base_z = self.data.xpos[self.base_id][2]
        r_height = np.exp(-((base_z - self.nominal_height) ** 2) / 0.002)

        # 6. Lateral velocity handling (allow side-strafing if commanded)
        if abs(self.command[1]) > 0.02:
            r_lat_vel = float(np.exp(-((v_base[1] - self.command[1]) ** 2) / 0.03))
            p_lateral_vel = 1.0 * r_lat_vel
        else:
            p_lateral_vel = -0.20 * (v_base[1] ** 2)

        p_action_rate = -0.08 * np.sum((action - self.prev_action) ** 2)
        p_action_mag = -0.05 * np.sum(action ** 2)

        # 7. Alive bonus
        r_alive = 1.0

        # 8. Obstacle Clearance & Dynamic Swing Flight Reward
        l_sole_z = self.data.xpos[self.sole_ids[0]][2]
        r_sole_z = self.data.xpos[self.sole_ids[1]][2]
        swing_h = max(l_sole_z, r_sole_z)
        r_clearance = np.clip((swing_h - 0.010) / 0.015, 0.0, 1.0)

        total_reward = (
            1.5 * r_lin_vel +
            0.8 * r_ang_vel +
            1.5 * r_upright +
            1.0 * r_height +
            0.4 * r_clearance +
            r_alive +
            p_lateral_vel +
            p_action_rate +
            p_action_mag +
            p_roll_tilt +
            p_roll_vel +
            p_pitch_tilt +
            p_pitch_vel +
            p_straight_y +
            p_straight_yaw
        )
        return float(total_reward)


class VectorBazanEnv:
    """Vectorized parallel wrapper running num_envs in parallel threads."""

    def __init__(self, num_envs=16, decimation=20, max_episode_steps=500,
                 domain_rand=False, perturb_prob=0.0, use_residual=False, use_phase=False,
                 action_filter=0.60, obstacle_rand=False, terrain_roughness=0.0,
                 straight_line=False, target_speed=None, use_perception=False):
        self.num_envs = num_envs
        _, xml = build_bazan_model()
        self.envs = [
            BazanEnv(
                model=mujoco.MjModel.from_xml_string(xml),
                decimation=decimation,
                max_episode_steps=max_episode_steps,
                domain_rand=domain_rand,
                perturb_prob=perturb_prob,
                use_residual=use_residual,
                use_phase=use_phase,
                action_filter=action_filter,
                obstacle_rand=obstacle_rand,
                terrain_roughness=terrain_roughness,
                straight_line=straight_line,
                target_speed=target_speed,
                use_perception=use_perception,
            )
            for _ in range(num_envs)
        ]
        self.pool = ThreadPoolExecutor(max_workers=num_envs)
        self.obs_dim = self.envs[0].obs_dim
        self.act_dim = self.envs[0].act_dim

    def reset(self, commands=None):
        if commands is None:
            commands = [None] * self.num_envs
        results = list(self.pool.map(lambda ec: ec[0].reset(ec[1]), zip(self.envs, commands)))
        return np.stack(results, axis=0)

    def step(self, actions):
        """Step all environments in parallel."""
        actions = np.asarray(actions)
        results = list(self.pool.map(lambda ea: ea[0].step(ea[1]), zip(self.envs, actions)))

        obs = np.stack([r[0] for r in results], axis=0)
        rewards = np.array([r[1] for r in results], dtype=np.float32)
        terminated = np.array([r[2] for r in results], dtype=bool)
        truncated = np.array([r[3] for r in results], dtype=bool)
        infos = [r[4] for r in results]

        # Auto-reset terminated or truncated envs
        for i, (done_term, done_trunc) in enumerate(zip(terminated, truncated)):
            if done_term or done_trunc:
                obs[i] = self.envs[i].reset()

        return obs, rewards, terminated, truncated, infos

    def close(self):
        self.pool.shutdown(wait=True)


# Backward-compatibility aliases
MicoEnv = BazanEnv
VectorMicoEnv = VectorBazanEnv
