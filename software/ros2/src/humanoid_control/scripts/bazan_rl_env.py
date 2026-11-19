#!/usr/bin/env python3
"""MuJoCo reinforcement-learning environment for Bazan, on the robot generated from the CAD (bazan_cad.urdf).

    BazanEnv        one environment, 50 Hz policy (20 x 1 ms MuJoCo steps)
    VectorBazanEnv  several in threads

Observations and actions are defined once in bazan_policy_io.py and shared with bazan_rl_node.py, so a policy
sees the same signals and drives the joints the same way in training and on the robot:
    use_perception  64-dim observation (IMU, joints, arms, previous action, command, phase, FSR, ToF, camera
                    proxy, INA226, hip-roll NTC) and 32-dim action (20 joints + 12 adaptive controls)
    otherwise       45 (+2 phase, +4 FSR) observation and 12 leg actions
The sensors come from MuJoCo the way the bridge publishes them: IMU site, bazan_mujoco.foot_forces / tof_range,
servo torques through PowerModel and ThermalModel.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
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

import bazan_policy_io as io  # noqa: E402

LEG_JOINTS, ARM_JOINTS, NECK_JOINTS = io.LEG_JOINTS, io.ARM_JOINTS, io.NECK_JOINTS
ALL_20_JOINTS = LEG_JOINTS + ARM_JOINTS + NECK_JOINTS
ADAPTIVE_12_NAMES, OBS_64_NAMES, ACTION_32_NAMES = io.ADAPTIVE_12_NAMES, io.OBS_64_NAMES, io.ACTION_32_NAMES


def build_bazan_model():
    """Bazan MuJoCo model: the twin generated from the CAD (bazan_cad.urdf)."""
    xml = mm.cad_mjcf()
    return mujoco.MjModel.from_xml_string(xml), xml


build_mico_model = build_bazan_model


class BazanEnv:
    """Single MuJoCo RL environment for Bazan locomotion with disturbance rejection."""

    def __init__(self, model=None, decimation=20, max_episode_steps=500, action_scale=io.ACTION_SCALE,
                 domain_rand=False, perturb_prob=0.0, use_residual=False, use_phase=False,
                 action_filter=io.ACTION_FILTER, obstacle_rand=False, terrain_roughness=0.0,
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
        self.action_scale = float(action_scale)
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

        self.action_map = io.ActionMap(action_scale, action_filter, use_residual, straight_line)
        self.control_dt = decimation * self.model.opt.timestep          # io.CONTROL_DT at the default 20
        self.power, self.thermal, self.camera = io.PowerModel(), io.ThermalModel(), io.CameraProxy()

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
        self.obs_arm_joint_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in io.OBS_ARM_JOINTS])
        self.all_act_ids = np.array([m.actuator(j).id for j in gait.JOINTS])
        self.imu_site = m.site("imu").id
        self.imu_sens = {n: slice(m.sensor_adr[m.sensor(n).id], m.sensor_adr[m.sensor(n).id] + 3)
                         for n in ("imu_acc", "imu_gyro")}
        self.hip_roll_act = self.leg_act_ids[list(io.HIP_ROLL)]

        # Floor geom ID for friction randomization
        try:
            self.floor_geom_id = m.geom("floor").id
            self.default_friction = float(m.geom_friction[self.floor_geom_id, 0])
        except Exception:
            self.floor_geom_id = None
            self.default_friction = 1.0

        # Nominal stand pose: the gait's start pose, CoM between the feet
        self.q_nominal = self.action_map.q_nominal.copy()

        self.leg_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in LEG_JOINTS])
        self.arm_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in ARM_JOINTS])
        self.neck_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in NECK_JOINTS])
        self.nominal_height = gait.SOLE + gait.ROLL_DOWN + gait.H0   # pelvis above the floor
        self.fall_height = 0.6 * self.nominal_height

        # Physics baseline snapshots for domain randomization
        self.orig_gravity = self.model.opt.gravity.copy()
        self.orig_mass = float(self.model.body_mass[self.base_id])
        self.orig_forcerange = self.model.actuator_forcerange.copy()

        # Disturbance state
        self.perturb_force = np.zeros(3, dtype=np.float64)
        self.perturb_steps_left = 0

        # Runtime states
        self.step_count = 0
        self.prev_action = np.zeros(self.act_dim, dtype=np.float64)
        self.command = np.array([io.V_REF, 0.0, 0.0], dtype=np.float64)

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
        self.action_map.reset()
        self.camera.reset()
        self.prev_action = np.zeros(self.act_dim, dtype=np.float64)
        if self.domain_rand:      # battery charge, room temperature, servos already warm from a previous run
            self.power.batt_v = float(np.random.uniform(7.2, 8.3))
            self.thermal.reset(np.random.uniform(25.0, 35.0), np.random.uniform(0.0, 25.0, 2))
        else:
            self.power.batt_v = io.BATT_V
            self.thermal.reset()

        if command is None:
            if getattr(self, "straight_line", False):
                sp = self.target_speed if self.target_speed is not None else io.V_REF
                # Forward and (25% with domain randomization) backward, around the reference gait's speed
                if self.domain_rand and np.random.rand() < 0.25:
                    vx_sample = float(np.random.uniform(-1.0, -0.5) * io.V_REF)
                else:
                    vx_sample = float(np.random.uniform(0.75, 1.25) * sp)
                self.command = np.array([
                    vx_sample,
                    0.0,
                    0.0,
                ], dtype=np.float64)
            else:
                # Random command in units of the reference gait's speed and turn rate
                self.command = np.array([
                    np.random.uniform(-1.0, 1.5) * io.V_REF,
                    np.random.uniform(-0.5, 0.5) * io.V_REF if np.random.rand() < 0.20 else 0.0,
                    np.random.uniform(-1.0, 1.0) * io.W_REF,
                ], dtype=np.float64)
        else:
            self.command = np.asarray(command, dtype=np.float64).copy()

        return self._get_obs()

    def _get_obs(self):
        """Observation from the simulated sensors, assembled by bazan_policy_io like on the robot."""
        d = self.data
        gravity = d.site_xmat[self.imu_site].reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
        gyro = d.sensordata[self.imu_sens["imu_gyro"]]
        leg_q_err = d.qpos[self.leg_joint_qadr] - self.leg_q_nominal
        leg_dq = d.qvel[self.leg_joint_dadr]
        t = self.step_count * self.control_dt
        if self.use_perception:
            tof = mm.tof_range(self.model, d)
            bus_v, current = self.power.read(d.actuator_force[self.all_act_ids])
            return io.obs64(gravity, gyro, d.sensordata[self.imu_sens["imu_acc"]][2], leg_q_err, leg_dq,
                            d.qpos[self.obs_arm_joint_qadr], self.prev_action[:12], self.command, t,
                            mm.foot_forces(self.model, d), tof, self.camera(tof), bus_v, current, self.thermal.temps)
        obs = [gravity, gyro, leg_q_err, leg_dq, self.prev_action[:12], self.command]
        if self.use_phase:
            obs.append(io.phase_vec(t))
        if self.use_fsr:
            obs.append(io.fsr_vec(mm.foot_forces(self.model, d)))
        return np.concatenate(obs).astype(np.float32)

    def step(self, action):
        """Execute one RL control step (decimated physics substeps)."""
        raw_action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)

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
                self.data.qpos[2] = max(self.fall_height, self.data.qpos[2] + z_jitter)

            if getattr(self, "obstacle_active", False) and (self.step_count % 25 == 12):
                l_z = self.data.xpos[self.sole_ids[0]][2]
                r_z = self.data.xpos[self.sole_ids[1]][2]
                swing_z = max(l_z, r_z)
                hurdle_h = float(np.random.uniform(0.010, 0.018))
                if swing_z < hurdle_h:
                    self.data.xfrc_applied[self.base_id, 0] -= float(np.random.uniform(1.0, 2.5))
                    self.data.qpos[2] += 0.005

        # 20 joint targets: the same mapping bazan_rl_node.py uses on the robot
        full_ctrl = self.action_map(raw_action, self.step_count * self.control_dt, self.command)

        self.data.ctrl[self.all_act_ids] = full_ctrl

        for _ in range(self.decimation):
            mujoco.mj_step(self.model, self.data)
        self.thermal.step(self.decimation * self.model.opt.timestep, self.data.actuator_force[self.hip_roll_act],
                          io.STALL_NM[[gait.JOINTS.index("l_hip_roll"), gait.JOINTS.index("r_hip_roll")]])

        self.step_count += 1
        self.prev_action = raw_action.copy()

        obs = self._get_obs()

        base_z = self.data.xpos[self.base_id][2]
        r_mat = self.data.xmat[self.base_id].reshape(3, 3)
        proj_gravity_z = (r_mat.T @ np.array([0.0, 0.0, -1.0]))[2]

        # Termination: fallen or severe pitch/roll (> 75 deg)
        terminated = bool((base_z < self.fall_height) or (proj_gravity_z > -0.25))
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

        # Speeds are scored in units of the reference gait's own speed and turn rate (io.V_REF, io.W_REF):
        # the CAD robot walks quasi-statically, about 9 mm/s.
        # 1. Forward/Backward velocity tracking with directional progress bonus
        cmd_vx = float(self.command[0])
        r_vel_tracking = float(np.exp(-(((v_base[0] - cmd_vx) / io.V_REF) ** 2) / 0.25))
        if abs(cmd_vx) > 0.2 * io.V_REF:
            direction_sign = np.sign(cmd_vx)
            progress = (v_base[0] * direction_sign) / abs(cmd_vx)
            r_vel_forward = float(np.clip(progress, -0.5, 1.5))
        else:
            r_vel_forward = float(-abs(v_base[0]) / io.V_REF)
        r_lin_vel = 2.5 * r_vel_tracking + 1.2 * r_vel_forward

        # 2. Angular yaw rate tracking
        r_ang_vel = np.exp(-(((omega_base[2] - self.command[2]) / io.W_REF) ** 2) / 0.25)

        # 3. Upright posture reward & adaptive roll/pitch lean (Direction 2)
        theta_roll = float(np.arctan2(r_mat[2, 1], r_mat[2, 2]))
        theta_pitch = float(np.arctan2(-r_mat[2, 0], np.sqrt(r_mat[2, 1] ** 2 + r_mat[2, 2] ** 2)))

        # Pitch: lean into a slope (no speed lean: the gait is quasi-static). Roll: follow the reference
        # gait's planned pelvis roll toward the loaded foot (bazan_gait LEAN) when residual, else stay level.
        g_x = -self.model.opt.gravity[0]
        g_z = -self.model.opt.gravity[2]
        pitch_err = theta_pitch - float(np.arctan2(g_x, g_z))
        roll_err = theta_roll - (self.action_map.ref_roll if self.use_residual else 0.0)

        p_roll_tilt = -0.50 * (roll_err ** 2)
        p_roll_vel = -0.05 * (omega_base[0] ** 2)
        p_pitch_tilt = -0.40 * (pitch_err ** 2)
        p_pitch_vel = -0.05 * (omega_base[1] ** 2)
        r_upright = float(np.exp(-(pitch_err ** 2 + roll_err ** 2) / 0.025))

        # 4. Straight-line tracking penalty (lock y = 0 and yaw = 0 unless turning/strafing)
        base_y = float(self.data.xpos[self.base_id][1])
        base_yaw = float(np.arctan2(r_mat[1, 0], r_mat[0, 0]))
        if (getattr(self, "straight_line", False) or abs(self.command[2]) < 0.5 * io.W_REF) and \
                abs(self.command[1]) < 0.2 * io.V_REF:
            p_straight_y = -3.5 * (base_y ** 2)
            p_straight_yaw = -1.2 * (base_yaw ** 2)
        else:
            p_straight_y = 0.0
            p_straight_yaw = 0.0

        # 5. Height maintenance (the gait crouches toward H0_LEANED in single support)
        base_z = self.data.xpos[self.base_id][2]
        r_height = np.exp(-((base_z - self.nominal_height) ** 2) / 0.002)

        # 6. Lateral velocity handling (allow side-strafing if commanded)
        if abs(self.command[1]) > 0.2 * io.V_REF:
            r_lat_vel = float(np.exp(-(((v_base[1] - self.command[1]) / io.V_REF) ** 2) / 0.25))
            p_lateral_vel = 1.0 * r_lat_vel
        else:
            p_lateral_vel = -0.20 * (v_base[1] / io.V_REF) ** 2

        p_action_rate = -0.08 * np.sum((action - self.prev_action) ** 2)
        p_action_mag = -0.05 * np.sum(action ** 2)

        # 7. Alive bonus
        r_alive = 1.0

        # 8. Obstacle Clearance & Dynamic Swing Flight Reward
        l_sole_z = self.data.xpos[self.sole_ids[0]][2]
        r_sole_z = self.data.xpos[self.sole_ids[1]][2]
        swing_h = max(l_sole_z, r_sole_z)
        r_clearance = np.clip((swing_h - 0.5 * gait.LIFT) / (0.5 * gait.LIFT), 0.0, 1.0)

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
                 action_filter=io.ACTION_FILTER, obstacle_rand=False, terrain_roughness=0.0,
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
