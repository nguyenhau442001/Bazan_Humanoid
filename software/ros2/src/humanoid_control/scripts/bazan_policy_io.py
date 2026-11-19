#!/usr/bin/env python3
"""What a Bazan locomotion policy sees and what its outputs do: one definition for training (bazan_rl_env.py, in
MuJoCo) and deployment (bazan_rl_node.py, on ROS 2). numpy only, no MuJoCo, no ROS.

    Signals -> obs64()                   the 64-dim observation, from raw sensor values in SI units
    ActionMap                            the 32-dim action -> 20 joint targets (filter, residual on the gait)
    PowerModel, ThermalModel             INA226 bus voltage / current and hip-roll NTC case temperatures,
                                         computed from servo torque (the same signals the real sensors measure)

Anything here that is not a measurement is an estimate marked as such; the P0 bench replaces it.
"""
import math
from dataclasses import dataclass, field

import numpy as np

try:
    import bazan_gait as gait
except ImportError:  # pragma: no cover
    import mico_gait as gait

LEG_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll"]
ARM_JOINTS = ["l_shoulder_pitch", "l_shoulder_roll", "l_elbow", "r_shoulder_pitch", "r_shoulder_roll", "r_elbow"]
NECK_JOINTS = ["neck_yaw", "head_pitch"]
OBS_ARM_JOINTS = ["l_shoulder_pitch", "l_elbow", "r_shoulder_pitch", "r_elbow"]
ADAPTIVE_12_NAMES = [
    "backlash_comp_l_hip", "backlash_comp_r_hip", "backlash_comp_l_ankle", "backlash_comp_r_ankle",
    "filter_alpha_legs", "filter_alpha_arms", "gait_cadence_scale", "step_clearance_lift",
    "pelvis_height_offset", "torso_lean_pitch", "max_torque_compliance", "camera_gimbal_gain"]
ACTION_32_NAMES = LEG_JOINTS + ARM_JOINTS + NECK_JOINTS + ADAPTIVE_12_NAMES
OBS_64_NAMES = (
    ["imu_gravity_x", "imu_gravity_y", "imu_gravity_z", "imu_gyro_roll", "imu_gyro_pitch", "imu_gyro_yaw",
     "imu_accel_z_impact"]
    + [f"{j}_err" for j in LEG_JOINTS] + [f"{j}_vel" for j in LEG_JOINTS]
    + [f"{j}_pos" for j in OBS_ARM_JOINTS] + [f"prev_action_{i}" for i in range(12)]
    + ["cmd_vel_vx", "cmd_vel_vy", "cmd_vel_yaw_rate", "gait_phase_sin", "gait_phase_cos",
       "fsr_left_heel", "fsr_left_toe", "fsr_right_heel", "fsr_right_toe", "tof_distance_obstacle",
       "cam_target_detected", "cam_target_angle_x", "cam_target_distance",
       "battery_bus_voltage", "total_load_current", "temp_left_hip_servo", "temp_right_hip_servo"])
assert len(OBS_64_NAMES) == 64 and len(ACTION_32_NAMES) == 32

CONTROL_DT = 0.02                     # s: policy period (50 Hz) in training and on the robot
ACTION_SCALE, ACTION_FILTER = 0.08, 0.60   # rad per unit action, EMA weight of the previous action
ROBOT_TAG = "bazan_cad"               # written into every ONNX policy; the node refuses policies without it
PHASE_PERIOD = 2 * gait.T_STEP        # s: one gait cycle (two steps) of the reference gait
V_REF = gait.STEP / gait.T_STEP       # m/s: forward speed of the reference gait (quasi-static, slow)
W_REF = gait.TURN / (2 * (gait.T_SHIFT + gait.T_SWING * gait.T_TURN + gait.T_SETTLE))   # rad/s: its turning rate
TOF_MAX = 2.0                         # m: VL53L0X range used for normalisation
HIP_ROLL = (LEG_JOINTS.index("l_hip_roll"), LEG_JOINTS.index("r_hip_roll"))
STALL_NM = np.array([gait.MODEL.joints[j]["effort"] for j in gait.JOINTS])   # URDF effort = catalogue stall

# ------------------------------------------------------------------ electrical and thermal models (estimates)
STALL_A_PER_NM = 0.85 / 0.343   # A per N.m of servo torque: MG92B reported stall current / stall torque (P0)
IDLE_A = 0.01                   # A per servo holding no load
SERVO_V, LOGIC_A = 6.0, 0.4     # servo rail (UBEC out) and Radxa + STM32 + sensors from the battery side
UBEC_EFF = 0.9
BATT_V, BATT_R = 7.6, 0.08      # 2S LiPo open-circuit voltage at mid charge, internal + wiring resistance (ohm)
V_LO, V_HI, I_FULL = 6.8, 8.4, 10.0   # normalisation of the INA226 readings
P_STALL_W = SERVO_V * 0.85      # copper loss at stall (all electrical power to heat)
R_TH, TAU_TH = 25.0, 300.0      # K/W case to ambient, s thermal time constant of a micro servo (P0)
T_AMB, T_MAX = 30.0, 85.0       # deg C ambient default, normalisation


@dataclass
class PowerModel:
    """INA226 on the servo rail: total servo current from torque, battery voltage sag from that load."""
    batt_v: float = BATT_V

    def read(self, efforts):
        i_servo = float(np.sum(IDLE_A + STALL_A_PER_NM * np.abs(efforts)))
        i_batt = i_servo * SERVO_V / (self.batt_v * UBEC_EFF) + LOGIC_A
        return self.batt_v - BATT_R * i_batt, i_servo


@dataclass
class ThermalModel:
    """Case temperature of the two hip-roll servos: copper loss (torque / stall)^2 * P_STALL, first-order lag."""
    ambient: float = T_AMB
    temps: np.ndarray = field(default_factory=lambda: np.full(2, T_AMB))

    def reset(self, ambient=T_AMB, preheat=(0.0, 0.0)):
        self.ambient = float(ambient)
        self.temps = self.ambient + np.asarray(preheat, float)

    def step(self, dt, hip_roll_efforts, hip_roll_stall):
        p = P_STALL_W * (np.asarray(hip_roll_efforts) / np.asarray(hip_roll_stall)) ** 2
        steady = self.ambient + R_TH * p
        self.temps = steady + (self.temps - steady) * math.exp(-dt / TAU_TH)
        return self.temps


class CameraProxy:
    """Stand-in for the camera detector until one exists: 'target ahead' when the ToF sees something closer
    than 0.8 m, debounced by a leaky integrator. Same on the robot and in training."""

    def __init__(self):
        self.prob = 0.0

    def reset(self):
        self.prob = 0.0

    def __call__(self, tof_m):
        self.prob = min(1.0, self.prob + 0.25) if tof_m < 0.4 * TOF_MAX else max(0.0, self.prob - 0.05)
        return np.array([self.prob, 0.0, min(tof_m / 3.0, 1.0)], dtype=np.float32)


def phase_vec(t):
    ph = (t % PHASE_PERIOD) / PHASE_PERIOD
    return np.array([math.sin(2 * math.pi * ph), math.cos(2 * math.pi * ph)], dtype=np.float32)


def fsr_vec(forces_n):
    """FSR402 heel / toe forces (N) through the conditioning sigmoid."""
    return (1.0 / (1.0 + np.exp(-0.8 * (np.asarray(forces_n, float) - 2.5)))).astype(np.float32)


def tof_val(range_m, gravity_z):
    """ToF range compensated for body pitch, normalised to [0, 1]."""
    return np.float32(np.clip(range_m * float(np.clip(-gravity_z, 0.7, 1.0)) / TOF_MAX, 0.0, 1.0))


def accel_impact(acc_z):
    """Heel-strike shock from the accelerometer's specific force along the IMU z axis (m/s^2)."""
    return np.float32(np.clip(acc_z / 9.81 - 1.0, -1.0, 3.0) / 3.0)


def obs64(gravity, gyro, acc_z, leg_q_err, leg_dq, arm_q, prev_leg_action, command, t, foot_forces, tof_m,
          cam, bus_v, current_a, hip_temps):
    return np.concatenate([
        gravity, gyro, [accel_impact(acc_z)], leg_q_err, leg_dq, arm_q, prev_leg_action, command, phase_vec(t),
        fsr_vec(foot_forces), [tof_val(tof_m, gravity[2])], cam,
        [(bus_v - V_LO) / (V_HI - V_LO), current_a / I_FULL],
        np.asarray(hip_temps) / T_MAX]).astype(np.float32)


# ------------------------------------------------------------------ actions
def reference(t, command, straight_line=False):
    """Leg joint commands of the reference gait (bazan_gait, planned on the CAD robot) for the residual policy."""
    vx, _, wz = (float(c) for c in command)
    if abs(wz) > 0.5 * W_REF and abs(vx) < 0.25 * V_REF:   # the reference gait only turns left
        q = gait.turn(t)
    elif straight_line:                              # stride from the commanded speed, 2 mm steps, CAD reach
        stride = round(float(np.clip(vx * gait.T_STEP, -gait.STEP, 1.5 * gait.STEP)) / 0.002) * 0.002
        q = gait.walk_continuous(t, step_len=stride)
    else:
        q = gait.walk_continuous(t)
    return np.array([q[j] for j in LEG_JOINTS])


def pelvis_roll(q):
    """Pelvis roll (rad) that puts the soles of leg pose q flat: the gait's planned lean toward the loaded foot."""
    R = gait.MODEL.fk(q)["l_sole"][0]
    return -math.atan2(R[2, 1], R[2, 2])


class ActionMap:
    """Policy output in [-1, 1] -> the 20 joint position targets (gait.JOINTS order)."""

    def __init__(self, action_scale=ACTION_SCALE, action_filter=ACTION_FILTER, residual=False,
                 straight_line=False):
        stand = gait.crouch_pose(1.0)[0]
        self.q_nominal = np.array([stand[j] for j in gait.JOINTS])
        self.idx = {n: np.array([gait.JOINTS.index(j) for j in js])
                    for n, js in (("leg", LEG_JOINTS), ("arm", ARM_JOINTS), ("neck", NECK_JOINTS))}
        self.scale, self.alpha = float(action_scale), float(action_filter)
        self.residual, self.straight_line = residual, straight_line
        self.filtered = None
        self.ref_roll = 0.0       # pelvis roll (rad, about x) the reference gait plans at the last call

    def reset(self):
        self.filtered = None

    def __call__(self, action, t, command):
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        if self.alpha > 0.0:
            self.filtered = a if self.filtered is None else self.alpha * self.filtered + (1 - self.alpha) * a
            a = self.filtered
        q = self.q_nominal.copy()
        if self.residual:
            base = reference(t, command, self.straight_line)
            self.ref_roll = pelvis_roll(dict(zip(LEG_JOINTS, base)))
        else:
            base = q[self.idx["leg"]]
        leg = base + a[:12] * self.scale
        if len(a) >= 32:
            leg[[1, 7, 4, 10]] += a[20:24] * 0.015                  # anti-backlash pre-tension
            q[self.idx["arm"]] = q[self.idx["arm"]] + a[12:18] * self.scale * 1.5
            neck = q[self.idx["neck"]] + a[18:20] * self.scale
            gimbal = float(np.clip(0.5 + a[31] * 0.5, 0.0, 1.0))
            neck[1] -= float(np.clip(a[29] * 0.08, -0.08, 0.08)) * gimbal
            q[self.idx["neck"]] = neck
        elif len(a) >= 16:
            arm4 = self.idx["arm"][[0, 2, 3, 5]]
            q[arm4] = q[arm4] + a[12:16] * self.scale * 1.5
        q[self.idx["leg"]] = leg
        return q


def policy_meta(residual, straight_line, perception):
    """Metadata the trainer writes into the ONNX file so the node rebuilds the same ActionMap."""
    return {"robot": ROBOT_TAG, "residual": bool(residual), "straight_line": bool(straight_line),
            "perception": bool(perception), "action_scale": ACTION_SCALE, "action_filter": ACTION_FILTER,
            "control_dt": CONTROL_DT}
