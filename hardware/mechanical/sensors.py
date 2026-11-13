"""Sensors and coordinate frames for Bazan Humanoid.

Defines the physical coordinate frames and orientations for all on-board sensors
following the ROS REP-103 convention (X forward, Y left, Z up; Camera optical: Z forward, X right, Y down).

Sensors included:
  - IMU (MPU6050 6-axis gyro/accel on torso deck 1 perfboard)
  - Camera (Raspberry Pi Camera V2, lens optical center, 15° down tilt)
  - ToF (VL53L0X laser rangefinder on head cam_mount, 15° down tilt)
  - FSR x4 per foot (Ground contact force sensing at 4 foot quadrants + toe/heel centroids)
  - NTC x2 (Thermistor 10 kΩ on left & right hip-roll servo aluminum cases)
"""
from dataclasses import dataclass
import json
import math
import numpy as np


@dataclass(frozen=True)
class SensorFrame:
    """A named sensor coordinate frame attached to a robot kinematic link."""
    name: str               # Unique sensor identifier
    kind: str               # 'imu', 'camera', 'camera_optical', 'tof', 'fsr', 'ntc'
    parent_link: str        # Kinematic link in URDF / CadQuery assembly
    pos: tuple              # (x, y, z) in millimetres relative to parent_link origin
    rpy_deg: tuple          # (roll, pitch, yaw) in degrees relative to parent_link
    axis: tuple = (0, 0, 1)  # Primary sensing axis (e.g. beam direction or surface normal)
    description: str = ""

    def pos_m(self) -> tuple:
        """Position in metres (for ROS / URDF / MuJoCo)."""
        return tuple(round(p / 1000.0, 6) for p in self.pos)

    def rpy_rad(self) -> tuple:
        """Orientation in radians (for ROS / URDF / MuJoCo)."""
        return tuple(round(math.radians(a), 6) for a in self.rpy_deg)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "kind": self.kind,
            "parent_link": self.parent_link,
            "pos_mm": list(self.pos),
            "pos_m": list(self.pos_m()),
            "rpy_deg": list(self.rpy_deg),
            "rpy_rad": list(self.rpy_rad()),
            "axis": list(self.axis),
            "description": self.description
        }


def get_body_sensors(body) -> dict[str, SensorFrame]:
    """Extract sensor frames located on the torso, head, and mounts."""
    import body as B

    sensors = {}

    # 1. IMU (MPU6050 on PERF sled on Deck 1)
    # Perfboard bay is at (13.0, 0.0) on Deck 1.
    # Deck 1 top face Z1T = 120.532 mm. Sled thickness = 4.0 mm, Boss height = 5.0 mm, PCB = 1.6 mm.
    # MPU6050 chip is at center of PCB, facing upright aligned with torso frame.
    imu_z = round(B.Z1T + B.SLED_T + B.BOSS_H + 1.6 + 0.8, 3)
    sensors["imu_frame"] = SensorFrame(
        name="imu_frame",
        kind="imu",
        parent_link="torso",
        pos=(13.0, 0.0, imu_z),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, 1),
        description="MPU6050 6-axis IMU on A+ monitoring perfboard (Deck 1)"
    )

    # 2. Camera & ToF on cam_mount (attached to head_frame)
    hd = B.HEAD_DECK + 3.0
    t = math.radians(15.0)
    n = np.array([math.cos(t), 0, -math.sin(t)])     # face normal (looking forward and down 15 deg)
    up = np.array([math.sin(t), 0, math.cos(t)])    # face up vector
    o = np.array([22.0, 0.0, hd])                   # bottom edge of wedge face
    face0 = o + 3.0 * n

    # Camera lens center: 24 mm up the face
    cc = face0 + 24.0 * up
    cam_pos = tuple(float(x) for x in np.round(cc, 3))
    # Camera body frame: pitch -15° (tilted down 15° about Y axis)
    sensors["camera_frame"] = SensorFrame(
        name="camera_frame",
        kind="camera",
        parent_link="head_frame",
        pos=cam_pos,
        rpy_deg=(0.0, -15.0, 0.0),
        axis=tuple(float(x) for x in np.round(n, 4)),
        description="Raspberry Pi Camera V2 lens center (15° downward tilt)"
    )

    # Camera optical frame: standard ROS REP-103 optical convention
    # In camera optical frame: Z points forward out of lens, X points right, Y points down
    # Rotation from base link: pitch -15°, then roll -90°, yaw -90° (or RPY: -105°, 0°, -90°)
    sensors["camera_optical_frame"] = SensorFrame(
        name="camera_optical_frame",
        kind="camera_optical",
        parent_link="camera_frame",
        pos=(0.0, 0.0, 0.0),
        rpy_deg=(-105.0, 0.0, -90.0),
        axis=(0, 0, 1),
        description="ROS standard camera optical frame (Z forward, X right, Y down)"
    )

    # ToF sensor center: 7 mm up the face
    tc = face0 + 7.0 * up
    tof_pos = tuple(float(x) for x in np.round(tc, 3))
    sensors["tof_frame"] = SensorFrame(
        name="tof_frame",
        kind="tof",
        parent_link="head_frame",
        pos=tof_pos,
        rpy_deg=(0.0, -15.0, 0.0),
        axis=tuple(float(x) for x in np.round(n, 4)),
        description="VL53L0X ToF laser distance sensor (15° downward beam)"
    )

    return sensors


def get_leg_sensors(leg, side: str = "l") -> dict[str, SensorFrame]:
    """Extract sensor frames located on one leg (FSR on foot, NTC on hip-roll)."""
    import leg as L
    sensors = {}
    prefix = f"{side}_"
    sign_y = 1.0 if side == "l" else -1.0

    # 1. Foot FSRs (Ground contact pressure sensing)
    # Ground contact plane under TPU sole
    z_sole = round(leg.sole_top - L.SOLE_PLATE - L.TPU_SOLE, 3)

    # 4 Quadrants on foot sole:
    # Sole length = 76mm (-38..+38), width = 38mm (-19..+19)
    x_toe, x_heel = 22.0, -22.0

    # Front-Left, Front-Right, Rear-Left, Rear-Right
    sensors[f"{prefix}fsr_front_left"] = SensorFrame(
        name=f"{prefix}fsr_front_left",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_toe, 9.5, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot ground contact FSR front-left ({side.upper()} leg)"
    )
    sensors[f"{prefix}fsr_front_right"] = SensorFrame(
        name=f"{prefix}fsr_front_right",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_toe, -9.5, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot ground contact FSR front-right ({side.upper()} leg)"
    )
    sensors[f"{prefix}fsr_rear_left"] = SensorFrame(
        name=f"{prefix}fsr_rear_left",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_heel, 9.5, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot ground contact FSR rear-left ({side.upper()} leg)"
    )
    sensors[f"{prefix}fsr_rear_right"] = SensorFrame(
        name=f"{prefix}fsr_rear_right",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_heel, -9.5, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot ground contact FSR rear-right ({side.upper()} leg)"
    )

    # Toe and Heel centroid contact points (used in 4-dim RL FSR observation)
    sensors[f"{prefix}fsr_toe"] = SensorFrame(
        name=f"{prefix}fsr_toe",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_toe, 0.0, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot toe FSR contact centroid ({side.upper()} leg)"
    )
    sensors[f"{prefix}fsr_heel"] = SensorFrame(
        name=f"{prefix}fsr_heel",
        kind="fsr",
        parent_link=f"{prefix}foot",
        pos=(x_heel, 0.0, z_sole),
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, 0, -1),
        description=f"Foot heel FSR contact centroid ({side.upper()} leg)"
    )

    # 2. NTC 10 kΩ Thermistor on hip-roll servo aluminum case
    # Attached to hip_block link carrying the hip_roll servo
    j_roll = leg.joints["hip_roll"]
    o_roll = j_roll.origin
    # Servo case center: middle of aluminum case along b
    ntc_pos = (round(float(o_roll[0]), 3), round(float(o_roll[1]) + 5.0 * sign_y, 3), round(float(o_roll[2]), 3))
    sensors[f"{prefix}ntc_hip_roll"] = SensorFrame(
        name=f"{prefix}ntc_hip_roll",
        kind="ntc",
        parent_link=f"{prefix}hip_block",
        pos=ntc_pos,
        rpy_deg=(0.0, 0.0, 0.0),
        axis=(0, sign_y, 0),
        description=f"NTC 10k thermistor on {side.upper()} hip-roll servo aluminum case"
    )

    return sensors


def get_all_sensors(body=None, leg=None) -> dict[str, SensorFrame]:
    """Return dictionary of all sensor frames on Bazan."""
    if body is None:
        import body as B
        body = B.Body()
    if leg is None:
        import leg as L
        leg = L.Leg()

    all_s = {}
    all_s.update(get_body_sensors(body))
    all_s.update(get_leg_sensors(leg, "l"))
    all_s.update(get_leg_sensors(leg, "r"))
    return all_s


def export_sensors_json(path: str = "sensors_manifest.json"):
    """Export all sensors to a JSON manifest for URDF and MuJoCo generators."""
    sensors = get_all_sensors()
    data = {name: s.to_dict() for name, s in sensors.items()}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data


if __name__ == "__main__":
    manifest = export_sensors_json("sensors_manifest.json")
    print(f"Exported {len(manifest)} sensor frames to sensors_manifest.json:")
    for name, s in manifest.items():
        print(f"  - [{s['kind']}] {name:24s} -> parent: {s['parent_link']:14s} pos_mm: {s['pos_mm']}")
