"""COTS (Commercial Off-The-Shelf) 3D Model Loader and Placer for Bazan Humanoid.

Loads accurate vendor STEP models for off-the-shelf components:
  - TowerPro MG90S / MG92B metal-gear micro servos (18 joints)
  - Adafruit-style PCA9685 16-channel PWM servo driver module
  - Radxa Zero / Raspberry Pi Zero W SBC
  - Raspberry Pi Camera V2 (Sony IMX219)
  - STMicroelectronics VL53L0X Time-of-Flight distance sensor
  - MR63ZZ miniature ball bearings (3x6x2.5mm)

Places each component exactly into its kinematic / mounting coordinate frame.
"""
import os
import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf, gp_Pnt, gp_Dir, gp_Ax3

import body as B

COTS_DIR = os.path.join(os.path.dirname(__file__), "cots")

# In-memory model cache
_CACHE = {}


def _load_step(filename):
    if filename not in _CACHE:
        path = os.path.join(COTS_DIR, filename)
        if not os.path.exists(path):
            raise FileNotFoundError(f"COTS file not found: {path}")
        imported = cq.importers.importStep(path)
        _CACHE[filename] = imported.val()
    return _CACHE[filename]


def get_cots_servo(optimized=True):
    """Get TowerPro MG90S servo aligned to joint local frame (origin at shaft axis / case top)."""
    key = "servo_local_opt" if optimized else "servo_local"
    if key not in _CACHE:
        if optimized:
            raw = _load_step("mg90s_opt.step")
            trsf = gp_Trsf()
            trsf.SetValues(1.0, 0.0, 0.0, 5.5,
                           0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, -22.7)
            local_shape = cq.Shape(raw.wrapped.Moved(cq.Location(trsf).wrapped))
        else:
            raw = _load_step("mg90s.step")
            trsf = gp_Trsf()
            trsf.SetValues(-1.0, 0.0, 0.0, 5.375,
                           0.0, 0.0, 1.0, 0.000,
                           0.0, 1.0, 0.0, -11.275)
            local_shape = cq.Shape(raw.wrapped.Moved(cq.Location(trsf).wrapped))
        _CACHE[key] = local_shape
    return _CACHE[key]


def place_servo_in_joint(joint, q_angle=0.0):
    """Place the COTS servo in world coordinates for a given joint."""
    servo_local = get_cots_servo()
    return joint.place(servo_local)


def get_cots_camera():
    """Get Raspberry Pi Camera V2 with origin at PCB center and normal along +Z."""
    return _load_step("rpi_camera.step")


def get_cots_tof():
    """Get VL53L0X sensor with origin at PCB center and normal along +Z."""
    return _load_step("vl53l0x.step")


def get_cots_pca9685():
    """Get PCA9685 module with origin at PCB center and normal along +Z."""
    return _load_step("pca9685.step")


def get_cots_radxa_zero():
    """Get Radxa Zero / RPi Zero with origin at PCB center and normal along +Z."""
    return _load_step("rpi_zero_w.step")


def get_cots_bearing():
    """Get MR63ZZ bearing with origin at bearing center and axis along +Z."""
    return _load_step("mr63zz.step")


def get_all_cots_electronics(body):
    """Return dictionary of placed COTS electronics components {name: solid}."""
    res = {}

    # 1. PCA9685 modules
    pca_raw = get_cots_pca9685()
    for bay_name in ("PCA_L", "PCA_R"):
        if bay_name in body.bays:
            deck, c, axis, pitch, sled = body.bays[bay_name]
            zt = B.Z0T if deck == "pelvis_plate" else (B.Z1T if deck == "deck_1" else B.Z2T)
            z_pos = zt + B.SLED_T + B.BOSS_H
            # PCA is mounted along X axis
            loc = cq.Location(cq.Vector(c[0], c[1], z_pos))
            res[f"cots_board_{bay_name}"] = cq.Shape(pca_raw.wrapped.Moved(loc.wrapped))

    # 2. Radxa Zero
    if "RADXA" in body.bays:
        radxa_raw = get_cots_radxa_zero()
        deck, c, axis, pitch, sled = body.bays["RADXA"]
        zt = B.Z0T if deck == "pelvis_plate" else (B.Z1T if deck == "deck_1" else B.Z2T)
        z_pos = zt + B.SLED_T + B.BOSS_H
        loc = cq.Location(cq.Vector(c[0], c[1], z_pos))
        res["cots_board_RADXA"] = cq.Shape(radxa_raw.wrapped.Moved(loc.wrapped))

    # 3. Pi Camera V2 & VL53L0X on cam_mount
    hd = B.HEAD_DECK + 3.0
    t = np.radians(15.0)
    n = np.array([np.cos(t), 0.0, -np.sin(t)])  # normal looking forward and down
    up = np.array([np.sin(t), 0.0, np.cos(t)])  # along face tilt upward
    right = np.array([0.0, 1.0, 0.0])
    o = np.array([22.0, 0.0, hd])
    face0 = o + 3.0 * n

    # Camera location (24mm up from base on the 15-deg inclined face)
    cc = face0 + 24.0 * up
    # Frame on inclined face: X=right, Y=up, Z=n
    ax_cam = gp_Ax3(gp_Pnt(*cc), gp_Dir(*n), gp_Dir(*right))
    trsf_cam = gp_Trsf()
    trsf_cam.SetTransformation(ax_cam)
    trsf_cam.Invert()
    loc_cam = cq.Location(trsf_cam)

    cam_raw = get_cots_camera()
    res["cots_camera_v2"] = cq.Shape(cam_raw.wrapped.Moved(loc_cam.wrapped))

    # ToF location (7mm up from base on the 15-deg inclined face)
    tc = face0 + 7.0 * up
    ax_tof = gp_Ax3(gp_Pnt(*tc), gp_Dir(*n), gp_Dir(*right))
    trsf_tof = gp_Trsf()
    trsf_tof.SetTransformation(ax_tof)
    trsf_tof.Invert()
    loc_tof = cq.Location(trsf_tof)

    tof_raw = get_cots_tof()
    res["cots_sensor_vl53l0x"] = cq.Shape(tof_raw.wrapped.Moved(loc_tof.wrapped))

    return res
