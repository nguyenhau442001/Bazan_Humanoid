#!/usr/bin/env python3
"""Mico in MuJoCo without ROS: the mico_gait demo (walk 8 steps, turn left 60 deg, wave).

--physics runs it like the ROS 2 bridge: servos follow the trajectory mico_gait.py sends, the robot can fall.
Without it the playback is kinematic: joint angles from mico_gait.targets(t), the base pose from the stance-foot
anchor of mico_playback.py, exactly what mico_view.launch.py shows in RViz2.

    mjpython sim/mujoco/mico_viewer.py --physics        # physics, loops the demo (macOS: mjpython)
    mjpython sim/mujoco/mico_viewer.py                  # kinematic playback
    mjpython sim/mujoco/mico_viewer.py --stand          # hold the standing pose
    python3 sim/mujoco/mico_viewer.py --png mico.png    # offscreen: stand / walk / turn / wave side by side
"""
import argparse
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
SCRIPTS_DIR = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
import mico_gait as gait  # noqa: E402
import mico_mujoco as mm  # noqa: E402
import mico_playback as mp  # noqa: E402

DT = 0.02   # 50 Hz, as mico_playback


class Playback:
    """Poses the MuJoCo model at demo time t; base pose from the stance foot, reset at the start of every loop."""

    def __init__(self, model, data):
        self.model, self.data = model, data
        self.kin = KinematicsFromModel(model)
        self.anchor = mp.SoleAnchor(self.kin)
        self.qadr = {j: model.jnt_qposadr[model.joint(j).id] for j in gait.JOINTS}
        self.last_t = None

    def set(self, t):
        q = gait.targets(t)
        if self.last_t is None or t < self.last_t:
            base = self.anchor.reset(q)
        else:
            base = self.anchor.update(q)
        self.last_t = t
        self.data.qpos[:3] = base[:3, 3]
        x, y, z, w = mp.quaternion(base[:3, :3])
        self.data.qpos[3:7] = [w, x, y, z]
        for j, a in self.qadr.items():
            self.data.qpos[a] = q.get(j, 0.0)
        mujoco.mj_kinematics(self.model, self.data)
        return base


class KinematicsFromModel:
    """mico_playback.Kinematics interface on top of the MuJoCo model: pose of a body in base_link."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.qadr = {j: model.jnt_qposadr[model.joint(j).id] for j in gait.JOINTS}

    def pose(self, q, link):
        self.data.qpos[:] = 0
        self.data.qpos[3] = 1.0                       # base at the origin, identity
        for j, a in self.qadr.items():
            self.data.qpos[a] = q.get(j, 0.0)
        mujoco.mj_kinematics(self.model, self.data)
        b = self.model.body(link).id
        t = np.eye(4)
        t[:3, :3] = self.data.xmat[b].reshape(3, 3)
        t[:3, 3] = self.data.xpos[b]
        return t


def visual_option():
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[1] = opt.geomgroup[2] = 1           # Blender shell + floor, hide the collision boxes
    return opt


def configure_camera(viewer, model, is_fpv):
    cam_names = [model.camera(i).name for i in range(model.ncam)]
    if is_fpv and "head_camera" in cam_names:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera("head_camera").id
    else:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.8, -15, 150


def render_png(model, path, fpv=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = mujoco.MjData(model)
    play = Playback(model, data)
    renderer = mujoco.Renderer(model, 480, 640 if fpv else 480)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.75, -15, 150
    shots = {0: "stand", int(7.0 / DT): "walk", int(22.0 / DT): "turn", int(33.0 / DT): "wave"}
    images = []
    for i in range(max(shots) + 1):
        base = play.set(i * DT)
        if i in shots:
            if fpv:
                renderer.update_scene(data, camera="head_camera", scene_option=visual_option())
            else:
                cam.lookat[:] = base[:3, 3] + [0, 0, 0.03]
                renderer.update_scene(data, cam, visual_option())
            images.append((shots[i], renderer.render().copy()))
    fig, axes = plt.subplots(1, len(images), figsize=(4.5 * len(images), 4.2))
    for ax, (name, img) in zip(axes, images):
        ax.imshow(img)
        view_label = " (FPV Head Camera)" if fpv else ""
        ax.set_title(f"{name}{view_label}")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    print(f"wrote {path}")


def view(model, stand, fpv=False):
    import mujoco.viewer

    data = mujoco.MjData(model)
    play = Playback(model, data)
    fpv_mode = [fpv]

    with mujoco.viewer.launch_passive(model, data) as v:
        v.opt.geomgroup[0] = 0
        configure_camera(v, model, fpv_mode[0])

        def key_callback(keycode):
            if keycode in (ord('v'), ord('V'), ord('c'), ord('C')):
                fpv_mode[0] = not fpv_mode[0]
                configure_camera(v, model, fpv_mode[0])
                mode_str = "FPV (head_camera)" if fpv_mode[0] else "Third-Person Orbit View"
                print(f"[📷 CAMERA] Switched to {mode_str}")

        v._key_callback = key_callback
        start = time.monotonic()
        while v.is_running():
            t = 0.0 if stand else (time.monotonic() - start) % gait.T_END
            base = play.set(t)
            if not fpv_mode[0]:
                v.cam.lookat[:] = base[:3, 3] + [0, 0, 0.03]
            v.sync()
            time.sleep(DT)


def view_physics(model, stand, fpv=False):
    """Spawn with straight legs, settle 1 s, then the mico_gait trajectory (2 s delay); start over at the end."""
    import mujoco.viewer

    sim = mm.MicoSim(model)
    times, positions = gait.trajectory_points(50.0, 2.0)
    length = times[-1] + 2.0
    fpv_mode = [fpv]

    with mujoco.viewer.launch_passive(sim.model, sim.data) as v:
        v.opt.geomgroup[0] = 0
        configure_camera(v, sim.model, fpv_mode[0])

        def key_callback(keycode):
            if keycode in (ord('v'), ord('V'), ord('c'), ord('C')):
                fpv_mode[0] = not fpv_mode[0]
                configure_camera(v, sim.model, fpv_mode[0])
                mode_str = "FPV (head_camera)" if fpv_mode[0] else "Third-Person Orbit View"
                print(f"[📷 CAMERA] Switched to {mode_str}")

        v._key_callback = key_callback
        while v.is_running():
            sim.reset()
            sim.step(1000)
            if not stand:
                sim.set_trajectory(gait.JOINTS, times, positions)
            t_end, wall0, sim0 = sim.time + length, time.monotonic(), sim.time
            while v.is_running() and (stand or sim.time < t_end):
                sim.step(int(DT / sim.model.opt.timestep))
                if not fpv_mode[0]:
                    v.cam.lookat[:] = sim.base_pose()[0] + [0, 0, 0.03]
                v.sync()
                time.sleep(max(0.0, (sim.time - sim0) - (time.monotonic() - wall0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=HERE / "mico.xml")
    ap.add_argument("--stand", action="store_true", help="hold the standing pose")
    ap.add_argument("--physics", action="store_true", help="servos and contacts instead of kinematic playback")
    ap.add_argument("--fpv", action="store_true", help="launch in First-Person View from head_camera (toggle: V key)")
    ap.add_argument("--png", type=Path, help="render stand / walk / turn / wave to this file and exit")
    ap.add_argument("--fpv-png", type=Path, help="render FPV stand / walk / turn / wave to this file and exit")
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model))
    if args.png:
        render_png(model, args.png, fpv=False)
    elif args.fpv_png:
        render_png(model, args.fpv_png, fpv=True)
    else:
        (view_physics if args.physics else view)(model, args.stand, args.fpv)


if __name__ == "__main__":
    main()
