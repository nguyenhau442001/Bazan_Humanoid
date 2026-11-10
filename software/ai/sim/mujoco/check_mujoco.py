#!/usr/bin/env python3
"""Mico demo in MuJoCo with physics, no ROS: the same trajectory mico_gait.py sends, the same servo model as
the ROS 2 bridge. Reports fell or not, distance, heading, tilt, hip height and servo loads against stall, and
writes the gait torques to sim/mini_humanoid/tables/mj_servo_loads.csv (read by kin_dyn.py and the engineering
docs).

    python3 sim/mujoco/check_mujoco.py                 # needs xacro (the ros2_jazzy conda env or the VM)
    python3 sim/mujoco/check_mujoco.py --urdf mico.urdf --kp 6 --kv 0.03

Exit code 1 if the robot falls.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mico_mjcf import REPO, mico_urdf, mm, resolve_mesh  # noqa: E402

LOADS = REPO / "sim" / "mini_humanoid" / "tables" / "mj_servo_loads.csv"

gait = mm.gait


def yaw(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(sim, start_delay=2.0, settle=1.0):
    sim.reset()
    sim.step(int(settle / sim.model.opt.timestep))                 # spawned, then the trajectory arrives
    times, positions = gait.trajectory_points(50.0, start_delay)
    sim.set_trajectory(gait.JOINTS, times, positions)
    t0 = sim.time
    start = sim.base_pose()[0]
    stall = np.abs(sim.model.actuator_forcerange[sim.act, 1])
    loads, efforts, tilt, hip_min, fell = [], [], 0.0, 1.0, None
    n = int((start_delay + gait.T_END + 1.0) / sim.model.opt.timestep)
    for i in range(n):
        sim.step()
        if i % 10 == 0:
            xyz, _ = sim.base_pose()
            tilt = max(tilt, sim.tilt())
            hip_min = min(hip_min, xyz[2])
            efforts.append(np.abs(sim.joint_efforts()))
            loads.append(efforts[-1] / stall)
            if fell is None and (xyz[2] < 0.07 or sim.tilt() > math.radians(45)):
                fell = sim.time - t0
    xyz, quat = sim.base_pose()
    loads, efforts = np.array(loads), np.array(efforts)
    return {"fell": fell, "dx": xyz[0] - start[0], "dy": xyz[1] - start[1], "yaw": math.degrees(yaw(quat)),
            "tilt": math.degrees(tilt), "hip_min": hip_min,
            "p99": dict(zip(gait.JOINTS, np.percentile(loads, 99, axis=0))),
            "over": dict(zip(gait.JOINTS, (loads > 1 / 3).mean(axis=0) * (len(loads) * 0.01))),
            "stall": dict(zip(gait.JOINTS, stall)),
            "p99_Nm": dict(zip(gait.JOINTS, np.percentile(efforts, 99, axis=0))),
            "rms_Nm": dict(zip(gait.JOINTS, np.sqrt((efforts ** 2).mean(axis=0))))}


def write_loads(r, path=LOADS):
    """Stall, p99 and RMS effort over the gait (N.m), and the time above the continuous rating (s)."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "stall_Nm", "p99_Nm", "rms_Nm", "p99_pct_stall", "s_above_continuous"])
        for j in sorted(r["stall"]):
            w.writerow([j, f"{r['stall'][j]:.4f}", f"{r['p99_Nm'][j]:.4f}", f"{r['rms_Nm'][j]:.4f}",
                        f"{100 * r['p99'][j]:.4f}", f"{r['over'][j]:.4f}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", help="xacro output of mico.urdf.xacro (default: run xacro)")
    ap.add_argument("--kp", type=float, default=mm.KP)
    ap.add_argument("--kv", type=float, default=mm.KV)
    ap.add_argument("--armature", type=float, default=mm.ARMATURE)
    ap.add_argument("--no-write", action="store_true", help=f"do not write {LOADS.name}")
    args = ap.parse_args()
    sim = mm.MicoSim(mm.build_mjcf(mico_urdf(args.urdf), resolve_mesh, args.kp, args.kv, args.armature))
    r = run(sim)
    print("fell: no" if r["fell"] is None else f"fell: yes, {r['fell']:.1f} s after the trajectory arrived")
    print(f"walked {r['dx'] * 1e3:.0f} mm forward, {r['dy'] * 1e3:+.0f} mm sideways, heading {r['yaw']:+.1f} deg "
          f"(target +60)")
    print(f"max tilt {r['tilt']:.1f} deg, lowest hip {r['hip_min'] * 1e3:.0f} mm")
    print("servo load, 99th percentile of stall (time above 1/3 stall):")
    for j in sorted(r["p99"], key=r["p99"].get, reverse=True)[:6]:
        print(f"  {j:18s} {r['p99'][j] * 100:5.1f}%  ({r['over'][j]:.1f} s)")
    if not args.no_write:
        write_loads(r)
        print(f"wrote {LOADS.relative_to(REPO)}")
    return 0 if r["fell"] is None else 1


if __name__ == "__main__":
    sys.exit(main())
