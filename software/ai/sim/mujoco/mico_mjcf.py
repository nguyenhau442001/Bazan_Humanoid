#!/usr/bin/env python3
"""Mico URDF -> MuJoCo MJCF (mico.xml), with the servos, IMU and contacts of the ROS 2 bridge.

The MJCF is built by src/humanoid_control/scripts/mico_mujoco.py (build_mjcf), the same code the bridge runs,
from the xacro output with the arguments every launch uses (look:=mico mass:=sim):
    python3 sim/mujoco/mico_mjcf.py                    # runs xacro (ros2_jazzy conda env or the VM)
    python3 sim/mujoco/mico_mjcf.py --urdf mico.urdf   # or give the xacro output

Run it again after sim/mini_humanoid/gen_robot.py changes the model. Meshes are referenced in place
(src/humanoid_description/meshes/visual/mico), relative to this folder.
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
SCRIPTS_DIR = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
import mico_mujoco as mm  # noqa: E402

XACRO = REPO / "software" / "ros2" / "src" / "humanoid_description" / "urdf" / "mico.urdf.xacro"
PACKAGE = "package://humanoid_description/"


def resolve_mesh(filename):
    """package://humanoid_description/... -> absolute path in this checkout."""
    if filename.startswith(PACKAGE):
        base_desc = REPO / "software" / "ros2" / "src" / "humanoid_description"
        return str(base_desc / filename[len(PACKAGE):])
    return filename


def resolve_mesh_relative(filename):
    """Same, relative to sim/mujoco, so the saved mico.xml works in any checkout."""
    return os.path.relpath(resolve_mesh(filename), HERE)


def mico_urdf(path=None):
    """xacro output with look:=mico mass:=sim, or the file at `path`."""
    if path:
        return Path(path).read_text()
    exe = shutil.which("xacro")
    if exe is None:
        sys.exit("xacro not found: activate the ros2_jazzy env (or use the VM), or pass --urdf (xacro output)")
    args = [exe, str(XACRO), "look:=mico", "mass:=sim"]
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", help="xacro output of mico.urdf.xacro (default: run xacro)")
    ap.add_argument("-o", "--out", type=Path, default=HERE / "mico.xml")
    args = ap.parse_args()
    xml = mm.build_mjcf(mico_urdf(args.urdf), resolve_mesh_relative, modelfiledir=args.out.parent.resolve())
    args.out.write_text(xml)
    sim = mm.MicoSim(args.out)
    m = sim.model
    print(f"{args.out}: {m.nbody} bodies, {m.njnt} joints, {m.nu} servos, {m.nmesh} meshes, {m.nsensor} sensors")


if __name__ == "__main__":
    main()
