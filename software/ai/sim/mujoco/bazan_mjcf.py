#!/usr/bin/env python3
"""CAD URDF -> MuJoCo MJCF (bazan.xml), with the servos, IMU, camera and contacts of the ROS 2 bridge.

The robot is software/ros2/src/humanoid_description/urdf/bazan_cad.urdf, generated from the CAD
(hardware/mechanical/gen_urdf.py). The MJCF is built by bazan_mujoco.cad_mjcf(), the same code the bridge runs:
    python3 sim/mujoco/bazan_mjcf.py            # write bazan.xml
    python3 sim/mujoco/bazan_mjcf.py --check    # exit 1 if bazan.xml is not what the CAD URDF gives (CI)

Run it again after gen_urdf.py changes the URDF. No ROS, no xacro needed.
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
SCRIPTS_DIR = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
import bazan_mujoco as mm  # noqa: E402

OUT = HERE / "bazan.xml"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="compare with the committed bazan.xml instead of writing")
    args = ap.parse_args()
    xml = mm.cad_mjcf()
    if args.check:
        if not OUT.exists() or OUT.read_text() != xml:
            print(f"{OUT.relative_to(REPO)} is stale: run python3 sim/mujoco/bazan_mjcf.py")
            return 1
        print("up to date")
        return 0
    OUT.write_text(xml)
    m = mm.BazanSim(OUT).model
    print(f"wrote {OUT.relative_to(REPO)}: {m.nbody} bodies, {m.njnt} joints, {m.nu} servos, {m.nsensor} sensors, "
          f"{m.body_mass.sum() * 1000:.0f} g")
    return 0


if __name__ == "__main__":
    sys.exit(main())
