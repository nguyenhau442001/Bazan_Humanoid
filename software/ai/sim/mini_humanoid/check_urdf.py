"""Check that the generated models and the ROS config agree with each other. Needs only the standard library.

    python check_urdf.py

1. humanoid.urdf and mico.urdf.xacro have the same links, joints, origins and limits
2. the gait (mico_gait.JOINTS) drives exactly the 20 revolute joints
3. every sensor / tool frame exists, and camera_optical_frame has the REP-103 optical rotation
"""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
XACRO = REPO / "software" / "ros2" / "src" / "humanoid_description" / "urdf" / "mico.urdf.xacro"
GAIT = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts" / "mico_gait.py"
FRAMES = ["imu_link", "camera_link", "camera_optical_frame", "tof_link", "l_mic_link", "r_mic_link",
          "l_sole", "r_sole", "l_gripper", "r_gripper"]


def colour_link(name):
    """Massless child links that only carry one shell colour for RViz2 (look:=mico): <link>__<colour>."""
    return "__" in name


def joints(root):
    out = {}
    for j in root.iter("joint"):
        if j.find("parent") is None or colour_link(j.get("name")):   # colour links
            continue
        o, lim = j.find("origin"), j.find("limit")
        out[j.get("name")] = (j.get("type"), j.find("parent").get("link"), j.find("child").get("link"),
                              o.get("xyz") if o is not None else None, o.get("rpy") if o is not None else None,
                              None if lim is None else (lim.get("lower"), lim.get("upper"), lim.get("effort")))
    return out


def main():
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("ok    " if cond else "FAIL  ") + msg)
        ok &= bool(cond)

    urdf = ET.parse(HERE / "humanoid.urdf").getroot()
    xac = ET.parse(XACRO).getroot()
    ju, jx = joints(urdf), joints(xac)
    revolute = [n for n, j in ju.items() if j[0] == "revolute"]
    check(len(revolute) == 20, f"URDF has {len(revolute)} revolute joints")
    check(ju == jx, "xacro joints (origins, axes, limits) identical to humanoid.urdf")
    lu = {ln.get("name") for ln in urdf.iter("link")}
    check(lu == {ln.get("name") for ln in xac.iter("link") if not colour_link(ln.get("name"))},
          f"same {len(lu)} links in URDF and xacro (plus the RViz2 colour links of look:=mico)")

    spec = importlib.util.spec_from_file_location("mico_gait", GAIT)
    gait = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gait)
    check(sorted(gait.JOINTS) == sorted(revolute), "mico_gait.JOINTS are the 20 URDF joints")

    for f in FRAMES:
        check(f in lu, f"frame {f}")

    # RViz2 (Jazzy) paints all visuals of a link with the first one's material: one shell colour per link
    mixed = [ln.get("name") for ln in xac.iter("link")
             if len({m.get("name") for v in ln.iter("visual") if v.find("geometry/mesh") is not None
                     for m in v.iter("material")}) > 1]
    check(not mixed, "look:=mico: one shell colour per link (RViz2 shows the same colours as MuJoCo)"
          + (f", mixed: {mixed}" if mixed else ""))
    opt = ju.get("camera_optical_joint")
    check(opt is not None and [round(float(v), 4) for v in opt[4].split()] == [-1.5708, 0.0, -1.5708],
          "camera_optical_frame rpy = (-pi/2, 0, -pi/2): z forward, x right, y down")

    print("\nall checks passed" if ok else "\nSOME CHECKS FAILED")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
