"""Render the leg: export the posed parts to STL, then light and render them in Blender (bpy, Cycles CPU).

    python render.py                    # renders/leg_*.png
    python render.py --samples 64
    python render.py --only leg_34,step_05_ankle_roll
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cadquery as cq

import leg as L
import pose as P

HERE = Path(__file__).resolve().parent
OUT = HERE / "renders"

LOOK = {"yaw_bracket": "white", "hip_block": "navy", "thigh": "white", "shin": "white", "ankle_block": "navy",
        "foot": "white", "sole": "sole", "hip_mount": "navy", "thrust_ring": "white"}


def look(name):
    if name in LOOK:
        return LOOK[name]
    return {"servo": "servo", "horn": "horn", "cap": "cap", "spacer": "horn"}[name.split("_", 1)[0]]


POSES = {
    "stand": {},
    "crouch": {"hip_pitch": -25, "knee": 50, "ankle_pitch": -25},
    "deep": {"hip_pitch": -60, "knee": 110, "ankle_pitch": -45, "hip_roll": 15, "ankle_roll": -15},
}

# cameras were framed on leg v3; scale them with the leg (v3: K = 1)
K = (L.L1 + L.L2 + L.HIP_ROLL_UP + L.ANKLE_ROLL_DOWN) / (48.0 + 48.0 + 20.0 + 20.0)


def _scaled(view):
    name, pose, cam, tgt, ortho, xray, exploded = view
    if K == 1.0:
        return view
    t = (tgt[0], tgt[1], tgt[2] * K)
    c = tuple(ti + K * (ci - ti0) for ci, ti0, ti in zip(cam, tgt, t))
    return name, pose, c, t, ortho and ortho * K, xray, exploded


VIEWS = [  # (file, pose, camera location (m), target, ortho scale or None, xray, exploded)
    ("leg_34", "stand", (0.42, 0.36, 0.06), (0, 0, -0.05), None, False, False),
    ("leg_side", "stand", (0, 0.6, -0.045), (0, 0, -0.045), 0.23, False, False),
    ("leg_front", "stand", (0.6, 0, -0.045), (0, 0, -0.045), 0.23, False, False),
    ("leg_xray", "stand", (0.42, 0.36, 0.06), (0, 0, -0.05), None, True, False),
    ("leg_crouch", "crouch", (0.05, 0.6, -0.045), (0, 0, -0.045), 0.23, False, False),
    ("leg_deep", "deep", (0.42, 0.36, 0.06), (0, 0, -0.05), None, False, False),
    ("leg_exploded", "stand", (0.55, 0.5, 0.05), (0, 0, -0.05), None, False, True),
]
VIEWS = [_scaled(v) for v in VIEWS]

# exploded view: links pulled apart along their joint axes (mm)
EXPLODE = {k: tuple(K * c for c in v) for k, v in {
    "pelvis": (0, 0, 75), "yaw_bracket": (0, 0, 40), "hip_block": (0, 0, 0), "thigh": (0, 30, -20),
    "shin": (0, 55, -40), "ankle_block": (0, 0, -60), "foot": (35, 0, -85), "sole": (35, 0, -100)}.items()}


# assembly steps: (file, title, links present, links added in this step); servos, horns and caps follow their links
STEPS = [
    ("step_03_pockets", "Servos into their pockets", ["ankle_block", "thigh", "hip_block", "hip_mount"],
     ["ankle_block", "thigh", "hip_block", "hip_mount"]),
    ("step_05_ankle_roll", "Foot on the ankle roll servo", ["foot", "ankle_block"], ["foot"]),
    ("step_06_ankle_pitch", "Shin on the ankle pitch servo", ["foot", "ankle_block", "shin"], ["shin"]),
    ("step_07_knee", "Thigh: knee servo into the shin", ["foot", "ankle_block", "shin", "thigh"], ["thigh"]),
    ("step_08_hip_pitch", "Hip block on the thigh", ["foot", "ankle_block", "shin", "thigh", "hip_block"],
     ["hip_block"]),
    ("step_09_hip_roll", "Yaw bracket on the hip roll servo",
     ["foot", "ankle_block", "shin", "thigh", "hip_block", "yaw_bracket"], ["yaw_bracket"]),
    ("step_10_hip_yaw", "Hip mount and thrust ring: the leg module",
     ["foot", "ankle_block", "shin", "thigh", "hip_block", "yaw_bracket", "pelvis"], ["pelvis"]),
]
# the four sub-assemblies side by side across the camera's view, all at mid height
STEP_SPREAD = {k: tuple(K * c for c in v) for k, v in {
    "ankle_block": (39, -46, 68), "thigh": (13, -15, -6), "hip_block": (-13, 15, -33),
    "hip_mount": (-39, 46, -90)}.items()}
STEP_CAM = _scaled(("", "", (0.45, 0.38, 0.08), (0, 0, -0.03), None, False, False))[2:4]


def export(tmp, samples, only=None):
    leg = L.Leg()
    manifest = {"samples": samples, "views": []}
    for vname, title, present, new in STEPS:
        if only and vname not in only:
            continue
        comps = P.posed(leg, {})
        files = []
        for n, s in comps.items():
            link = P.owner(leg, n)
            key = n if n in ("hip_mount", "thrust_ring") else link
            if vname == "step_03_pockets" and n == "servo_hip_yaw":
                key = "hip_mount"
            if key not in present and link not in present:
                continue
            if vname == "step_03_pockets":
                if not (n in present or n.startswith("servo_") and key in present):
                    continue
                s = s.moved(cq.Location(cq.Vector(*STEP_SPREAD.get(key, (0, 0, 0)))))
            c = "new" if (key in new or link in new) and not n.startswith(("servo_", "horn_", "cap_")) else look(n)
            f = Path(tmp) / f"{vname}__{n}.stl"
            cq.exporters.export(s, str(f), tolerance=0.05, angularTolerance=0.15)
            files.append((str(f), c))
        manifest["views"].append(dict(name=vname, files=files, cam=STEP_CAM[0], target=STEP_CAM[1],
                                      ortho=None, xray=False))
    for vname, pname, cam, tgt, ortho, xray, exploded in VIEWS:
        if only and vname not in only:
            continue
        comps = P.posed(leg, POSES[pname])
        files = []
        for n, s in comps.items():
            if exploded:
                link = P.owner(leg, n)
                dx = EXPLODE.get(link, (0, 0, 0))
                if n.startswith("servo_"):
                    jn = n.split("_", 1)[1]
                    j = leg.joints[jn]
                    dx = tuple(d + 28 * a for d, a in zip(dx, j.a))     # servos slide out along their shafts
                s = s.moved(cq.Location(cq.Vector(*dx)))
            f = Path(tmp) / f"{vname}__{n}.stl"
            cq.exporters.export(s, str(f), tolerance=0.05, angularTolerance=0.15)
            files.append((str(f), look(n)))
        manifest["views"].append(dict(name=vname, files=files, cam=cam, target=tgt, ortho=ortho, xray=xray))
    return manifest


def main():
    samples = int(sys.argv[sys.argv.index("--samples") + 1]) if "--samples" in sys.argv else 48
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
    OUT.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        m = export(tmp, samples, only)
        m["out"] = str(OUT)
        mf = Path(tmp) / "manifest.json"
        mf.write_text(json.dumps(m))
        subprocess.run([sys.executable, str(HERE / "blender_render.py"), str(mf)], check=True)


if __name__ == "__main__":
    main()
