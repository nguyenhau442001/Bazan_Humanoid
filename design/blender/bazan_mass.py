"""Mass, centre of mass and inertia of Mico v2.0 from the Blender shell, per URDF link.

    python mico_mass.py      # needs numpy + trimesh; Blender is not required

Shell parts are integrated from their closed meshes (outer solid minus the solid shrunk by the wall).
Bought parts (servos, battery, boards) are uniform boxes at their catalogue mass. Every link frame is
parallel to base_link in the stand pose, so a tensor about the link CoM in world axes is already in the
link frame. Writes:
    mass_report.json                     all numbers for the report page
    humanoid_blender.urdf                sim/mini_humanoid/humanoid.urdf with these inertials (full tensors)
    tables/parts_mass.csv, tables/links_mass.csv
and prints the static servo loads from kin_dyn.py for the old and the new model.
"""
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mico_parts as mp  # noqa: E402

SIM = mp.REPO / "sim" / "mini_humanoid"
sys.path.insert(0, str(SIM))
G = 9.81


class Props:
    """Mass (g), first moment (g.mm) and inertia about the world origin (g.mm^2)."""

    def __init__(self, m=0.0, mc=None, I0=None):
        self.m = m
        self.mc = np.zeros(3) if mc is None else mc
        self.I0 = np.zeros((3, 3)) if I0 is None else I0

    def __add__(self, o):
        return Props(self.m + o.m, self.mc + o.mc, self.I0 + o.I0)

    def __sub__(self, o):
        return Props(self.m - o.m, self.mc - o.mc, self.I0 - o.I0)

    def scale(self, k):
        return Props(self.m * k, self.mc * k, self.I0 * k)

    @property
    def com(self):
        return self.mc / self.m

    def I_at(self, p):
        """Inertia tensor about point p (parallel axes)."""
        c = self.com
        Ic = self.I0 - self.m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
        d = c - np.asarray(p)
        return Ic + self.m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def from_com(m, c, Ic):
    c = np.asarray(c, float)
    return Props(m, m * c, Ic + m * (np.dot(c, c) * np.eye(3) - np.outer(c, c)))


def mesh_props(V, F, density):
    t = trimesh.Trimesh(V, F, process=True)
    assert t.is_watertight and t.volume > 0
    return from_com(t.volume * density, t.center_mass, t.moment_inertia * density), t.volume, t.area


def box_props(m, size, c):
    x, y, z = size
    return from_com(m, c, np.diag([m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)]))


def part_props(p, materials=None, wall_scale=1.0):
    kind = p.mass[0]
    mats = materials or {}
    if kind == "shell":
        t = p.mass[1] * wall_scale
        rho = mp.MATERIALS[mats.get(p.mass[2], p.mass[2])]
        outer, vo, area = mesh_props(*mp.part_mesh(p), rho)
        inner, vi, _ = mesh_props(*mp.part_mesh(p, shrink=t), rho)
        return outer - inner, dict(volume=vo - vi, area=area, wall=t, material=mats.get(p.mass[2], p.mass[2]))
    if kind == "solid":
        pr, v, area = mesh_props(*mp.part_mesh(p), mp.MATERIALS[p.mass[1]])
        return pr, dict(volume=v, area=area, wall=None, material=p.mass[1])
    raise ValueError(kind)


CATEGORY = {"servo": "Servos", "battery": "Battery 2S", "pcb": "Electronics", "hidden": "Wiring",
            "extra": "Sensors, brackets, screws"}


def compute(materials=None, wall_scale=1.0):
    links = defaultdict(Props)
    rows = []
    cats = defaultdict(float)
    for p in mp.parts():
        if not p.mass:
            continue
        pr, info = part_props(p, materials, wall_scale)
        links[p.link] += pr
        cat = "Soles" if p.mass[0] == "solid" and "sole" in p.name else "Printed shell"
        cats[cat] += pr.m
        rows.append(dict(part=p.name, link=p.link, group=cat, grams=pr.m, material=info["material"],
                         wall_mm=info["wall"], volume_cm3=info["volume"] / 1000, area_cm2=info["area"] / 100,
                         com=pr.com.tolist()))
    for name, link, size, pos, grams, kind in mp.components():
        pr = box_props(grams, size, pos)
        links[link] += pr
        cat = "Sensors, brackets, screws" if name.endswith("_bracket") or kind == "extra" else CATEGORY[kind]
        cats[cat] += grams
        rows.append(dict(part=name, link=link, group=cat, grams=grams, material="catalogue", wall_mm=None,
                         volume_cm3=None, area_cm2=None, com=list(map(float, pos))))
    return links, rows, dict(cats)


def link_table(links):
    out = {}
    for name, pr in links.items():
        o = np.array(mp.ORIGIN[name])
        Ic = pr.I_at(pr.com)
        out[name] = dict(mass=pr.m, com=(pr.com - o).tolist(), I=Ic.tolist(),
                         principal=sorted(np.linalg.eigvalsh(Ic).tolist(), reverse=True))
    return out


def whole_body(links):
    tot = sum(links.values(), Props())
    return tot, tot.I_at(tot.com)


# ---------------------------------------------------------------- URDF
def urdf_name(body):
    return f"{body}_link"


def write_urdf(table, src, dst):
    s = src.read_text()
    for body, d in table.items():
        ln = urdf_name(body)
        m = d["mass"] / 1000
        c = np.array(d["com"]) / 1000
        it = np.array(d["I"]) * 1e-9
        new = (f'<inertial><origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}"/><mass value="{m:.4f}"/>'
               f'<inertia ixx="{it[0, 0]:.3e}" iyy="{it[1, 1]:.3e}" izz="{it[2, 2]:.3e}" '
               f'ixy="{it[0, 1]:.3e}" ixz="{it[0, 2]:.3e}" iyz="{it[1, 2]:.3e}"/></inertial>')
        pat = re.compile(rf'(<link name="{ln}">\s*)<inertial>.*?</inertial>', re.S)
        s, n = pat.subn(lambda mo: mo.group(1) + new, s, count=1)
        assert n == 1, ln
    dst.write_text(s)


def old_links(path):
    import kin_dyn
    u = kin_dyn.Urdf(path)
    return {k[:-5] if k.endswith("_link") else k: v[0] * 1000 for k, v in u.links.items() if v[0] > 0}


def static_loads(path):
    import kin_dyn
    u = kin_dyn.Urdf(path)
    return {k: (v * 100 / G, lab) for k, (v, lab) in kin_dyn.static_cases(u).items()}   # kg.cm


# ---------------------------------------------------------------- main
def main():
    links, rows, cats = compute()
    table = link_table(links)
    tot, Itot = whole_body(links)
    com = tot.com

    # material and wall sensitivity on the printed parts only
    variants = {}
    for label, mats, ws in [("PLA, walls as drawn (1.0 / 1.2 mm)", None, 1.0),
                            ("PLA, walls +0.4 mm (0.8 mm nozzle, 2 perimeters)", None, 1.4),
                            ("LW-PLA, walls as drawn", {"PLA": "LW-PLA"}, 1.0),
                            ("PLA, walls 0.8 mm (thin, fragile)", None, 0.8)]:
        _, _, c2 = compute(mats, ws)
        variants[label] = sum(c2.values())

    # stability numbers for the walking controller
    zc = com[2]
    foot_x = (10 - 37, 10 + 37)                     # sole extent relative to the ankle axis (mm)
    lipm = dict(com_height_mm=zc, omega=np.sqrt(G / (zc / 1000)), tc_s=np.sqrt((zc / 1000) / G))
    margins = dict(front_mm=foot_x[1] - com[0], back_mm=com[0] - foot_x[0],
                   lateral_shift_to_one_foot_mm=22 + mp.gen_robot.FOOT_OUT - abs(com[1]))

    out_urdf = HERE / "humanoid_blender.urdf"
    write_urdf(table, SIM / "humanoid.urdf", out_urdf)
    old = old_links(SIM / "humanoid.urdf")
    loads_old = static_loads(SIM / "humanoid.urdf")
    loads_new = static_loads(out_urdf)
    servo_of = {b.joint.name: b.joint.servo for b in mp.gen_robot.walk(mp.gen_robot.ROBOT) if b.joint}
    rated = {k: v["stall"] * 100 / G / 3 for k, v in mp.gen_robot.SERVOS.items()}   # 1/3 stall, kg.cm
    joints = []
    for j in servo_of:
        lo, ln = loads_old.get(j, (0, "")), loads_new.get(j, (0, ""))
        joints.append(dict(joint=j, servo=servo_of[j], old=lo[0], new=ln[0], case=ln[1],
                           rated=rated[servo_of[j]], margin=rated[servo_of[j]] / max(ln[0], 1e-6)))

    report = dict(
        total_g=tot.m, old_total_g=sum(old.values()), com_mm=com.tolist(),
        I_body=(Itot).tolist(), I_body_principal=sorted(np.linalg.eigvalsh(Itot).tolist(), reverse=True),
        categories=dict(sorted(cats.items(), key=lambda kv: -kv[1])), variants=variants,
        lipm=lipm, margins=margins,
        links={k: dict(v, old_mass=old.get(k, 0)) for k, v in table.items()},
        parts=rows, joints=joints,
        height_mm=max(mp.part_mesh(p)[0][:, 2].max() for p in mp.parts()),
    )
    (HERE / "mass_report.json").write_text(json.dumps(report, indent=1))

    tab = HERE / "tables"
    tab.mkdir(exist_ok=True)
    with open(tab / "parts_mass.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["part", "link", "group", "mass_g", "material", "wall_mm", "volume_cm3", "area_cm2"])
        for r in rows:
            w.writerow([r["part"], r["link"], r["group"], f"{r['grams']:.2f}", r["material"], r["wall_mm"] or "",
                        f"{r['volume_cm3']:.2f}" if r["volume_cm3"] else "",
                        f"{r['area_cm2']:.1f}" if r["area_cm2"] else ""])
    with open(tab / "links_mass.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["link", "mass_g", "old_mass_g", "com_x_mm", "com_y_mm", "com_z_mm",
                    "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz (g.mm^2, about link CoM)"])
        for k, v in table.items():
            it = np.array(v["I"])
            w.writerow([f"{k}_link", f"{v['mass']:.1f}", f"{old.get(k, 0):.1f}", *[f"{c:.1f}" for c in v["com"]],
                        *[f"{x:.0f}" for x in (it[0, 0], it[1, 1], it[2, 2], it[0, 1], it[0, 2], it[1, 2])]])

    print(f"total {tot.m:.0f} g (old model {sum(old.values()):.0f} g), CoM {com.round(1)} mm")
    for k, v in report["categories"].items():
        print(f"  {k:28s} {v:6.1f} g")
    for k, v in variants.items():
        print(f"  shell variant {k:50s} total {v:.0f} g")
    print("LIPM", {k: round(float(v), 3) for k, v in lipm.items()}, "margins", margins)
    for j in joints:
        print(f"  {j['joint']:18s} {j['servo']:6s} old {j['old']:.2f}  new {j['new']:.2f}  rated {j['rated']:.2f} "
              f"kg.cm  x{j['margin']:.1f}  ({j['case']})")


if __name__ == "__main__":
    main()
