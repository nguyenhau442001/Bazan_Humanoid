"""Linear static FEA of a driven leg link (gmsh tet10 + CalculiX C3D10), mm / N / MPa.

    python fea.py shin            # the shin as built today and the hybrid shin (metal side plates + PETG core)

The link is clamped where it sits on the lower joint (horn recess and bearing seat around the ankle-pitch axis)
and loaded where the upper joint drives it (around the knee axis):

    landing   3 x robot weight along -z and 0.3 x robot weight sideways (one foot, hard landing)
    stall     knee servo stall torque about the knee axis
    both      landing + stall

Parts of an assembly are fused into one conforming mesh (bonded: screws and the boss capture modelled as glue),
each with its own material. Reported per case: peak von Mises per part, the 99.5th percentile (nodal-load spikes
are not real), the peak more than 2 mm from loaded and clamped nodes, and the largest displacement.
Printed parts are meshed solid: the walls and infill of the real print make them weaker, so the margins below
are optimistic for the printed part (see notes in the report).
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import cadquery as cq
import numpy as np

# E (MPa), nu, allowable for repeated load (MPa) and what it is
MAT = {"PLA": (2300.0, 0.36, 12.0, "PLA printed: keep repeated stress < 10–15 MPa (interlayer 15–25)"),
       "PETG": (2000.0, 0.38, 10.0, "PETG printed: keep repeated stress < 8–12 MPa"),
       "AL5052": (70300.0, 0.33, 95.0, "Al 5052-H32: yield 193 MPa, /2"),
       "CF3K": (50000.0, 0.10, 150.0, "carbon 3K twill plate, in-plane, quasi-isotropic estimate")}
G = 9.81e-3                                                  # N per gram


def mesh(parts, size=1.6):
    """parts: [(name, cq solid, material)] -> nodes (n,3), tets (m,10) node ids 1-based, element material names."""
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    with tempfile.TemporaryDirectory() as d:
        tags = []
        for i, (name, s, _) in enumerate(parts):
            f = Path(d) / f"p{i}.step"
            cq.exporters.export(cq.Workplane().add(s), str(f))
            tags.append(gmsh.model.occ.importShapes(str(f)))
        flat = [t for ts in tags for t in ts if t[0] == 3]
        if len(flat) > 1:
            out, owner = gmsh.model.occ.fragment(flat, [])
        else:
            owner = [flat]
        gmsh.model.occ.synchronize()
        # every fragment volume belongs to the first input part that owns it
        k, vol_part = 0, {}
        for i, ts in enumerate(tags):
            for t in ts:
                if t[0] != 3:
                    continue
                for v in owner[k]:
                    vol_part.setdefault(v[1], i)
                k += 1
    for i in range(len(parts)):
        vs = [v for v, p in vol_part.items() if p == i]
        gmsh.model.addPhysicalGroup(3, vs, i + 1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", size)
    gmsh.option.setNumber("Mesh.MeshSizeMin", size / 3)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
    gmsh.model.mesh.generate(3)
    ids, xyz, _ = gmsh.model.mesh.getNodes()
    order = np.argsort(ids)
    ids, xyz = ids[order], xyz.reshape(-1, 3)[order]
    remap = {int(t): j + 1 for j, t in enumerate(ids)}
    tets, mats = [], []
    for i in range(len(parts)):
        for v in [v for v, p in vol_part.items() if p == i]:
            et, _, en = gmsh.model.mesh.getElements(3, v)
            for t, n in zip(et, en):
                if t != 11:                                   # 10-node tetrahedron
                    continue
                n = np.array([remap[int(x)] for x in n]).reshape(-1, 10)
                tets.append(n[:, [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]])    # gmsh -> CalculiX: swap nodes 9 and 10
                mats += [parts[i][2]] * len(n)
    gmsh.finalize()
    return xyz, np.vstack(tets), mats, [p[0] for p in parts]


def near_axis(xyz, p, a, r, a_lo, a_hi):
    """Nodes within radius r of the axis (p, a) and between a_lo and a_hi along it."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    v = xyz - np.asarray(p, float)
    t = v @ a
    rr = np.linalg.norm(v - np.outer(t, a), axis=1)
    return np.where((rr <= r) & (t >= a_lo) & (t <= a_hi))[0], t, rr


def solve(xyz, tets, mats, fixed, loads):
    """loads: {case: (n, 3) nodal force array}. Returns {case: (von Mises per node, |u| per node)}."""
    used = sorted(set(mats))
    with tempfile.TemporaryDirectory() as d:
        L = ["*NODE, NSET=NALL"] + [f"{i + 1}, {x:.6f}, {y:.6f}, {z:.6f}" for i, (x, y, z) in enumerate(xyz)]
        for m in used:
            L.append(f"*ELEMENT, TYPE=C3D10, ELSET=E{m}")
            L += [f"{k + 1}, " + ", ".join(map(str, t)) for k, t in enumerate(tets) if mats[k] == m]
        L.append("*NSET, NSET=FIX")
        L += [", ".join(str(i + 1) for i in fixed[j:j + 12]) for j in range(0, len(fixed), 12)]
        for m in used:
            E, nu, _, _ = MAT[m]
            L += [f"*MATERIAL, NAME={m}", "*ELASTIC", f"{E}, {nu}", f"*SOLID SECTION, ELSET=E{m}, MATERIAL={m}"]
        L += ["*BOUNDARY", "FIX, 1, 3"]
        for case, F in loads.items():
            L += ["*STEP", "*STATIC", "*CLOAD, OP=NEW"]
            for i in np.where(np.abs(F).sum(1) > 0)[0]:
                for k in range(3):
                    if F[i, k]:
                        L.append(f"{i + 1}, {k + 1}, {F[i, k]:.6e}")
            L += ["*NODE FILE", "U", "*EL FILE", "S", "*END STEP"]
        Path(d, "job.inp").write_text("\n".join(L) + "\n")
        subprocess.run(["ccx", "-i", "job"], cwd=d, check=True, capture_output=True)
        return read_frd(Path(d, "job.frd"), len(xyz), list(loads))


def read_frd(path, n, cases):
    res, block = {}, None
    k = -1
    U = S = None
    for line in path.read_text().splitlines():
        if line.startswith(" -4"):
            block = line.split()[1]
            if block == "DISP":
                k += 1
                U, S = np.zeros((n, 3)), np.zeros((n, 6))
        elif line.startswith(" -1") and block in ("DISP", "STRESS"):
            i = int(line[3:13]) - 1
            vals = [float(line[13 + 12 * j:25 + 12 * j]) for j in range((len(line) - 13) // 12)]
            if block == "DISP":
                U[i] = vals[:3]
            else:
                S[i] = vals[:6]
        elif line.startswith(" -3") and block == "STRESS":
            sx, sy, sz, txy, tyz, tzx = S.T
            vm = np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
                         + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))
            res[cases[k]] = (vm, np.linalg.norm(U, axis=1))
            block = None
        elif line.startswith(" -3"):
            block = None
    return res


def node_parts(tets, mats, n):
    """Material of each node (a node shared by two materials gets the stiffer one's name last)."""
    out = np.array([""] * n, dtype=object)
    for t, m in zip(tets, mats):
        out[t - 1] = m
    return out


def report(title, xyz, tets, mats, res, loaded, fixed):
    nm = node_parts(tets, mats, len(xyz))
    far = np.ones(len(xyz), bool)
    for idx in (loaded, fixed):
        pts = xyz[idx]
        for j in range(0, len(xyz), 4000):
            d = np.min(np.linalg.norm(xyz[j:j + 4000, None, :] - pts[None, ::3, :], axis=2), axis=1)
            far[j:j + 4000] &= d > 2.0
    rows = []
    for case, (vm, u) in res.items():
        for m in sorted(set(mats)):
            sel = nm == m
            fs = sel & far
            peak_far = vm[fs].max() if fs.any() else 0.0
            j = np.where(fs)[0][np.argmax(vm[fs])] if fs.any() else 0
            rows.append(dict(case=case, mat=m, peak=float(vm[sel].max()), p995=float(np.percentile(vm[sel], 99.5)),
                             peak_far=float(peak_far), at=np.round(xyz[j], 1).tolist(), umax=float(u.max()),
                             allow=MAT[m][2]))
    print(f"\n{title}")
    print(f"  {'case':8s} {'material':7s} {'peak':>6s} {'p99.5':>6s} {'peak>2mm':>8s} {'allow':>6s} {'u max':>6s}  at")
    for r in rows:
        print(f"  {r['case']:8s} {r['mat']:7s} {r['peak']:6.1f} {r['p995']:6.1f} {r['peak_far']:8.1f} "
              f"{r['allow']:6.0f} {r['umax']:6.3f}  {r['at']}")
    return rows


def leg_link_cases(xyz, upper, lower, weight_g, stall_nmm, side_sign=1.0):
    """Clamp around the lower joint axis, load around the upper one. upper/lower: Joint objects."""
    import joint as J
    W = weight_g * G
    fixed = set()
    for j in (lower,):
        for (lo, hi), r in (((J.HP[0] - 0.5, J.HP[1] + 0.5), J.HORN_R - 1.5), ((j.IP[0] - 0.5, j.IP[1] + 0.5), 3.6)):
            idx, _, _ = near_axis(xyz, j.p, j.R[:, 2], r, lo, hi)
            fixed |= set(idx.tolist())
    fixed = np.array(sorted(fixed))
    j = upper
    h_idx, t, rr = near_axis(xyz, j.p, j.R[:, 2], J.HORN_R - 1.5, J.HP[0] - 0.5, J.HP[1] + 0.5)
    i_idx, _, _ = near_axis(xyz, j.p, j.R[:, 2], 3.6, j.IP[0] - 0.5, j.IP[1] + 0.5)
    loaded = np.concatenate([h_idx, i_idx])
    n = len(xyz)
    land = np.zeros((n, 3))
    land[loaded] = np.array([0.0, 0.3 * W * side_sign, -3.0 * W]) / len(loaded)
    stall = np.zeros((n, 3))
    a = j.R[:, 2] / np.linalg.norm(j.R[:, 2])
    v = xyz[h_idx] - j.p
    rad = v - np.outer(v @ a, a)
    r2 = (np.linalg.norm(rad, axis=1) ** 2).sum()
    stall[h_idx] = stall_nmm * np.cross(a, rad) / r2                # F_i = T r_i / sum r^2, tangential
    return fixed, loaded, {"landing": land, "stall": stall, "both": land + stall}


def shin_variants():
    import leg as LEG
    old, hy = LEG.Leg(), LEG.Leg("hybrid")
    core, ph, pi = hy.parts["shin"], hy.parts["shin_plate_h"], hy.parts["shin_plate_i"]
    return {"A printed PLA (today)": [("shin", old.parts["shin"], "PLA")],
            f"B PETG core + Al 5052 {LEG.PLATE_T:g}": [
                ("plate_h", ph, "AL5052"), ("plate_i", pi, "AL5052"), ("core", core, "PETG")],
            f"B PETG core + carbon {LEG.PLATE_T:g}": [
                ("plate_h", ph, "CF3K"), ("plate_i", pi, "CF3K"), ("core", core, "PETG")],
            "core alone, PETG (no plates)": [("core", core, "PETG")]}


def shin(hybrid_parts=None):
    import audit as A
    import hardware as hw
    import leg as LEG
    import shell as SH
    lg = LEG.Leg()
    sh = SH.BazanShell(leg=lg)
    _, _, items = A.weigh(sh.leg, sh.body, sh)
    weight = sum(g for _, _, g, _ in items)
    stall = hw.MG92B_STALL_NMM if hasattr(hw, "MG92B_STALL_NMM") else 343.0
    jk, ja = lg.joints["knee"], lg.joints["ankle_pitch"]
    variants = hybrid_parts or {"printed PLA (today)": [("shin", lg.parts["shin"], "PLA")]}
    out = {}
    for title, parts in variants.items():
        xyz, tets, mats, _ = mesh(parts)
        fixed, loaded, loads = leg_link_cases(xyz, jk, ja, weight, stall)
        res = solve(xyz, tets, mats, fixed, loads)
        out[title] = report(f"{title}: robot {weight:.0f} g, stall {stall:.0f} N·mm, {len(tets)} tet10",
                            xyz, tets, mats, res, loaded, fixed)
    return out


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "shin"
    if which == "shin":
        shin(shin_variants())
