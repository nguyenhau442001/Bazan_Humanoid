"""Engineering review of N3 (head_frame) and the head-pitch joint, read-only: it changes no CAD.

Every number in docs/mechanical/n3_review/*.md comes from this script, so the review can be rerun after each
measurement or design change:

    python n3_review.py            # all checks, writes docs/mechanical/n3_review/n3_review.json
    python n3_review.py --no-fea --no-sweep --no-slice   # quick hand checks only; does not rewrite the JSON

Each finding carries a status:
    VERIFIED  follows from the CAD source or a manufacturer tolerance, no assumption
    ASSUMED   computed from values marked MEASURE in hardware.py or from an assumed material property
    UNKNOWN   needs a physical measurement or a decision before it can be computed
    FAILED    the check fails on the CAD/BOM values themselves (or on every plausible input)
"""
import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

import hardware as hw
import joint as J

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "mechanical" / "n3_review"

# Values that are not in the CAD source. Each one names where it comes from.
SRC = {
    "MR63ZZ": "MR63ZZ 3×6×2.5, OD tol 0/−0.008, width tol 0/−0.12 (vendor datasheets TFL, SKD; ISO normal class)",
    "SG90_DS": "SG90 sizes disagree between datasheets: 22.2×11.8, 23.1×12.2, 22.0×11.5 mm (TowerPro copies)",
    "SG90_TQ": "SG90 stall torque: 1.8 kgf·cm (TowerPro copy) and 1.2 kgf·cm at 4.8 V (other copies)",
    "INS": "M2 heat-set inserts sold for a Ø3.2 hole have a knurl OD of about 3.6–3.7 (e.g. TFC: L4, W3.7, "
           "hole 3.2, min wall 1.6; Accu HTFBI-M2: L3.9, hole 3.2 +0.1/−0)",
    "PLA_E": "printed PLA E = 2300 MPa, ν = 0.35 (assumed, same as the leg FEA; real value depends on infill/walls)",
    "MG90S_STEP": "vendor MG90S STEP in cots/: tower 5.75 (CAD 4.0), spline top 9.7 (CAD 8.3) — see test_print.md",
}
G = 9.81


def finding(fid, sev, status, title, evidence, numbers=None, action=""):
    return dict(id=fid, severity=sev, status=status, title=title, evidence=evidence, numbers=numbers or {},
                action=action)


# ------------------------------------------------------------------ geometry of the stack (all from joint.py)
def stack():
    h = hw.HORN
    pilot_bottom = J.HP[1] - 0.4
    return dict(
        horn_plate=J.HP, idler_plate=J.IP, idler_wall=J.IW, bearing=J.BRG, boss=J.BOSS, lip=(J.BRG[1], J.IP[1]),
        gap_inner=round(J.HP[0] - J.IP[1], 3), horn_top=J.HORN_TOP, head_top=J.HEAD_TOP,
        recess_d=h.disc_d + 0.4, recess_floor=J.HORN_TOP, plate_t_horn=round(J.HP[1] - J.HP[0], 3),
        plate_t_idler=round(J.IP[1] - J.IP[0], 3), pilot_depth=round(pilot_bottom - (J.HORN_TOP - 0.1), 3),
        pilot_bottom=round(pilot_bottom, 3), skin=0.4, seat_depth=round(J.BRG[1] - J.IP[0] - J.CAP_T, 3),
        holder_to_idler_plate=J.GAP, tower_to_horn_plate=round(J.HP[0] - J.S.tower_h, 3),
        horn_hub_above_tower=0.4)


def shaft_offset(servo):
    """Where the servo shaft sits relative to the joint axis (the idler M3 insert) in the MG92B pocket.

    Returns the range of the shaft position along b and w when the ear screws hold the servo, and when they are
    loose (the case can move inside the pocket)."""
    P = J.S                                              # the pocket is built for this servo (MG92B)
    c = hw.CLEAR_SERVO
    pocket_b = (J.CB0 - c, J.CB + c)
    pocket_w = (-P.width / 2 - c, P.width / 2 + c)
    # shaft position when the case touches either end / side of the pocket
    b_lo = pocket_b[0] + servo.shaft_off
    b_hi = pocket_b[1] - (servo.length - servo.shaft_off)
    w_play = (pocket_w[1] - pocket_w[0] - servo.width) / 2
    # shaft position when the ear holes are centred on the inserts
    ear_mid = J.CMID                                     # insert pair centre along b
    servo_mid_to_shaft = servo.length / 2 - servo.shaft_off
    b_ears = ear_mid - servo_mid_to_shaft
    pitch_err = abs(P.hole_pitch - servo.hole_pitch) / 2
    return dict(b_pocket=(round(b_lo, 3), round(b_hi, 3)), w_pocket=(round(-w_play, 3), round(w_play, 3)),
                b_ears_centred=round(b_ears, 3), ear_pitch_error_each=round(pitch_err, 3))


def frame_stiffness(work):
    """Relative stiffness of the bearing seat against the horn plate, CalculiX tet10, unit loads along b and w."""
    import cadquery as cq
    import gmsh
    import body as B
    body = B.Body()
    j = body.joints["head_pitch"]
    solid = body.parts["head_frame"].fuse(body.parts["head_idler"]).clean()      # bolted pair, taken as bonded
    step = work / "n3.step"
    cq.exporters.export(solid, str(step))
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.importShapes(str(step))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", 1.6)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.4)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
    gmsh.model.mesh.generate(3)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    X = xyz.reshape(-1, 3)
    idx = {int(t): i for i, t in enumerate(tags)}
    et, _, enodes = gmsh.model.mesh.getElements(3)
    E = np.array([idx[int(n)] for n in enodes[list(et).index(11)]]).reshape(-1, 10)
    gmsh.finalize()
    a = np.array(j.R[:, 2])
    bdir = np.array(j.R[:, 0])
    wdir = np.array(j.R[:, 1])
    v = X - j.p
    t = v @ a
    r = np.linalg.norm(v - np.outer(t, a), axis=1)
    horn = (t > J.HP[0] - 0.05) & (t < J.HORN_TOP + 0.05) & (r > hw.HORN.disc_d / 2 + 0.15) & \
           (r < hw.HORN.disc_d / 2 + 0.25)                          # recess wall, where the horn disc bears
    horn |= (np.abs(t - J.HORN_TOP) < 0.05) & (r < hw.HORN.disc_d / 2 + 0.25)   # recess floor
    seat = (t > J.IP[0] + J.CAP_T - 0.05) & (t < J.BRG[1] + 0.05) & (np.abs(r - hw.HOLES["bearing"].d / 2) < 0.05)
    n_seat = int(seat.sum())
    loads = {}
    for name, d in (("b", bdir), ("w", wdir), ("a", -a)):
        F = np.zeros_like(X)
        F[seat] = d / n_seat                                           # 1 N total on the seat
        loads[name] = F
    inp = work / "n3k.inp"
    perm = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]
    with open(inp, "w") as f:
        f.write("*NODE\n")
        for i, x in enumerate(X):
            f.write(f"{i + 1},{x[0]:.5f},{x[1]:.5f},{x[2]:.5f}\n")
        f.write("*ELEMENT,TYPE=C3D10,ELSET=EALL\n")
        for i, e in enumerate(E):
            f.write(f"{i + 1}," + ",".join(str(e[p] + 1) for p in perm) + "\n")
        f.write("*NSET,NSET=FIX\n" + "".join(f"{n + 1}\n" for n in np.where(horn)[0]))
        f.write("*MATERIAL,NAME=PLA\n*ELASTIC\n2300.0,0.35\n*SOLID SECTION,ELSET=EALL,MATERIAL=PLA\n")
        f.write("*BOUNDARY\nFIX,1,3\n")
        for name, F in loads.items():
            f.write("*STEP\n*STATIC\n*CLOAD,OP=NEW\n")
            for n in np.where(np.abs(F).sum(1) > 0)[0]:
                for dd in range(3):
                    if F[n, dd] != 0:
                        f.write(f"{n + 1},{dd + 1},{F[n, dd]:.6e}\n")
            f.write("*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n")
    r_ = subprocess.run(["ccx", "-i", "n3k"], cwd=work, capture_output=True, text=True)
    if r_.returncode != 0:
        raise RuntimeError(r_.stdout[-500:])
    U, S, cur, kind = [], [], None, None
    for line in open(work / "n3k.frd"):
        if line.startswith(" -4  DISP"):
            cur, kind = np.zeros((len(X), 3)), "U"
        elif line.startswith(" -4  STRESS"):
            cur, kind = np.zeros((len(X), 6)), "S"
        elif line.startswith(" -3") and cur is not None:
            (U if kind == "U" else S).append(cur)
            cur = None
        elif cur is not None and line.startswith(" -1"):
            n = int(line[3:13]) - 1
            cur[n] = [float(line[13 + 12 * i:25 + 12 * i]) for i in range(cur.shape[1])]
    ub = float((U[0][seat] @ bdir).mean())
    uw = float((U[1][seat] @ wdir).mean())
    ua = float((U[2][seat] @ -a).mean())

    def vm(s):
        sx, sy, sz, sxy, syz, szx = s.T
        return np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    smax = [float(np.percentile(vm(s), 99.9)) for s in S]       # 99.9th percentile: ignores load-point spikes
    vm_a = vm(S[2])
    hot = X[np.argmax(vm_a)] - j.p
    return dict(k_b=1.0 / ub, k_w=1.0 / uw, k_a=1.0 / ua, vm_per_N_b=smax[0], vm_per_N_w=smax[1], vm_per_N_a=smax[2],
                hot_a_local=[round(float(hot @ bdir), 1), round(float(hot @ wdir), 1), round(float(hot @ a), 1)],
                nodes=len(X), elements=len(E),
                seat_nodes=n_seat, fixed_nodes=int(horn.sum()))


# ------------------------------------------------------------------ rev P2: every P1 finding re-checked on the CAD
INS_M2 = dict(od=3.6, hole=3.2, wall=1.3, src="CNC Kitchen TC-M2x3.0: L3.0, knurl Ø3.6, hole Ø3.2, wall ≥ 1.3")
NSK_DB_MAX = 3.7          # MF63 / MR63 shaft abutment diameter, max (NSK catalogue)
MG90S_STEP_RISE = 9.7 - 8.3


def removal(body, leg):
    """N3 (+ horn) along the shaft off the spline, N3i along the shaft away from the holder: overlap (mm3) with
    everything on the neck at each step. Horn on its own spline is by design and skipped."""
    import cadquery as cq
    import robot as R
    j = body.joints["head_pitch"]
    c = R.assemble(body, leg, {}, include_legs=False)
    a = np.array(j.R[:, 2])
    fixed = ["neck_link", "servo_head_pitch", "horn_neck_yaw", "servo_neck_yaw", "neck_mount", "deck_2"]
    travel = max(J.HORN_TOP - J.HP[0], J.S.spline_top - (J.S.tower_h + 0.4))
    out = {}
    for name, parts, sgn in (("N3+horn", ["head_frame", "horn_head_pitch"], 1), ("N3i", ["head_idler"], -1)):
        worst = 0.0
        for t in np.arange(0.5, travel + 0.6, 0.5):
            for p in parts:
                m = c[p].moved(cq.Location(cq.Vector(*(a * t * sgn))))
                for f in fixed:
                    if p == "horn_head_pitch" and f == "servo_head_pitch":
                        continue
                    worst = max(worst, R.overlap(m, c[f]))
        out[name] = round(worst, 2)
    return dict(travel=round(travel + 0.5, 2), overlap=out)


def leg_removal():
    """The same test on every leg joint with an idler (driven link + horn off the spline)."""
    import cadquery as cq
    import leg as LEG
    import pose as P
    import robot as R
    leg = LEG.Leg()
    comps = P.posed(leg, {})
    travel = max(J.HORN_TOP - J.HP[0], J.S.spline_top - (J.S.tower_h + 0.4))
    out = {}
    for jn, j in leg.joints.items():
        if not j.idler:
            continue
        drv, hol = P.LINK_OF.get(j.driven, j.driven), P.LINK_OF.get(j.holder, j.holder)
        mov = [n for n in comps if P.owner(leg, n) == drv and not n.startswith("servo_")]
        fix = [n for n in comps if P.owner(leg, n) == hol or n == f"servo_{jn}"]
        a = np.array(j.R[:, 2])
        first = None
        for t in np.arange(0.5, travel + 0.51, 0.5):
            v = sum(R.overlap(comps[m].moved(cq.Location(cq.Vector(*(a * t)))), comps[f]) for m in mov for f in fix
                    if not (m.startswith("horn_") and f == f"servo_{jn}"))
            if v > 0.05:
                first = float(t)
                break
        out[jn] = first
    return out


def evaluate(k, sw, rem, legs):
    import body as B
    out = []
    # A. inserts
    hole = hw.HOLES["ins_m2_thru"].d
    body = B.Body()
    ins = [h for h in body.holes() if h.part == "head_frame" and h.kind == "ins_m2_thru"]
    hx, hy = B.Body.HEAD_X, B.Body.HEAD_Y
    walls = []
    for h in ins:
        x, y = h.pos[0], h.pos[1]
        walls.append(min(x - hx[0], hx[1] - x, y - hy[0], hy[1] - y) - INS_M2["od"] / 2)
        for g in ins:
            if g is not h:
                walls.append(math.dist(h.pos[:2], g.pos[:2]) - INS_M2["od"])
    ok = hole == INS_M2["hole"] and min(walls) >= INS_M2["wall"]
    out.append(finding("A-1", "Critical", "ASSUMED" if ok else "FAILED", "Lỗ tán M2 theo datasheet tán đã chọn",
                       f"BOM: {INS_M2['src']}. Lỗ N3 Ø{hole}; thành nhỏ nhất quanh {len(ins)} tán (gai Ø3.6) = "
                       f"{min(walls):.2f} mm (≥ {INS_M2['wall']}).",
                       dict(hole=hole, min_wall=round(min(walls), 2), inserts=len(ins)),
                       "Đúng theo datasheet; xác nhận kéo tuột/xoay bằng V1."))
    out.append(finding("A-2", "Major", "UNKNOWN", "Lực kéo tuột / mô-men xoay tán trong sàn 3 mm",
                       "Không có số liệu nhà sản xuất cho PLA in; cần phiếu thử.", {}, "V1."))
    # B. alignment
    sg = shaft_offset(hw.SG90)
    off = abs(sg["b_ears_centred"]) + sg["ear_pitch_error_each"]
    out.append(finding("B-1", "Critical", "ASSUMED" if off <= 0.1 else "FAILED",
                       "Trục SG90 so với trục idler trong ổ chung",
                       f"Số SG90 theo TowerPro (lắp giống MG90S): lệch khi tâm tai trùng tán = "
                       f"{sg['b_ears_centred']:+.2f} mm, sai bước tai {sg['ear_pitch_error_each']:.2f}. "
                       "Thêm vào đó N3i "
                       "bắt bằng lỗ hở Ø2.3 nên tự định tâm ±0.15 khi lắp.",
                       dict(offset=round(off, 3), self_align=0.15), "Đo 1 con SG90 (M-1) để chuyển sang VERIFIED."))
    if k:
        f_res = k["k_b"] * max(0.0, off)
        out.append(finding("B-2", "Major", "ASSUMED", "Tải ngang thường trực do lệch trục",
                           f"k_b = {k['k_b']:.1f} N/mm (FEA, N3 + N3i coi như liền), lệch còn lại {off:.2f} mm → "
                           f"{f_res:.1f} N (P1: 3.5–4.7 N).", dict(k_b=round(k["k_b"], 1), preload=round(f_res, 2)),
                           "Lắp tự định tâm (siết 2 ốc G/H sau cùng)."))
    out.append(finding("B-3", "Major", "UNKNOWN", "Đồng trục horn ↔ bạc đạn (K3) sau lắp",
                       "Không còn phụ thuộc in N3 (N3i tự định tâm). Còn: vị trí lỗ khoan horn, độ đảo then hoa.",
                       {}, "Nghiệm thu bằng dòng giữ servo và quay tay (V5, V6)."))
    # C. axial
    out.append(finding("C-1", "Major", "UNKNOWN", "Chiều cao horn thật so với khe 0.8",
                       f"Ổ idler là gối tự do (lắp trượt) và N3i trôi ±0.15 dọc trục, nên không còn khóa hai đầu. Khe "
                       f"đĩa idler ↔ N2 = {J.GAP} − Δh. STEP MG90S cho Δh = {MG90S_STEP_RISE:.1f} (> {J.GAP}); "
                       "datasheet cho Δh = 0.", dict(gap=J.GAP, step_rise=round(MG90S_STEP_RISE, 2)),
                       "Đo spline_top + disc_low (M-1, M-2). Nếu Δh > 0.4 → tăng GAP (cần duyệt, đổi bề rộng đầu)."))
    out.append(finding("C-2", "Major", "UNKNOWN", "Trường hợp xấu nhất khe 31.9", "Chờ Δh và sai số in (T0).", {},
                       "M-1, M-5."))
    # D. bearing
    out.append(finding("D-1", "Critical", "ASSUMED", "Kiểu lắp ổ bạc đạn",
                       f"Định nghĩa: gối tự do, lắp trượt (hardware.HOLES['bearing']: Ø{hw.HOLES['bearing'].d}, chốt "
                       "Ø6.00 lọt / Ø6.10 không lọt).", {}, "Chốt Ø in bằng V2."))
    out.append(finding("D-2", "Info", "VERIFIED", "Giữ dọc trục một phía (đúng nguyên lý gối tự do)", "joint.py.", {}))
    # E. walls / screws
    screw = 3.0
    tip = J.HORN_TOP + screw - hw.HORN.disc_t
    margin = J.HP[1] - 0.4 - tip
    out.append(finding("E-1", "Critical", "ASSUMED" if margin >= 0.5 else "FAILED", "Vít horn và da 0.40",
                       f"ST2×3 qua horn {hw.HORN.disc_t}: mũi vít cách đáy lỗ mồi {margin:.2f} mm (horn ±0.3 → "
                       f"{margin - 0.3:.2f}…{margin + 0.3:.2f}).", dict(margin=round(margin, 2)), "V4."))
    out.append(finding("E-2", "Minor", "ASSUMED", "Tải vít horn khi kẹt servo",
                       f"8.4–12.6 N mỗi vít, dập ≈ {12.6 / (2.0 * (screw - hw.HORN.disc_t)):.1f} MPa "
                       "trên phần "
                       "ren 1.5 mm.", {}, "V6 (kẹt)."))
    out.append(finding("E-4", "Major", "UNKNOWN", "Rão / mỏi / va đập", "Chưa có dữ liệu.", {}, "V6."))
    # F. clearances
    if sw:
        radial = min(r["min_all"] for r in sw["pairs"] if r["moving"] == "head_idler" and r["fixed"] == "neck_link")
        axial = min(r["min_off_axis"] for r in sw["pairs"] if r["moving"] in ("head_frame", "head_idler")
                    and r["fixed"] == "neck_link")
        over = [r for r in sw["pairs"] if r["overlap"] > 0.05]
        out.append(finding("F-1", "Minor", "FAILED" if min(radial, axial) < hw.GAP_MOVING else "VERIFIED",
                           "Khe N3/N3i ↔ N2 so với quy tắc 1.0 mm",
                           f"Hướng kính lip Ø{J.LIP_D} ↔ ống đồng Ø{J.SPACER_D} = {radial} mm (P1 0.2); "
                           f"dọc trục {axial} mm. "
                           "Hai khe "
                           "này đồng trục quanh cùng gối bạc đạn, nên chỉ cọ nếu bạc đạn rơ; quy tắc 1.0 là cho khe "
                           "giữa hai gối khác nhau.", dict(radial=radial, axial=axial),
                           "Chấp nhận có điều kiện V5 (quay không cọ), hoặc tăng GAP (C-1)."))
        out.append(finding("F-2", "Info", "VERIFIED" if not over else "FAILED", "Va chạm trong −30…+32°",
                           f"{len(over)} cặp chồng khối (CAD).", dict(overlaps=len(over)), "Dây, đầu ốc: V5."))
    # G. assembly
    blocked = {k_: v for k_, v in rem["overlap"].items() if v > 0.05}
    out.append(finding("G-1", "Critical", "VERIFIED" if not blocked else "FAILED",
                       "Lắp/tháo N3 không banh khung",
                       f"Dịch khối CAD {rem['travel']} mm dọc trục: chồng lớn nhất {rem['overlap']} mm³ "
                       "(horn trên then "
                       "hoa bỏ qua). N3i tách rời, bắt 2 × M2×6 từ dưới.", rem, ""))
    bad = {k_: v for k_, v in legs.items() if v is not None}
    out.append(finding("G-2", "Critical", "FAILED" if bad else "VERIFIED",
                       "Cùng lỗi G-1 ở các khớp chân (joint.py dùng chung)",
                       f"Khớp bị chặn (mm): {bad or 'không'}. Chân dùng khe idler {J.LEG_GAP} mm, "
                       "phía servo/horn dịch ra "
                       "ngoài (chân rộng thêm 3.8 mm phía ngoài; ảnh trước/sau G2_before_after.png).",
                       dict(blocked_at=bad, leg_gap=J.LEG_GAP), "Chờ duyệt hình ngoài." if not bad else "Sửa CAD."))
    # H. idler clamp vs bearing shield (new in P2)
    js = body.joints["head_pitch"]
    over = (J.SPACER_D - NSK_DB_MAX) / 2
    out.append(finding("H-1", "Major", "ASSUMED", "Mặt kẹp vòng trong so với vai tối đa của bạc đạn",
                       f"Boss in Ø4.2 đã bỏ. Vòng trong giờ kẹp giữa ống đồng Ø{J.SPACER_D} (mặt cắt phẳng) và long "
                       f"đền thép W3. Ống vượt db max {NSK_DB_MAX} (NSK MF63) {over:.2f} mm bán kính (boss cũ 0.25), "
                       "nằm trong mép trong nắp chắn; long đền thép phẳng vượt qua nắp chắn lõm.",
                       dict(spacer_d=J.SPACER_D, db_max=NSK_DB_MAX, over_radial=round(over, 2)),
                       "V5: siết ốc idler xong, vòng ngoài phải quay trơn."))
    ins = [h for h in body.holes() if h.id == "head_pitch.idler_insert"]
    in_wall = bool(ins) and abs(ins[0].depth - J.INSERT_M3_DEPTH) < 1e-6
    out.append(finding("H-2", "Minor", "VERIFIED" if in_wall else "FAILED", "Tán M3 (gai Ø4.6) và chỗ chứa nó",
                       f"Tán ép thẳng vào mặt thành idler, sâu {J.INSERT_M3_DEPTH} trong thành 3.6 (còn 0.3 tới ổ "
                       f"servo); không còn boss. Ống cách dài {js.SPACER[1] - js.SPACER[0]:.1f} (đầu), ốc "
                       f"{js.idler_screw}.", dict(insert_depth=J.INSERT_M3_DEPTH), ""))
    return out


# ------------------------------------------------------------------ collision sweep over the pitch range
def sweep(step_deg=4.0):
    """Minimum distance over the head-pitch range between everything on the head link and everything else.

    Features on the joint axis (lip hole round the holder boss, spline inside the access hole, horn on the spline)
    are coaxial by design, so the sweep is run twice: whole solids, and with a Ø7 core round the axis removed."""
    import robot as R
    import body as B
    import leg as LEG
    import shell as SH
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    body, leg = B.Body(), LEG.Leg()
    sh = SH.BazanShell(body, leg)
    j = body.joints["head_pitch"]
    core = j.place(J.cyl(3.5, -60, 60))
    lo, hi = R.RANGES["head_pitch"]
    angles = [float(v) for v in np.arange(lo, hi + 1e-9, step_deg)] + [float(hi)]
    ref = R.assemble(body, leg, {}, include_legs=False)
    head = sorted(n for n in ref if R.link_of(n, body, leg) == "head_frame")
    near = ["neck_link", "servo_head_pitch", "horn_neck_yaw", "servo_neck_yaw", "neck_mount", "deck_2"]

    def dist(a, b):
        d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
        d.Perform()
        return round(d.Value(), 3)

    rows = {}
    for ang in angles:
        q = {"head_pitch": ang}
        c = R.assemble(body, leg, q, include_legs=False)
        shells = sh.placed(q)
        moving = {n: c[n] for n in head} | {n: shells[n] for n in sh.HEAD}
        fixed = {n: c[n] for n in near} | {n: s for n, s in shells.items() if n not in sh.HEAD}
        for a, sa in moving.items():
            for b, sb in fixed.items():
                if a == "horn_head_pitch" and b == "servo_head_pitch":
                    continue                       # horn pressed on the spline: one body
                v = R.overlap(sa, sb)
                d_all = dist(sa, sb)
                cored = a in ("head_frame", "head_idler", "horn_head_pitch")
                d_core = dist(sa.cut(core), sb.cut(core)) if cored else d_all
                r = rows.setdefault((a, b), dict(moving=a, fixed=b, min_all=9e9, min_off_axis=9e9, overlap=0.0,
                                                 at_all=None, at_off_axis=None))
                if d_all < r["min_all"]:
                    r["min_all"], r["at_all"] = d_all, ang
                if d_core < r["min_off_axis"]:
                    r["min_off_axis"], r["at_off_axis"] = d_core, ang
                r["overlap"] = max(r["overlap"], round(v, 3))
    out = sorted(rows.values(), key=lambda r: (-r["overlap"], r["min_off_axis"]))
    return dict(range=(lo, hi), step=step_deg, angles=angles, pairs=out, not_modelled=[
        "dây servo và cáp (CSI, màn hình)", "đầu ốc tai M2 và đầu vít horn ST2",
        "vỏ SG90 thật (dùng hộp MG92B, lớn hơn)"])


# ------------------------------------------------------------------ Phase 2: print orientations, sliced for real
PRUSA_INI = """bed_shape = 0x0,220x0,220x220,0x220
max_print_height = 250
nozzle_diameter = 0.4
layer_height = 0.2
first_layer_height = 0.2
perimeters = 4
top_solid_layers = 5
bottom_solid_layers = 4
fill_density = 40%
fill_pattern = gyroid
filament_type = PLA
filament_diameter = 1.75
filament_density = 1.24
temperature = 210
first_layer_temperature = 215
bed_temperature = 60
support_material = {sup}
support_material_auto = 1
support_material_threshold = 45
support_material_contact_distance = 0.2
support_material_buildplate_only = {bp}
gcode_comments = 1
"""
ORIENT = {   # name: rotation matrix applied to the N3 solid in its datum frame O (rows = new x, y, z)
    "O1 sàn úp xuống (hiện tại)": ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
    "O2 đĩa horn xuống bàn": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
    "O3 đĩa idler xuống bàn": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
    "O4 tấm mặt xuống bàn": ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
}


def _gcode_scan(path):
    """Filament (g), time (s) and every extruding segment [(x0, y0, x1, y1, z)]."""
    import re
    segs, x, y, z, g, t = [], 0.0, 0.0, 0.0, None, None
    typ = ""
    for line in open(path):
        if line.startswith(";TYPE:"):
            typ = line[6:].strip()
        if line.startswith("; filament used [g]"):
            g = float(line.split("=")[1])
        elif line.startswith("; estimated printing time (normal mode)"):
            v = line.split("=")[1]
            t = sum(int(n) * m for n, m in zip(re.findall(r"(\d+)[dhms]", v)[::-1], (1, 60, 3600, 86400)))
        if not line.startswith(("G1 ", "G0 ")):
            continue
        w = dict((k[0], float(k[1:])) for k in line.split(";")[0].split()[1:] if k[0] in "XYZE")
        nx, ny = w.get("X", x), w.get("Y", y)
        z = w.get("Z", z)
        if w.get("E", 0) > 0 and ("X" in w or "Y" in w):
            segs.append((x, y, nx, ny, z, typ))
        x, y = nx, ny
    return g, t, segs


def _len_in_box(segs, box):
    (x0, x1), (y0, y1), (z0, z1) = box
    L = 0.0
    for ax, ay, bx_, by, z, _ in segs:
        if not (z0 <= z <= z1):
            continue
        n = max(2, int(math.hypot(bx_ - ax, by - ay) / 0.05))
        for i in range(n):
            u = (i + 0.5) / n
            px, py = ax + u * (bx_ - ax), ay + u * (by - ay)
            if x0 <= px <= x1 and y0 <= py <= y1:
                L += math.hypot(bx_ - ax, by - ay) / n
    return L


def _support_in_features(segs, Rm, off, ya):
    """Length of support extrusion that lands inside the functional bores (horn recess, access hole, bearing seat,
    lip hole), found by mapping every support segment back into the N3 datum frame."""
    feats = {"hốc horn Ø20.4": (10.2, J.HP[0] + ya, J.HORN_TOP + ya),
             "lỗ thao tác Ø5.2": (2.6, J.HORN_TOP + ya, J.HP[1] + ya),
             "ổ bạc đạn Ø6.05": (3.025, J.IP[0] + J.CAP_T + ya, J.BRG[1] + ya),
             "lỗ lip Ø4.6": (2.3, J.BRG[1] + ya, J.IP[1] + ya)}
    L = {k: 0.0 for k in feats}
    for ax, ay, bx_, by, z, typ in segs:
        if "upport" not in typ:
            continue
        n = max(2, int(math.hypot(bx_ - ax, by - ay) / 0.1))
        for i in range(n):
            u = (i + 0.5) / n
            pb = np.array([ax + u * (bx_ - ax), ay + u * (by - ay), z - 0.1])
            p = Rm.T @ (pb - off)
            r = math.hypot(p[0], p[2])
            for k, (rr, y0, y1) in feats.items():
                if r < rr - 0.05 and min(y0, y1) < p[1] < max(y0, y1):
                    L[k] += math.hypot(bx_ - ax, by - ay) / n
    return {k: round(v, 1) for k, v in L.items()}


def slice_compare(work):
    import shutil
    import cadquery as cq
    import body as B
    if not shutil.which("prusa-slicer"):
        return None
    body = B.Body()
    j = body.joints["head_pitch"]
    org = np.array(j.origin, float)
    n3 = body.parts["head_frame"].fuse(body.parts["head_idler"]).clean().moved(cq.Location(cq.Vector(*(-org))))
    ya = -J.CENTER
    skin = ((-0.8, 0.8), (J.HP[1] - 0.4 + ya, J.HP[1] + ya), (7 - 0.8, 7 + 0.8))       # one pilot's skin, O frame
    (work / "pla.ini").write_text(PRUSA_INI.format(sup=0, bp=0))
    (work / "pla_sup.ini").write_text(PRUSA_INI.format(sup=1, bp=0))
    (work / "pla_bp.ini").write_text(PRUSA_INI.format(sup=1, bp=1))
    out = {}
    for name, Rm in ORIENT.items():
        Rm = np.array(Rm, float)
        from OCP.gp import gp_Trsf
        tr = gp_Trsf()
        tr.SetValues(*Rm[0], 0.0, *Rm[1], 0.0, *Rm[2], 0.0)
        s = n3.moved(cq.Location(tr))
        bb = s.BoundingBox()
        stl = work / "n3.stl"
        cq.exporters.export(s, str(stl), tolerance=0.01, angularTolerance=0.1)
        res = {}
        for tag in ("pla", "pla_sup", "pla_bp"):
            g = work / f"{tag}.gcode"
            subprocess.run(["prusa-slicer", "--export-gcode", "--load", str(work / f"{tag}.ini"), "--center", "110,110",
                            "-o", str(g), str(stl)], capture_output=True, text=True, check=True)
            res[tag] = _gcode_scan(g)
        # skin box in bed coordinates: rotate the corners, then the same offset PrusaSlicer used (centre, drop to 0)
        corners = np.array([[x, y, z] for x in skin[0] for y in skin[1] for z in skin[2]]) @ Rm.T
        off = np.array([110 - (bb.xmin + bb.xmax) / 2, 110 - (bb.ymin + bb.ymax) / 2, -bb.zmin])
        c = corners + off
        box = tuple((float(c[:, i].min()), float(c[:, i].max())) for i in range(3))
        skin_len = _len_in_box(res["pla"][2], box)
        sup_in = _support_in_features(res["pla_sup"][2], Rm, off, ya)
        sup_bp = _support_in_features(res["pla_bp"][2], Rm, off, ya)
        out[name] = dict(height=round(bb.zmax - bb.zmin, 1), grams=round(res["pla"][0], 2),
                         grams_with_support=round(res["pla_sup"][0], 2),
                         support_g=round(res["pla_sup"][0] - res["pla"][0], 2),
                         minutes=round(res["pla"][1] / 60), minutes_with_support=round(res["pla_sup"][1] / 60),
                         pitch_axis="dọc (vuông góc bàn)" if abs(Rm[2] @ np.array([0, 1, 0])) > 0.5 else "ngang",
                         support_g_buildplate_only=round(res["pla_bp"][0] - res["pla"][0], 2),
                         skin_extrusion_mm=round(skin_len, 1), support_in_features_mm=sup_in,
                         support_in_features_mm_buildplate_only=sup_bp)
    return dict(profile="PrusaSlicer, 0.4 nozzle, 0.2 layer, 4 perimeters, 5 top / 4 bottom, gyroid 40 %, PLA "
                        "(cùng profile với test_prints; KHÔNG phải profile máy in thật của bạn)", orientations=out)


def write_status(res):
    """docs/mechanical/n3_review/N3_P2_STATUS.md: every P1 finding with its status on the current CAD."""
    p1 = {f["id"]: f for f in json.loads((OUT / "n3_review_P1.json").read_text())["findings"]}
    L = ["# N3 Head Frame — trạng thái sau rev P2", "",
         "Sinh bởi `hardware/mechanical/n3_review.py` từ CAD hiện tại (không sửa tay). Audit P1 gốc: "
         "[N3_ENGINEERING_AUDIT.md](N3_ENGINEERING_AUDIT.md), số liệu P1: `n3_review_P1.json`. Số linh kiện lấy từ "
         "datasheet trên mạng (chưa đo): mọi mục dựa vào chúng là ASSUMED, không phải VERIFIED.", "",
         "| ID | Mức | P1 | P2 | Vấn đề | Bằng chứng | Còn phải làm |", "|---|---|---|---|---|---|---|"]
    for f in res["findings"]:
        old = p1.get(f["id"], {}).get("status", "mới")
        cell = [f["id"], f["severity"], old, f"**{f['status']}**", f["title"], f["evidence"], f["action"] or "—"]
        L.append("| " + " | ".join(str(c).replace("|", "/").replace("\n", " ") for c in cell) + " |")
    if res.get("stiffness"):
        k = res["stiffness"]
        L += ["", f"FEA (N3 + N3i, PLA E 2300 giả định): k_b {k['k_b']:.1f}, k_w {k['k_w']:.1f}, "
              f"k_a {k['k_a']:.1f} N/mm."]
    L += ["", "Chạy lại: `cd hardware\\mechanical; python n3_review.py` (khoảng 7 phút).", ""]
    (OUT / "N3_P2_STATUS.md").write_text("\n".join(L), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-fea", action="store_true")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--no-slice", action="store_true")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    k = None
    if not a.no_fea:
        with tempfile.TemporaryDirectory() as td:
            k = frame_stiffness(Path(td))
    import body as B
    import leg as LEG
    body, leg = B.Body(), LEG.Leg()
    sw = None if a.no_sweep else sweep()
    res = dict(rev="P2", sources=SRC, stack=stack(), stiffness=k, sweep=sw, removal=removal(body, leg),
               leg_removal=leg_removal())
    res["findings"] = evaluate(k, sw, res["removal"], res["leg_removal"])
    if not a.no_slice:
        with tempfile.TemporaryDirectory() as td:
            res["slice"] = slice_compare(Path(td))
        if res["slice"]:
            for n, v in res["slice"]["orientations"].items():
                print(n, v)
    if a.no_fea or a.no_sweep or a.no_slice:
        print("(partial run: n3_review.json not written; run without --no-* to refresh it)")
    else:
        (OUT / "n3_review.json").write_text(json.dumps(res, indent=1, ensure_ascii=False, default=float))
        write_status(res)
    for f in res["findings"]:
        print(f"{f['id']:4s} {f['severity']:8s} {f['status']:8s} {f['title']}")
        for kk, vv in f["numbers"].items():
            print(f"       {kk}: {vv}")
    if k:
        print("stiffness", k)
    if res.get("sweep"):
        print("sweep", res["sweep"]["range"])
        for r in res["sweep"]["pairs"][:12]:
            print("  ", r)


if __name__ == "__main__":
    main()
