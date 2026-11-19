"""Part and assembly drawings of the head (H) from the CAD code, on the same engine as the N3 drawing.

    python drawings_parts.py N2 N4 C3 C4 C5 H0     # write docs/mechanical/drawings/<code>/
    python drawings_parts.py all --check           # every head drawing; exit 1 on any validation defect

A part drawing has: sheet 1 front / top / left views (first angle, ISO 5456-2), an isometric view, overall sizes and
the spacing of every group of identical holes on the view that sees them as circles; sheet 2 sections through the
middle of the part and the feature table (every hole of the part in the CAD, with its coordinates from the head
datum O). The assembly drawing H0 has the assembled views, an exploded view with balloons, the parts list and the
fastener list taken from the same hole records.

Every number comes from the CAD. The checks (written next to each drawing) look again at the CAD, not at the
drawing: overall sizes against the solid's exact bounding box, each feature's hole against the solid (a point
inside the hole must be outside the material), the feature table against the hole records, the view alignment and
scale, text inside the frame and not overlapping, the title block, and for H0 that every part is ballooned once
and every quantity matches the CAD.
"""
import json
import sys
import textwrap
from datetime import date

import cadquery as cq
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_OUT

import drawings as D
from drawings import DEPTH, FS_TITLE, INK, W_THICK, W_THIN, Sheet, fmt, fmt2, hlr, project, section

ISO_SCALES = (5.0, 2.0, 1.0, 0.5)
OWNER = "Hau Nguyen Ngoc"
STANDARDS = "ISO 128 · ISO 129-1 · ISO 5455 · ISO 5456-2 · ISO 5457 · ISO 7200 (tham chiếu)"
STATUS = "SƠ BỘ — chưa phát hành chế tạo"

KIND_TEXT = {  # hole kind: what it is on the drawing
    "ins_m2": "lỗ tán cấy nhiệt M2 (L3, gai Ø3.6), mù",
    "ins_m2_thru": "lỗ tán cấy nhiệt M2 (L3, gai Ø3.6), xuyên",
    "ins_m3": "lỗ tán cấy nhiệt M3 short (L3, gai Ø4.6), mù",
    "pilot_st2": "lỗ mồi vít tự ren ST2×3 (bắt horn), mù",
    "access": "lỗ thao tác tới ốc tâm horn (tua vít PH0)",
    "bearing": "ổ bạc đạn MR63ZZ, lắp trượt (gối tự do)",
    "cb_m2": "lỗ hở M2 + bậc chìm đầu ốc",
    "clr_m2": "lỗ hở M2",
    "boss_m2": "trụ tán cấy M2 (lỗ mù)",
    "tie": "khe dây rút",
    "tube": "ống dẫn ốc từ đỉnh mũ (tua vít đi qua)",
    "window": "cửa sổ quang học",
    "pocket": "ổ servo (thân SG90/MG92B), mở về phía trục",
    "recess": "hốc horn tròn (đĩa horn nằm phẳng đáy hốc)",
    "cable": "rãnh luồn dây + giắc servo ở cuối ổ",
}


def joint_feats(j, role, org):
    """Pocket and cable window of a holder, horn recess of a driven link: functional features that are not screw
    holes, with their size from joint.py (local frame: b along the servo, w across, a along the shaft)."""
    import hardware as hw
    import joint as J
    out = []
    if role == "holder":
        S, C = J.S, J.C
        L, W = J.CB - J.CB0 + 2 * C, S.width + 2 * C
        depth = 0.0 - (J.CASE[0] - C)
        out.append(dict(id=f"{j.name}.pocket", kind="pocket", d=W, depth=depth, size=f"{fmt(W)} × {fmt(L)} {DEPTH}"
                        f"{fmt(depth)}", dir=-np.asarray(j.wdir((0, 0, 1))),
                        pos=np.asarray(j.world(((J.CB0 + J.CB) / 2, 0, 0.0)), float) - org,
                        mate=f"servo {j.servo.name} (khe {C} mỗi bên)"))
        wa = J.WINDOW
        zc = J.CASE[0] + (S.cable_z[0] + S.cable_z[1]) / 2
        out.append(dict(id=f"{j.name}.cable", kind="cable", d=wa[0], depth=0.0, size=f"{fmt(wa[0])} × {fmt(wa[1])} "
                        "xuyên", dir=np.asarray(j.wdir((1, 0, 0))),
                        pos=np.asarray(j.world((J.CB + C + 0.1, 0, zc)), float) - org, mate="dây + giắc 3 chân"))
    else:
        d = hw.HORN.disc_d + 0.4
        depth = J.HORN_TOP - J.HP[0]
        out.append(dict(id=f"{j.name}.recess", kind="recess", d=d, depth=depth, size=f"Ø{fmt(d)} {DEPTH}{fmt(depth)}",
                        dir=np.asarray(j.wdir((0, 0, 1))), pos=np.asarray(j.world((0, 0, J.HP[0])), float) - org,
                        mate=f"horn tròn {j.servo.name} (Ø{fmt(hw.HORN.disc_d)})"))
    return out


# ------------------------------------------------------------------ geometry helpers
def exact_bbox(s):
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(s.wrapped, b, False, False)
    x0, y0, z0, x1, y1, z1 = b.Get()
    return np.array([x0, y0, z0]), np.array([x1, y1, z1])


def outside(s, p):
    c = BRepClass3d_SolidClassifier(s.wrapped, gp_Pnt(*map(float, p)), 1e-6)
    return c.State() == TopAbs_OUT


def view_of_dir(d):
    """The principal view that sees a hole with axis d as a circle."""
    i = int(np.argmax(np.abs(d)))
    return ("front", "left", "top")[i]


def choose_scale(size, room):
    for k in ISO_SCALES:
        if all(k * s <= r for s, r in zip(size, room)):
            return k
    return ISO_SCALES[-1]


def ratio(k):
    return f"{fmt(k)}:1" if k >= 1 else f"1:{fmt(1 / k)}"


# ------------------------------------------------------------------ the parts of the head, in the head datum frame
def head_parts():
    """{code: dict(name, title, solid, holes, material, print, notes)} with solids moved to the datum O."""
    import body as B
    import gen_manual as GM
    import joint as J
    import leg as LEG
    import shell as SH
    body = B.Body()
    sh = SH.BazanShell(body, LEG.Leg())
    org = np.array(body.joints["head_pitch"].origin, float)
    mv = cq.Location(cq.Vector(*(-org)))
    out = {}

    def holes_of(part):
        rows = []
        for h in body.holes():
            if h.part == part:
                rows.append(dict(id=h.id, kind=h.kind, d=h.d, depth=h.depth, dir=np.round(np.asarray(h.dir, float), 6),
                                 pos=np.asarray(h.pos, float) - org, mate=h.fastener))
        return rows

    for name, code in (("neck_link", "N2"), ("cam_mount", "N4")):
        c, title, ref, pr = GM.BODY_PARTS[name]
        hs = holes_of(name)
        for j in body.joints.values():
            if j.holder == name:
                hs += joint_feats(j, "holder", org)
            if j.driven == name:
                hs += joint_feats(j, "driven", org)
        out[code] = dict(name=name, title=f"{code} {title}", solid=body.parts[name].moved(mv), holes=hs,
                         material="PLA", print=pr, ref=ref)
    # N2: what moved in rev P2 of the idler (no boss, insert in the wall face, brass spacer)
    j = body.joints["head_pitch"]
    out["N2"]["notes"] = [
        f"Tán M3 short ép từ MẶT NGOÀI thành idler, sâu {J.INSERT_M3_DEPTH} (còn 0.3 tới ổ servo); không còn boss in "
        f"(rev trước: boss Ø4.2 chứa tán gai Ø4.6). Vòng trong MR63ZZ kẹp bằng ống đồng Ø{J.SPACER_D} × 0.45 dài "
        f"{j.SPACER[1] - j.SPACER[0]:.1f} + W3 + {j.idler_screw} đầu dù.",
        "Ổ servo head pitch dựng cho thân MG92B/MG90S (SG90 chính hãng cùng kích thước lắp). Khoan lỗ tai SG90 "
        "lên Ø2.3 trước khi lắp (servo_fit.py).",
        "Đĩa horn neck yaw: horn tròn + 2 × ST2×3 từ phía servo, như mọi khớp (joint.py)."]
    out["N4"]["notes"] = [
        "Mặt camera và ToF nghiêng 15° xuống dưới (trục nhìn N = (cos15°, 0, −sin15°)).",
        "4 trụ tán M2 (B) giữ module camera và ToF; 2 lỗ bậc (cb_m2) bắt N4 xuống tán B của N3 bằng M2×6 từ trên."]
    # shells: clearance holes from shell.screws, guide tubes from the crown, the visor's optical windows
    shell_meta = {"head_front": ("C3", "Head Shell Front", "PLA"), "head_back": ("C4", "Head Shell Back", "PLA"),
                  "visor": ("C5", "Visor", "PETG trong suốt")}
    for name, (code, title, mat) in shell_meta.items():
        s = sh.parts[name]
        holes = []
        for k, (part, p, d) in enumerate([x for x in sh.screws if x[0] == name]):
            p = np.asarray(p, float) - org
            holes.append(dict(id=f"{name}.tab{k + 1}", kind="clr_m2", d=2 * SH.CLR_M2, depth=0.0,
                              dir=np.asarray(d, float), pos=p, mate="M2×6 vào tán A của N3 (thả qua ống từ đỉnh)"))
            top = sh.parts[name].BoundingBox().zmax - org[2]
            holes.append(dict(id=f"{name}.tube{k + 1}", kind="tube", d=2 * SH.TUBE_R[0], depth=0.0,
                              dir=np.array([0, 0, -1.0]), pos=np.array([p[0], p[1], top]),
                              mate="lục giác 1.5 / ốc M2 đi qua"))
        if name == "visor":
            for lab, c, r in (("cam", SH.CAM_C, SH.CAM_HOLE), ("tof", SH.TOF_C, SH.TOF_HOLE)):
                holes.append(dict(id=f"visor.{lab}", kind="window", d=2 * r, depth=0.0, dir=-np.asarray(SH.N, float),
                                  pos=np.asarray(c, float) - org + 40.0 * 0 * np.asarray(SH.N, float),
                                  mate="camera OV5647" if lab == "cam" else "ToF VL53L0X"))
        notes = {"C3": ["Mũ trước: 2 tab trên sàn đầu, ống dẫn ốc lên đỉnh mũ, tai có lưới mic, cửa sổ kính (C5 dán "
                        "từ trong).", "Thành 1.2 mm = 3 đường đùn đầu 0.4; khe 0.4 giữa các mép ghép."],
                 "C4": ["Mũ sau: 2 tab trên sàn đầu, rãnh cổ quanh trục gật để mũ không chạm servo cổ "
                        "khi gật −30…+32°.",
                        "Thành 1.2 mm; mép ghép với C3 có lưỡi gà (seam tongue) thuộc C3."],
                 "C5": ["PETG trong, dán từ trong lên gờ 2.5 mm của C3; cửa sổ để trống cho camera và ToF (không nhìn "
                        "qua nhựa).", "Không có lỗ ốc."]}[code]
        out[code] = dict(name=name, title=f"{code} {title}", solid=s.moved(mv), holes=holes, material=mat,
                         print="mặt ngoài lớn nhất lên trên, support chỉ từ bàn" if code != "C5"
                         else "mặt trong úp xuống bàn, không support", ref="trục gật đầu (O)", notes=notes)
    for code in ("N2", "N4"):
        out[code].setdefault("notes", [])
    return out, org


# ------------------------------------------------------------------ feature groups
def features(holes):
    """Group identical holes (kind, size, depth, direction) under one letter; number the members."""
    keyf = lambda h: (h["kind"], round(h["d"], 2), round(h["depth"], 2), tuple(np.round(h["dir"], 3)))  # noqa: E731
    groups = {}
    for h in sorted(holes, key=lambda h: (h["kind"], h["id"])):
        groups.setdefault(keyf(h), []).append(h)
    feats = []
    for gi, (k, mem) in enumerate(groups.items()):
        letter = "ABCDEFGHJKLMNPQRSTUVWXYZ"[gi]
        mem.sort(key=lambda h: tuple(np.round(h["pos"], 2)))
        for i, h in enumerate(mem):
            feats.append(dict(h, tag=f"{letter}{i + 1}", group=letter, what=KIND_TEXT.get(h["kind"], h["kind"])))
    return feats


def dir_text(d):
    """Axis name, or the nearest axis and the angle off it (the camera face is tilted 15°)."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    i = int(np.argmax(np.abs(d)))
    name = ("+" if d[i] > 0 else "−") + "XYZ"[i]
    ang = np.degrees(np.arccos(min(1.0, abs(d[i]))))
    return name if ang < 0.5 else f"{name} {ang:.0f}°"


def size_text(ft):
    if "size" in ft:
        return ft["size"]
    if ft["kind"] == "tie":
        return "1.60 × 3.00 xuyên"
    if ft["depth"]:
        return f"Ø{ft['d']:.2f} {DEPTH}{ft['depth']:.2f}"
    return f"Ø{ft['d']:.2f} xuyên"


# ------------------------------------------------------------------ part drawing
def draw_part(code, P, rev="P1"):
    s, holes = P["solid"], P["holes"]
    feats = features(holes)
    lo, hi = exact_bbox(s)
    size = hi - lo
    c3 = (lo + hi) / 2
    # rooms: front (Y wide, Z tall), top (Y wide, X tall), left (X wide, Z tall)
    K = choose_scale((size[1], size[2], size[0]), (125.0, 90.0, 80.0))
    tb = dict(owner=OWNER, project="Bazan Humanoid — cụm đầu (H)", title=P["title"], number=f"BZN-H-{code}",
              material=P["material"], mass=f"{part_mass(P):.1f} g (ước tính)", scale=ratio(K), designed=OWNER,
              checked="TBD", approved="TBD", date=date.today().isoformat(), rev=rev, status=STATUS,
              standards=STANDARDS)
    sheets = []
    s1 = Sheet(1, "views")
    fx = 30 + 30 + K * size[1] / 2
    fy = 287 - 32 - K * size[2] / 2
    ty = fy - K * size[2] / 2 - 42 - K * size[0] / 2
    lx = fx + K * size[1] / 2 + 52 + K * size[0] / 2
    fc, tc, lc = project(c3, "front"), project(c3, "top"), project(c3, "left")
    s1.place("front", "front", fx, fy, fc[0], fc[1], K)
    s1.place("top", "top", fx, ty, fc[0], tc[1], K)
    s1.place("left", "left", lx, fy, lc[0], fc[1], K)
    for nm in ("front", "top", "left"):
        vis, hid = hlr(s, nm)
        s1.draw_edges(nm, hid, w=W_THIN, ls=(0, (3, 1.5)))
        s1.draw_edges(nm, vis, w=W_THICK)
    # overall sizes from the exact bounding box (the extreme points are not always on a face: on_surface False)
    s1.dim("front", (c3[0], lo[1], hi[2]), (c3[0], hi[1], hi[2]), (0, 1, 0), "above", on_surface=False)
    s1.dim("front", (c3[0], lo[1], lo[2]), (c3[0], lo[1], hi[2]), (0, 0, 1), "left", on_surface=False)
    s1.dim("left", (lo[0], c3[1], hi[2]), (hi[0], c3[1], hi[2]), (1, 0, 0), "above", on_surface=False)
    # each group: centre marks, spacing along the two paper axes, and the first member from the box edge
    seen = set()
    for g in sorted({ft["group"] for ft in feats}):
        mem = [ft for ft in feats if ft["group"] == g]
        vw = view_of_dir(mem[0]["dir"])
        for ft in mem:
            if ft["kind"] not in ("tie", "pocket", "cable"):
                s1.centermark(vw, ft["pos"], ft["d"] / 2)
        axes = {"front": (1, 2), "left": (0, 2), "top": (1, 0)}[vw]
        sides = {"front": ("below", "right"), "left": ("below", "right"), "top": ("below", "right")}[vw]
        for ax_i, side in zip(axes, sides):
            vals = sorted({round(ft["pos"][ax_i], 3) for ft in mem})
            e = np.eye(3)[ax_i]
            if len(vals) > 1 and (vw, ax_i, vals[0], vals[-1]) not in seen:
                seen.add((vw, ax_i, vals[0], vals[-1]))
                p0 = mem[0]["pos"].copy()
                p0[ax_i] = vals[0]
                p1 = p0.copy()
                p1[ax_i] = vals[-1]
                s1.dim(vw, p0, p1, e, side, on_surface=False)
            key = (vw, ax_i, vals[0])
            if key not in seen and len(seen) < 40:
                seen.add(key)
                p0 = mem[0]["pos"].copy()
                p0[ax_i] = lo[ax_i]
                p1 = mem[0]["pos"].copy()
                p1[ax_i] = vals[0]
                if abs(vals[0] - lo[ax_i]) > 0.3:
                    s1.dim(vw, p0, p1, e, side, on_surface=False)
    for ft in feats:
        s1.tag(view_of_dir(ft["dir"]), ft["pos"], ft["tag"])
    for nm, t in (("front", "Hình chiếu đứng (nhìn từ +X)"), ("left", "Hình chiếu cạnh (nhìn từ −Y)"),
                  ("top", "Hình chiếu bằng (nhìn từ +Z)")):
        s1.caption(nm, t, ratio(K))
    iv, _ = hlr(s, "iso")
    ic = project(c3, "iso")
    corners = np.array([project((x, y, z), "iso") for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                        for z in (lo[2], hi[2])])
    span = np.ptp(corners, axis=0)
    room_h = 150.0 - 66.0                              # between the title block (y 58) and the legend
    ki = min(K, 100.0 / span[0], room_h / span[1])
    s1.place("iso", "iso", 355, 66 + ki * span[1] / 2, ic[0], ic[1], ki)
    s1.draw_edges("iso", iv, w=W_THICK * 0.8)
    s1.caption("iso", "Hình trục đo", "minh hoạ, không đo")
    # legend
    xl, yy = 302, 281
    s1.text(xl, yy, "Ký hiệu đặc tính", fs=9, role="head", weight="bold", va="top")
    yy -= 7
    for g in sorted({ft["group"] for ft in feats}):
        mem = [ft for ft in feats if ft["group"] == g]
        s1.text(xl, yy, f"{g}1–{g}{len(mem)}" if len(mem) > 1 else f"{g}1", fs=7.2, role="legend", va="top",
                weight="bold")
        s1.text(xl + 13, yy, f"{len(mem)}× {size_text(mem[0])}", fs=7.2, role="legend", va="top")
        s1.text(xl + 13, yy - 3.4, textwrap.shorten(mem[0]["what"], 52, placeholder="…"), fs=6.4, role="legend",
                va="top")
        yy -= 7.6
    s1.text(xl, yy - 2, "Gốc toạ độ", fs=9, role="head", weight="bold", va="top")
    s1.text(xl, yy - 8, "O = trục gật đầu ∩ mặt phẳng giữa chồng khớp,\ntư thế 0 (chung cho mọi bản vẽ cụm đầu).\n"
            "X tới trước, Y sang trái, Z lên.\nKích thước bao: hộp bao chính xác của chi tiết.\n"
            "Toạ độ từng đặc tính: tờ 2.", fs=7.0, role="note", va="top")
    sheets.append(s1)

    # sheet 2: sections through the middle, the feature table and notes
    s2 = Sheet(2, "sections")
    Ks = min(K, choose_scale((size[1], size[2], size[0]), (110.0, 70.0, 70.0)))
    av, apol, _ = section(s, c3, "front")
    bv, bpol, _ = section(s, c3, "left")
    s2.place("A", "front", 30 + 25 + Ks * size[1] / 2, 287 - 28 - Ks * size[2] / 2, fc[0], fc[1], Ks)
    s2.place("B", "left", 30 + 25 + Ks * size[1] + 50 + Ks * size[0] / 2, 287 - 28 - Ks * size[2] / 2, lc[0],
             fc[1], Ks)
    for nm, v, pol in (("A", av, apol), ("B", bv, bpol)):
        s2.draw_edges(nm, v)
        s2.hatch(nm, pol)
    s2.caption("A", f"Mặt cắt A–A (X = {fmt(c3[0])})", ratio(Ks))
    s2.caption("B", f"Mặt cắt B–B (Y = {fmt(c3[1])})", ratio(Ks))
    s2.text(302, 281, "Ghi chú mặt cắt", fs=9, role="head", weight="bold", va="top")
    s2.text(302, 274, "Mặt phẳng cắt qua tâm hộp bao.\nKhông vẽ nét khuất; gạch 45°, nét mảnh.", fs=7.0, role="note",
            va="top")
    ytab = min(s2.bbox["A"][1], s2.bbox["B"][1]) - 14
    cols = [("Mã", 10), ("Đặc tính", 92), ("Kích thước", 30), ("Hướng", 12), ("X", 15), ("Y", 15), ("Z", 15),
            ("Lắp với", 120), ("ID CAD", 30)]
    xs = np.cumsum([0] + [w for _, w in cols]) + 25.0
    rh = 5.2
    rows = [[ft["tag"], ft["what"], size_text(ft), dir_text(ft["dir"]), fmt2(ft["pos"][0]), fmt2(ft["pos"][1]),
             fmt2(ft["pos"][2]), textwrap.shorten(ft["mate"], 70, placeholder="…"), ft["id"].split(".", 1)[-1]]
            for ft in feats]
    s2.text(25, ytab + 6, f"BẢNG ĐẶC TÍNH — {P['title']} (toạ độ từ O, mm)", fs=FS_TITLE * 0.85, role="head",
            weight="bold", va="bottom")
    for r, row in enumerate([[c for c, _ in cols]] + rows):
        y = ytab - r * rh
        for c, val in enumerate(row):
            s2.ax.add_patch(plt.Rectangle((xs[c], y - rh), xs[c + 1] - xs[c], rh, fill=r == 0, facecolor="#e8e8e8",
                                          edgecolor=INK, lw=W_THIN))
            num = 4 <= c <= 6 and r > 0
            s2.text(xs[c + 1] - 1.0 if num else xs[c] + 1.0, y - rh / 2, str(val), fs=6.2, role="table", va="center",
                    ha="right" if num else "left", weight="bold" if r == 0 else "normal")
    yy = ytab - (len(rows) + 1) * rh - 6
    notes = [("VẬT LIỆU / IN", [f"M1. {P['material']}. In: {P['print']}. Thông số máy in, bù lỗ XY: TBD (phiếu T0)."]),
             ("GHI CHÚ", P["notes"] + ["Kích thước không ghi dung sai: TBD theo kết quả phiếu thử T0 (không áp "
                                       "ISO 2768 khi chưa có số đo máy in)."])]
    for head, items in notes:
        s2.text(25, yy, head, fs=8.0, role="head", weight="bold", va="top")
        yy -= 4.8
        for it in items:
            w = textwrap.wrap(it, 120)
            s2.text(28, yy, "\n".join(w), fs=6.8, role="note", va="top", linespacing=1.3)
            yy -= 3.2 * len(w) + 1.2
    sheets.append(s2)
    for sh in sheets:
        sh.furniture(tb, len(sheets))
    return dict(code=code, sheets=sheets, solid=s, feats=feats, holes=holes, tb=tb, bbox=(lo, hi), scale=K)


def part_mass(P):
    import audit as A
    if P["material"].startswith("PETG"):
        return 1.27e-3 * P["solid"].Volume()
    return A.printed_g(P["solid"], "arm")


# ------------------------------------------------------------------ validation
def validate(res):
    rep, bad = [], 0

    def item(name, ok, detail):
        nonlocal bad
        bad += not ok
        rep.append(dict(check=name, ok=bool(ok), detail=detail))

    sheets = res["sheets"]
    if res.get("solid") is not None:
        s, (lo, hi) = res["solid"], res["bbox"]
        size = hi - lo
        for sh in sheets:
            for d in sh.dims:
                p0, p1 = np.array(d.p0), np.array(d.p1)
                measured = abs(np.dot(p1 - p0, d.axis))
                shown = float("".join(ch for ch in d.text.split(" ")[0] if ch in "0123456789."))
                item(f"tờ {sh.number} [{d.view}] {d.text} = giá trị trên CAD", abs(measured - shown) < 0.006,
                     f"{measured:.3f}")
        shown = sorted(float(d.text) for d in sheets[0].dims[:3])
        item("kích thước bao = hộp bao chính xác của khối đặc",
             np.allclose(sorted(np.round(size, 2)), shown, atol=0.006), f"CAD {np.round(size, 2)}, ghi {shown}")
        # every feature is a real hole in the solid
        miss = []
        for ft in res["feats"]:
            dvec = ft["dir"] / np.linalg.norm(ft["dir"])
            depth = ft["depth"] or 1.0
            ok = any(outside(s, ft["pos"] + dvec * t) for t in (0.3 * depth, 0.5 * depth, 0.8 * depth))
            if not ok:
                miss.append(ft["tag"])
        item("mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc", not miss, ", ".join(miss) or
             f"{len(res['feats'])} đặc tính")
        tags = [ft["tag"] for ft in res["feats"]]
        item("mã đặc tính không trùng", len(tags) == len(set(tags)), ", ".join(tags))
        item("bảng = mọi lỗ của chi tiết trong CAD", sorted(ft["id"] for ft in res["feats"])
             == sorted(h["id"] for h in res["holes"]), f"{len(res['feats'])}")
        s1 = sheets[0]
        a, b, c = s1.P("front", (0, 0, 0)), s1.P("top", (0, 0, 0)), s1.P("left", (0, 0, 0))
        item("thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm)", abs(a[0] - b[0]) < 1e-6, f"{a[0] - b[0]:.1e}")
        item("thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm)", abs(a[1] - c[1]) < 1e-6, f"{a[1] - c[1]:.1e}")
        F, T, L = s1.bbox["front"], s1.bbox["top"], s1.bbox["left"]
        item("góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau",
             T[3] < F[1] and L[0] > F[2], f"bằng y1 {T[3]:.0f} < đứng y0 {F[1]:.0f}; cạnh x0 {L[0]:.0f} > đứng x1 "
             f"{F[2]:.0f}")
    tbx = (D.BORDER[2] - 180.0, D.BORDER[1], D.BORDER[2], D.BORDER[1] + 48.0)
    for sh in sheets:
        for nm, bb in sh.bbox.items():
            inside = bb[0] >= D.BORDER[0] and bb[2] <= D.BORDER[2] and bb[1] >= D.BORDER[1] and bb[3] <= D.BORDER[3]
            clear = bb[2] < tbx[0] or bb[1] > tbx[3]
            item(f"tờ {sh.number} hình {nm}: trong khung, không chạm khung tên", inside and clear,
                 f"[{bb[0]:.0f}, {bb[1]:.0f}, {bb[2]:.0f}, {bb[3]:.0f}]")
        for nm, (cx, cy, uc, vc, k, mv) in sh.views.items():
            if mv == "iso" or nm.startswith("x"):
                continue
            p, q = sh.P(nm, (0, 0, 0)), sh.P(nm, (0, 10, 0) if mv != "left" else (10, 0, 0))
            r = np.linalg.norm(q - p) / 10
            item(f"tỉ lệ tờ {sh.number} hình {nm} là tỉ lệ ISO 5455", abs(r - k) < 1e-9 and k in ISO_SCALES,
                 ratio(k))
        sh.fig.canvas.draw()
        rnd = sh.fig.canvas.get_renderer()
        boxes = []
        for t, role in sh.texts:
            if t.get_text().strip():
                boxes.append((t.get_window_extent(rnd).transformed(sh.ax.transData.inverted()), t.get_text()[:28],
                              role))
        X0, Y0, X1, Y1 = D.BORDER
        out = [t for bb, t, role in boxes if role != "zone" and not (bb.x0 >= X0 - .01 and bb.x1 <= X1 + .01 and
                                                                     bb.y0 >= Y0 - .01 and bb.y1 <= Y1 + .01)]
        item(f"tờ {sh.number}: chữ nằm trong khung", not out, "; ".join(out[:5]) or "ok")
        hits = []
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                b1, b2 = boxes[i][0], boxes[j][0]
                if b1.x0 < b2.x1 - 0.2 and b2.x0 < b1.x1 - 0.2 and b1.y0 < b2.y1 - 0.2 and b2.y0 < b1.y1 - 0.2:
                    hits.append(f"'{boxes[i][1]}' × '{boxes[j][1]}'")
        item(f"tờ {sh.number}: chữ không chồng nhau", not hits, "; ".join(hits[:6]) or "ok")
    tb = res["tb"]
    empty = [k for k, v in tb.items() if not str(v).strip()]
    item("khung tên đủ trường", not empty, ", ".join(empty) or "đủ")
    item("đánh số tờ liên tục", [sh.number for sh in sheets] == list(range(1, len(sheets) + 1)), "")
    for extra in res.get("checks", []):
        item(*extra)
    return rep, bad


# ------------------------------------------------------------------ H0: the head assembly
def draw_h0(parts, org):
    import body as B
    import leg as LEG
    import robot as R
    import shell as SH
    body = B.Body()
    sh = SH.BazanShell(body, LEG.Leg())
    mv = cq.Location(cq.Vector(*(-org)))
    comps = R.assemble(body, LEG.Leg(), {}, include_legs=False)
    bought = {"servo_head_pitch": ("SG90", "Servo SG90 (head pitch)", "mua"),
              "horn_head_pitch": ("HORN", "Horn tròn SG90", "mua, kèm servo"),
              "spacer_head_pitch": ("TUBE4", "Ống đồng Ø4 × 0.45, dài 1.3", "mua, cắt"),
              "board_DISP": ("DISP", "Module màn hình (mặt)", "mua")}
    items = [  # (item, key, code, name, qty, spec)
        ("neck_link", "N2", parts["N2"]["title"], "PLA, bản vẽ BZN-H-N2"),
        ("head_frame", "N3", "N3 Head Frame", "PLA, bản vẽ BZN-H-N3"),
        ("head_idler", "N3i", "N3i Head Idler Bracket", "PLA, bản vẽ BZN-H-N3"),
        ("cam_mount", "N4", parts["N4"]["title"], "PLA, bản vẽ BZN-H-N4"),
        ("head_front", "C3", parts["C3"]["title"], "PLA, bản vẽ BZN-H-C3"),
        ("head_back", "C4", parts["C4"]["title"], "PLA, bản vẽ BZN-H-C4"),
        ("visor", "C5", parts["C5"]["title"], "PETG trong, bản vẽ BZN-H-C5"),
    ] + [(k, v[0], v[1], v[2]) for k, v in bought.items()]
    solids = {}
    for key, *_ in items:
        if key in sh.parts:
            solids[key] = sh.parts[key].moved(mv)
        else:
            solids[key] = comps[key].moved(mv)
    for k, sld in sh.optics().items():
        solids[k] = sld.moved(mv)
    items += [("board_CAM", "CAM", "Camera OV5647 (trên N4)", "mua"), ("board_TOF", "TOF", "ToF VL53L0X (trên N4)",
                                                                       "mua")]
    # fasteners from the hole records of the head parts
    fast = {}

    def add(code, what, n, where):
        f = fast.setdefault(code, [what, 0, set()])
        f[1] += n
        f[2].add(where)
    for h in body.holes():
        if h.part not in ("neck_link", "head_frame", "head_idler", "cam_mount"):
            continue
        k = h.kind
        if k in ("ins_m2", "ins_m2_thru", "boss_m2"):
            add("INS-M2", "Tán cấy nhiệt M2 L3 gai Ø3.6", 1, h.part)
        if k == "boss_m2":
            add("M2x6", "Ốc M2×6 lục giác chìm", 1, "board camera/ToF")
        elif k == "ins_m3":
            add("INS-M3", "Tán cấy nhiệt M3 short L3 gai Ø4.6", 1, h.part)
        elif k == "pilot_st2":
            add("ST2x3", "Vít tự ren PA 2×3 (horn)", 1, h.part)
        elif k == "bearing":
            add("MR63ZZ", "Bạc đạn MR63ZZ 3×6×2.5", 1, h.part)
            add("W3", "Long đền thép M3 3.2×5×0.5", 1, h.part)
            add("M3x6B", "Ốc M3×6 đầu dù ISO 7380 (idler)", 1, h.part)
        elif k == "tie":
            add("TIE", "Dây rút 2.5 mm (2 khe / dây)", 0.5, h.part)
    ear = [h for h in body.holes() if h.part == "neck_link" and h.id.startswith("head_pitch.ear")]
    shell_scr = [x for x in sh.screws if x[0] in ("head_front", "head_back")]
    cam_scr = [h for h in body.holes() if h.part == "cam_mount" and h.kind == "cb_m2"]
    idl = [h for h in body.holes() if h.part == "head_idler" and h.kind == "clr_m2"]
    add("M2x6", "Ốc M2×6 lục giác chìm", len(ear) + len(shell_scr) + len(cam_scr) + len(idl),
        "tai servo, vỏ, N4, N3i")
    boss = [h for h in body.holes() if h.part == "cam_mount" and h.kind == "boss_m2"]
    # sheet 1: assembled views + parts list
    sheets = []
    allfix = [solids[k] for k in solids]
    lo = np.min([exact_bbox(x)[0] for x in allfix], axis=0)
    hi = np.max([exact_bbox(x)[1] for x in allfix], axis=0)
    size, c3 = hi - lo, (lo + hi) / 2
    K = choose_scale((size[1], size[2], size[0]), (110.0, 95.0, 95.0))
    tb = dict(owner=OWNER, project="Bazan Humanoid — cụm đầu (H)", title="H0 Cụm đầu (bản vẽ lắp)", number="BZN-H-H0",
              material="xem danh mục", mass="xem danh mục", scale=ratio(K), designed=OWNER, checked="TBD",
              approved="TBD", date=date.today().isoformat(), rev="P1", status=STATUS, standards=STANDARDS)
    s1 = Sheet(1, "assembly")
    fx, fy = 30 + 25 + K * size[1] / 2, 287 - 30 - K * size[2] / 2
    lx = fx + K * size[1] / 2 + 40 + K * size[0] / 2
    fc, lc = project(c3, "front"), project(c3, "left")
    s1.place("front", "front", fx, fy, fc[0], fc[1], K)
    s1.place("left", "left", lx, fy, lc[0], fc[1], K)
    whole = cq.Compound.makeCompound([x for x in allfix])
    for nm in ("front", "left"):
        vis, _ = hlr(whole, nm)
        s1.draw_edges(nm, vis, w=W_THICK * 0.7)
    s1.dim("front", (c3[0], lo[1], hi[2]), (c3[0], hi[1], hi[2]), (0, 1, 0), "above", on_surface=False)
    s1.dim("front", (c3[0], lo[1], lo[2]), (c3[0], lo[1], hi[2]), (0, 0, 1), "left", on_surface=False)
    s1.dim("left", (lo[0], c3[1], hi[2]), (hi[0], c3[1], hi[2]), (1, 0, 0), "above", on_surface=False)
    s1.dim("left", (0, 0, lo[2]), (0, 0, 0), (0, 0, 1), "right", on_surface=False)
    s1.centermark("left", (0, 0, 0), 3.0)
    s1.caption("front", "Cụm đầu lắp xong (nhìn từ +X)", ratio(K))
    s1.caption("left", "Nhìn từ −Y; dấu tâm = trục gật đầu O", ratio(K))
    # parts list (ISO 7573 style), bottom left
    cols = [("Mục", 10), ("Mã", 14), ("Tên", 70), ("SL", 9), ("Vật liệu / nguồn", 62)]
    xs = np.cumsum([0] + [w for _, w in cols]) + 25.0
    rh = 5.0
    y0 = min(s1.bbox["front"][1], s1.bbox["left"][1]) - 22
    s1.text(25, y0 + 5, "DANH MỤC CHI TIẾT", fs=9, role="head", weight="bold", va="bottom")
    for r, row in enumerate([[c for c, _ in cols]] + [[str(i + 1), it[1], it[2], "1", it[3]]
                                                      for i, it in enumerate(items)]):
        y = y0 - r * rh
        for c, val in enumerate(row):
            s1.ax.add_patch(plt.Rectangle((xs[c], y - rh), xs[c + 1] - xs[c], rh, fill=r == 0, facecolor="#e8e8e8",
                                          edgecolor=INK, lw=W_THIN))
            s1.text(xs[c] + 1.0, y - rh / 2, val, fs=6.4, role="table", va="center", weight="bold" if r == 0 else
                    "normal")
    sheets.append(s1)
    # sheet 2: exploded isometric view with balloons
    s2 = Sheet(2, "exploded")
    explode = {"head_front": (55, 0, 34), "head_back": (-60, 0, 30), "visor": (95, 0, 40), "cam_mount": (22, 0, 22),
               "board_CAM": (48, 0, 30), "board_TOF": (48, 0, 30), "board_DISP": (40, 0, -12),
               "head_idler": (0, -30, -12), "spacer_head_pitch": (0, -18, -2), "horn_head_pitch": (0, 18, -2),
               "head_frame": (0, 0, 10), "servo_head_pitch": (0, 0, -26), "neck_link": (0, 0, -40)}
    ex = {k: solids[k].moved(cq.Location(cq.Vector(*explode.get(k, (0, 0, 0))))) for k in solids}
    whole = cq.Compound.makeCompound(list(ex.values()))
    elo, ehi = exact_bbox(whole)
    ec = (elo + ehi) / 2
    corners = np.array([project((x, y, z), "iso") for x in (elo[0], ehi[0]) for y in (elo[1], ehi[1])
                        for z in (elo[2], ehi[2])])
    span = np.ptp(corners, axis=0)
    ke = max(k for k in ISO_SCALES if k * span[0] <= 220 and k * span[1] <= 200) if any(
        k * span[0] <= 220 and k * span[1] <= 200 for k in ISO_SCALES) else 0.5
    ic = project(ec, "iso")
    s2.place("xiso", "iso", 160, 160, ic[0], ic[1], ke)
    vis, _ = hlr(whole, "iso")
    s2.draw_edges("xiso", vis, w=W_THICK * 0.6)
    s2.caption("xiso", "Hình tách rời (trục đo)", ratio(ke) + ", minh hoạ")
    # balloons in two columns, spaced 10 mm, leaders to each part's centre
    cx0, cy0, cx1, cy1 = s2.bbox["xiso"]
    mid = (cx0 + cx1) / 2
    targets = []
    for i, (key, *_r) in enumerate(items):
        lo_, hi_ = exact_bbox(ex[key])
        targets.append((i + 1, s2.P("xiso", (lo_ + hi_) / 2)))
    for side in (-1, 1):
        col = sorted([t for t in targets if (t[1][0] < mid) == (side < 0)], key=lambda t: -t[1][1])
        ys, prev = [], None
        for _, P in col:
            y = P[1] if prev is None else min(P[1], prev - 10.0)
            ys.append(y)
            prev = y
        shift = max(0.0, 20.0 - ys[-1]) if ys else 0.0
        bx = cx0 - 14 if side < 0 else cx1 + 14
        for (num, P), y in zip(col, ys):
            y += shift
            s2.line([P[0], bx - side * 4.2], [P[1], y], w=W_THIN)
            s2.ax.plot([P[0]], [P[1]], "o", color=INK, ms=1.6)
            s2.ax.add_patch(plt.Circle((bx, y), 4.2, fill=False, lw=W_THIN, color=INK))
            s2.text(bx, y, str(num), fs=8, role="balloon", ha="center", va="center", weight="bold")
    # fastener list and the assembly order
    xt = 302
    s2.text(xt, 281, "BỘ ỐC / LINH KIỆN NHỎ (từ CAD)", fs=8.5, role="head", weight="bold", va="top")
    yy = 274
    for code, (what, n, where) in fast.items():
        s2.text(xt, yy, f"{code}  ×{int(round(n))}", fs=7.0, role="table", va="top", weight="bold")
        s2.text(xt + 22, yy, textwrap.shorten(what, 48, placeholder="…"), fs=6.6, role="table", va="top")
        yy -= 4.6
    s2.text(xt, yy - 3, "TRÌNH TỰ LẮP", fs=8.5, role="head", weight="bold", va="top")
    steps = ["1. Ép tán: N2 (tai servo, tán M3 ở mặt ngoài thành idler), N3 (A, B, G), N4 (trụ).",
             "2. SG90 vào ổ N2, 2 × M2×6 qua tai (lỗ tai khoan Ø2.3); servo ở 1500 µs.",
             "3. Horn lên N3 (2 × ST2×3); đẩy N3 dọc trục cho horn ăn then hoa; vít tâm horn qua lỗ Ø5.2.",
             "4. MR63ZZ trượt vào N3i; ống đồng 1.3 + W3 + M3×6 đầu dù vào tán M3 của N2.",
             "5. 2 × M2×6 từ dưới qua đế N3i vào tán G của N3 (siết sau cùng: tự định tâm).",
             "6. Màn hình buộc dây rút vào tấm mặt; N4 + camera + ToF lên sàn (2 × M2×6).",
             "7. Dán C5 vào C3 từ trong; C3, C4 lên sàn: 4 × M2×6 thả qua ống từ đỉnh mũ.",
             "Tháo: ngược lại; không banh khung."]
    yy -= 9
    for st in steps:
        w = textwrap.wrap(st, 52)
        s2.text(xt, yy, "\n".join(w), fs=6.6, role="note", va="top", linespacing=1.25)
        yy -= 3.1 * len(w) + 1.0
    sheets.append(s2)
    for sh_ in sheets:
        sh_.furniture(tb, len(sheets))
    # checks specific to an assembly
    n_ball = sum(1 for t, role in s2.texts if role == "balloon")
    nums = sorted(int(t.get_text()) for t, role in s2.texts if role == "balloon")
    checks = [("mọi mục trong danh mục có đúng một số chỉ dẫn (balloon)", nums == list(range(1, len(items) + 1)),
               f"{n_ball} số chỉ dẫn, {len(items)} mục"),
              ("số tán M2 = số lỗ tán M2 của N2, N3, N4 trong CAD",
               fast["INS-M2"][1] == len([h for h in body.holes() if h.part in ("neck_link", "head_frame", "cam_mount")
                                         and h.kind in ("ins_m2", "ins_m2_thru", "boss_m2")]), f"{fast['INS-M2'][1]}"),
              ("số ốc M2×6 = tai servo + vỏ + N4 + N3i + trụ board trong CAD",
               fast["M2x6"][1] == len(ear) + len(shell_scr) + len(cam_scr) + len(idl) + len(boss),
               f"{fast['M2x6'][1]}"),
              ("cụm đầu lắp xong không chồng khối (robot.collisions ở tư thế 0)",
               not [h for h in R.collisions(body, LEG.Leg(), {}) if any(k in str(h) for k in solids)], "")]
    return dict(code="H0", sheets=sheets, solid=None, tb=tb, checks=checks, items=items, fast=fast)


# ------------------------------------------------------------------ main
def write(res, out):
    code = res["code"]
    rep, bad = validate(res)
    out.mkdir(parents=True, exist_ok=True)
    num = f"BZN-H-{code}"
    with PdfPages(out / f"{num}.pdf", metadata={"CreationDate": None, "ModDate": None}) as pdf:
        for sh in res["sheets"]:
            pdf.savefig(sh.fig)
            sh.fig.savefig(out / f"{num}_sheet{sh.number}.png", dpi=110)
            plt.close(sh.fig)
    lines = [f"# {num} · kiểm tra bản vẽ tự động", "", f"Sinh bởi `hardware/mechanical/drawings_parts.py {code}`. "
             f"Lỗi: **{bad}**.", "", "| Kiểm tra | Kết quả | Chi tiết |", "| --- | --- | --- |"]
    lines += [f"| {r['check']} | {'đạt' if r['ok'] else '**LỖI**'} | {r['detail']} |" for r in rep]
    (out / f"{num}_validation.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    json.dump(rep, open(out / f"{num}_validation.json", "w"), ensure_ascii=False, indent=1)
    nd = sum(len(s.dims) for s in res["sheets"])
    print(f"{code}: {len(res['sheets'])} sheets, {nd} dimensions, {len(res.get('feats', []))} features, "
          f"validation defects {bad}")
    for r in rep:
        if not r["ok"]:
            print("  LỖI:", r["check"], "—", r["detail"])
    return bad


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    want = ["N2", "N4", "C3", "C4", "C5", "H0"] if not args or args == ["all"] else args
    parts, org = head_parts()
    bad = 0
    for code in want:
        res = draw_h0(parts, org) if code == "H0" else draw_part(code, parts[code])
        bad += write(res, D.DOCS / code)
    return 1 if (bad and "--check" in sys.argv) else 0


if __name__ == "__main__":
    sys.exit(main())
