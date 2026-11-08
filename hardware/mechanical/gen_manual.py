"""Generate the whole-robot assembly manual (Vietnamese) from leg.py + body.py:

    docs/mechanical/mico_v4_lap_rap.md          the manual (BOM, parts, holes, steps, wiring, service)
    docs/mechanical/mico_v4_lap_rap.html        the same, one self-contained printable page
    docs/mechanical/v4/drawings/<part>.png      3-view drawing per printed part, holes labelled H1…
    docs/mechanical/v4/steps/<step>.png         one picture per assembly step, new parts orange, screws red
    mechanical/out/{L,R}_<part>.{step,stl}       print files for the body parts (legs: gen_docs.py)

    python gen_manual.py            # about 3 min
"""
import base64
import html
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cadquery as cq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import body as B  # noqa: E402
import electronics as el  # noqa: E402
import hardware as hw  # noqa: E402
import leg as LEG  # noqa: E402
import pic  # noqa: E402
import robot as R  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
DOCS = REPO / "docs" / "mechanical"
V4 = DOCS / "v4"
OUT = HERE / "out"
SIDED = {"sled_pca", "sled_ina", "shoulder_mount", "shoulder_link", "upper_arm", "forearm"}
PLA, TPU = 1.24, 1.21

# ------------------------------------------------------------------ part catalogue (body)
BODY_PARTS = {  # part: (code, Vietnamese name, reference point name, print note)
    "pelvis_plate": ("P0", "Pelvis Plate (Deck 0)", "tâm robot (x=0, y=0) trên mặt TRÊN của tấm",
                     "mặt dưới úp xuống bàn; tường pin hướng lên"),
    "deck_1": ("P1", "Deck 1", "tâm robot trên mặt trên tầng", "phẳng, 2 viền, 15 % infill"),
    "deck_2": ("P2", "Deck 2", "tâm robot trên mặt trên tầng", "phẳng, 2 viền, 15 % infill"),
    "sled_pca": ("S1", "PCA9685 Sled (L/R)", "tâm bay trên mặt deck 0", "đế phẳng xuống bàn, trụ hướng lên"),
    "sled_perf": ("S2", "A+ Board Sled (Black Pill)", "tâm bay trên mặt deck 1", "đế phẳng xuống bàn"),
    "sled_radxa": ("S3", "Radxa Zero 3W Sled", "tâm bay trên mặt deck 2", "đế phẳng xuống bàn"),
    "sled_ubec": ("S4", "UBEC Sled", "tâm bay trên mặt deck 1", "đế phẳng xuống bàn"),
    "sled_buck": ("S5", "5 V Buck Sled", "tâm bay trên mặt deck 1", "đế phẳng xuống bàn"),
    "sled_ina": ("S6", "INA226 Sled (L/R)", "tâm bay trên mặt deck 1", "đế phẳng xuống bàn"),
    "shoulder_mount": ("A1", "Shoulder Mount (L/R)", "điểm trục vai trên mặt đầu servo",
                       "mặt bích xuống bàn, ổ servo mở lên trên"),
    "shoulder_link": ("A2", "Shoulder Link (L/R)", "điểm trục roll trên mặt đầu servo",
                      "nằm mặt đĩa horn xuống bàn, có support trong ổ"),
    "upper_arm": ("A3", "Upper Arm (L/R)", "điểm trục khuỷu trên mặt đầu servo", "nằm nghiêng mặt đĩa horn xuống bàn"),
    "forearm": ("A4", "Forearm and Mitten (L/R)", "điểm trục khuỷu trên mặt đầu servo", "nằm mặt đĩa horn xuống bàn"),
    "neck_mount": ("N1", "Neck Mount", "điểm trục cổ trên mặt đầu servo", "mặt bích xuống bàn"),
    "neck_link": ("N2", "Neck Link", "tâm trục gật đầu (giữa ổ)", "đĩa horn cổ xuống bàn"),
    "head_frame": ("N3", "Head Frame", "tâm trục gật đầu (giữa ổ)", "mặt sàn đầu úp xuống bàn, support dưới 2 đĩa"),
    "cam_mount": ("N4", "Camera + ToF Mount (15° Tilt)", "mặt sàn đầu, tâm bay camera", "đế xuống bàn"),
}
LEG_PARTS = {  # from leg.py / leg_v3.md
    "foot": ("L1", "Foot"), "sole": ("L2", "TPU Sole"), "ankle_block": ("L3", "Ankle Block"),
    "shin": ("L4", "Shin"), "thigh": ("L5", "Thigh"), "hip_block": ("L6", "Hip Block"),
    "yaw_bracket": ("L7", "Yaw Bracket"), "thrust_ring": ("L8", "Thrust Ring"),
    "hip_mount": ("L9", "Hip Mount"),
}
FAST_VI = {
    "M2x6": "ốc lục giác chìm M2×6 inox", "M2x10": "ốc lục giác chìm M2×10 inox",
    "M3x6": "ốc lục giác chìm M3×6 inox", "M3x6B": "ốc đầu dù M3×6 inox (trục idler)",
    "ST2x4": "vít tự ren đầu tròn 2×4 (bắt horn)", "INS-M2": "tán cấy nhiệt M2, dài 3, OD 3.2",
    "INS-M3": "tán cấy nhiệt M3, dài 3, OD 4.2", "MR63ZZ": "bạc đạn MR63ZZ 3×6×2.5",
    "W3": "long đền mỏng M3 3.2×5×0.5", "SO40": "trụ nhựa M3 đực-cái 40 mm (+6 ren)",
    "SO35": "trụ nhựa M3 đực-cái 35 mm (+6 ren)", "TIE": "dây rút 2.5×100 mm", "STRAP": "dây dán pin velcro 20 mm",
    "FOAM": "băng keo xốp 2 mặt 1 mm",
}


def ref_point(body, part):
    j = body.joints
    if part in ("pelvis_plate",):
        return np.array([0, 0, B.Z0T])
    if part in ("deck_1", "deck_2"):
        return np.array([0, 0, B.Z1T if part == "deck_1" else B.Z2T])
    if part.startswith("sled_"):
        bay = {"sled_pca": "PCA_L", "sled_perf": "PERF", "sled_radxa": "RADXA", "sled_ubec": "UBEC",
               "sled_buck": "BUCK", "sled_ina": "INA_L"}[part]
        deck, c, *_ = body.bays[bay]
        z = {"pelvis_plate": B.Z0T, "deck_1": B.Z1T, "deck_2": B.Z2T}[deck]
        return np.array([c[0], c[1], z])
    jn = {"shoulder_mount": "shoulder_pitch", "shoulder_link": "shoulder_roll", "upper_arm": "elbow",
          "forearm": "elbow", "neck_mount": "neck_yaw"}.get(part)
    if jn:
        return np.asarray(j[jn].p, float)
    if part in ("neck_link", "head_frame"):
        return np.asarray(j["head_pitch"].origin, float)
    if part == "cam_mount":
        return np.array([12.0, 0.0, B.HEAD_DECK + 3.0])
    raise KeyError(part)


def fmt(v):
    return "(" + ", ".join(f"{x:+.1f}" for x in v) + ")"


def dname(d):
    d = tuple(np.round(np.asarray(d, float), 3))
    return {(1, 0, 0): "+x", (-1, 0, 0): "−x", (0, 1, 0): "+y", (0, -1, 0): "−y", (0, 0, 1): "+z",
            (0, 0, -1): "−z"}.get(d, fmt(d))


# ------------------------------------------------------------------ drawings
def drawing(shape, holes, ref, title, path):
    s = shape.moved(cq.Location(cq.Vector(*(-ref))))
    edges = [np.array([[p.x, p.y, p.z] for p in e.positions(np.linspace(0, 1, 20))]) for e in s.Edges()]
    views = [("Front view (from +x)", 1, 2, 0, False), ("Left view (from +y)", 0, 2, 1, True),
             ("Top view (from +z)", 0, 1, 2, False)]
    bb = s.BoundingBox()
    fig, axs = plt.subplots(1, 3, figsize=(16, 6.2), dpi=120)
    for ax, (t, i, j, k, flip) in zip(axs, views):
        for pts in edges:
            uu = -pts[:, i] if flip else pts[:, i]
            ax.plot(uu, pts[:, j], color="#2A3D5E", lw=0.55)
        for n, h in enumerate(holes):
            p = np.asarray(h.pos) - ref
            u, v = (-p[i] if flip else p[i]), p[j]
            if abs(h.dir[k]) > 0.9:
                ax.add_patch(plt.Circle((u, v), h.d / 2, fill=False, color="#E8590C", lw=1.1))
            else:
                d = np.asarray(h.dir)
                du, dv = (-d[i] if flip else d[i]), d[j]
                L_ = h.depth if h.depth else 3.0
                ax.plot([u, u + du * L_], [v, v + dv * L_], color="#E8590C", lw=2.0)
            ax.annotate(f"H{n + 1}", (u, v), xytext=(3, 3), textcoords="offset points", fontsize=6.5, color="#C92A2A")
        ax.plot([0], [0], "+", color="#C92A2A", ms=10, mew=1.2)
        ax.set_title(t, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(True, color="#e3e7ee", lw=0.5)
        ax.set_xlabel(("−" if flip else "") + "xyz"[i] + " (mm)")
        ax.set_ylabel("xyz"[j] + " (mm)")
        ax.tick_params(labelsize=7)
    fig.suptitle(f"{title} · origin (+) = reference point · envelope {bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f} mm",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ------------------------------------------------------------------ BOM
def count_body(body):
    cnt = Counter()
    shell = 0
    for h in body.holes():
        k = 2 if h.part in SIDED else 1
        kind, f = h.kind, h.fastener
        if kind == "ins_m3_thru" or (kind == "ins_m3"):
            cnt["INS-M3"] += k
        elif kind in ("ins_m2", "ins_m2_thru", "boss_m2"):
            cnt["INS-M2"] += k
            if kind == "boss_m2" or "servo ear" in f:
                cnt["M2x6"] += k
            if h.id.split(".")[-1].startswith("shell"):
                shell += k
        elif kind in ("cb_m2", "clr_m2"):
            cnt["M2x6"] += k
        elif kind == "pilot_st2":
            cnt["ST2x4"] += k
        elif kind == "bearing":
            cnt["MR63ZZ"] += k
            cnt["W3"] += k
            cnt["M3x6B"] += k
        elif kind == "tie":
            cnt["TIE"] += k / 2
    cnt["SO40"] += 4
    cnt["SO35"] += 4
    cnt["M3x6"] += 4
    cnt["STRAP"] += 1
    cnt["FOAM"] += 1
    return cnt, shell


def count_leg(leg):
    cnt = Counter()
    for h in leg.holes():
        f = h.fastener
        if f.startswith("INS-M2"):
            cnt["INS-M2"] += 1
            if "M2x8" not in f:               # the hip_mount screws come from the pelvis (counted there)
                cnt["M2x6"] += 1
        elif f.startswith("INS-M3"):
            cnt["INS-M3"] += 1
        elif f.startswith("MR63ZZ"):
            cnt["MR63ZZ"] += 1
            cnt["W3"] += 1
            cnt["M3x6B"] += 1
        elif f.startswith("ST2x4"):
            cnt["ST2x4"] += 1
    return cnt                                 # sole and thrust-ring screws: counted with their inserts


# ------------------------------------------------------------------ pictures of the steps
def colour(n):
    base = n.split(".")[-1]
    if base.startswith("servo_"):
        return pic.COL["servo"]
    if base.startswith("horn_"):
        return pic.COL["horn"]
    if base.startswith("board_"):
        return pic.COL["board"]
    if base == "battery":
        return pic.COL["batt"]
    if base.startswith("so"):
        return pic.COL["so"]
    return pic.COL["old"]


def world_holes(body, leg, part, side="L", pred=lambda h: True):
    """Hole entry points of a part in the robot frame (zero pose)."""
    out = []
    if part in LEG_PARTS:
        for h in leg.holes():
            if h.part == part and pred(h):
                p = np.asarray(h.pos, float) + [0, LEG.HIP_Y, 0]
                if side == "R":
                    p[1] = -p[1]
                out.append(p)
        return out
    for h in body.holes():
        if h.part == part and pred(h):
            p = np.asarray(h.pos, float)
            if side == "R":
                p[1] = -p[1]
            out.append(p)
    return out


def step_pic(comps, shown, new, path, marks=(), az=35, el=25, labels=(), title=None, size=(8, 7), focus=None):
    items = []
    for n in shown:
        if n not in comps:
            continue
        c = pic.COL["new"] if n in new and not n.split(".")[-1].startswith(("servo_", "horn_", "board_")) \
            else colour(n)
        if n in new and n.split(".")[-1].startswith("servo_"):
            c = "#3B5BA9"
        items.append((comps[n], c, 1.0))
    fx = [comps[n] for n in (focus if focus is not None else new) if n in comps]
    pic.render(items, path, az=az, el=el, marks=marks, labels=labels, title=title, size=size, focus=fx)


# ------------------------------------------------------------------ the manual
def main():
    body, leg = B.Body(), LEG.Leg()
    (V4 / "drawings").mkdir(parents=True, exist_ok=True)
    (V4 / "steps").mkdir(parents=True, exist_ok=True)
    OUT.mkdir(exist_ok=True)
    comps = R.assemble(body, leg, {})
    names = list(comps)
    S = body.summary()
    floor = leg.floor
    top = max(p.BoundingBox().zmax for p in comps.values())

    # print files and drawings
    holes_by = defaultdict(list)
    for h in body.holes():
        holes_by[h.part].append(h)
    for part, shape in body.parts.items():
        cq.exporters.export(shape, str(OUT / f"L_{part}.step"))
        cq.exporters.export(shape, str(OUT / f"L_{part}.stl"), tolerance=0.02, angularTolerance=0.1)
        if part in SIDED:
            m = shape.mirror("XZ")
            cq.exporters.export(m, str(OUT / f"R_{part}.step"))
            cq.exporters.export(m, str(OUT / f"R_{part}.stl"), tolerance=0.02, angularTolerance=0.1)
        hs = sorted(holes_by[part], key=lambda h: h.id)
        holes_by[part] = hs
        code, vi, refn, _ = BODY_PARTS[part]
        drawing(shape, hs, ref_point(body, part), f"{code} · {part} · {vi}", V4 / "drawings" / f"{part}.png")

    # ---------------------------------------------------------------- steps
    steps = []

    def add(key, title, text, shown, new, marks=(), az=35, el=25, labels=(), bom=None, focus=None):
        steps.append(dict(key=key, title=title, text=text, img=f"v4/steps/{key}.png", bom=bom or {}))
        step_pic(comps, shown, new, V4 / "steps" / f"{key}.png", marks=marks, az=az, el=el, labels=labels,
                 focus=focus)

    legL = [n for n in names if n.startswith("L.leg.")]
    legR = [n for n in names if n.startswith("R.leg.")]
    legs = legL + legR

    def lp(*parts, side="L"):
        return [f"{side}.leg.{p}" for p in parts]

    # leg module (left shown; build two, the right one mirrored)
    order = [("foot", "sole"), ("ankle_block",), ("shin",), ("thigh",), ("hip_block",), ("yaw_bracket",),
             ("thrust_ring", "hip_mount")]
    jn_of = {("ankle_block",): ["ankle_roll", "ankle_pitch"], ("thigh",): ["knee"], ("hip_block",):
             ["hip_pitch", "hip_roll"], ("thrust_ring", "hip_mount"): ["hip_yaw"]}
    shown = []
    leg_text = {
        "foot": "Bắt đế TPU (L2) dưới bàn chân (L1): 4 × M2×6 từ dưới lên, đầu ốc chìm 2 mm trong đế, vặn vào 4 tán M2 "
                "đã cấy ở 4 góc bàn chân.",
        "ankle_block": "Khối cổ chân (L3) mang 2 servo MG92B: ankle roll (trục x) và ankle pitch (trục y). Mỗi servo: "
                       "luồn dây qua rãnh cuối ổ, đẩy servo vào từ phía trục, 2 × M2×6 qua tai servo vào tán M2. "
                       "Lắp bàn chân vào ankle roll: đĩa horn ép vào trục khi servo ở 1500 µs, bàn chân song song mặt "
                       "khối; vít giữa horn qua lỗ Ø5.2; phía đối diện: long đền W3 lên bạc đạn rồi ốc đầu dù M3×6 "
                       "vào tán M3 — vặn vừa khít, bạc đạn phải còn quay.",
        "shin": "Ống chân (L4) lên trục ankle pitch (ống chân thẳng đứng): horn + vít giữa, idler M3×6 + W3.",
        "thigh": "Servo knee vào ổ trong đùi (L5); lắp đầu trên ống chân vào trục knee (gối thẳng): horn + vít "
                 "giữa, idler M3×6 + W3.",
        "hip_block": "Khối hông (L6) mang hip roll và hip pitch. Đùi lên trục hip pitch (chân thẳng).",
        "yaw_bracket": "Giá yaw (L7) lên trục hip roll (khối hông nằm ngang): horn + vít giữa, idler M3×6 + W3.",
        "thrust_ring": "Servo hip yaw MG90S vào đế gắn chân (L9) từ dưới, 2 × M2×6 qua tai. Ép mặt trên giá yaw vào "
                       "trục (chân hướng thẳng ra trước), vít giữa horn từ dưới. Lắp vòng chặn tải (L8) từ dưới: "
                       "2 × M2×6 vào đế gắn chân. Giá yaw phải xoay nhẹ, khe 0.2 mm với vòng; 1 giọt mỡ lithium.",
    }
    for i, grp in enumerate(order):
        new = lp(*grp)
        for jn in jn_of.get(grp, []):
            new += lp(f"servo_{jn}", f"horn_{jn}")
        if grp == ("ankle_block",):
            new += lp("servo_ankle_roll", "horn_ankle_roll")
        if grp == ("shin",):
            new += lp("servo_ankle_pitch", "horn_ankle_pitch")
        if grp == ("hip_block",):
            new += lp("servo_hip_roll", "horn_hip_roll")
        if grp == ("yaw_bracket",):
            new += lp("servo_hip_roll", "horn_hip_roll")
        new = list(dict.fromkeys(new))
        shown = list(dict.fromkeys(shown + new))
        mk = []
        for g in grp:
            for p in world_holes(body, leg, g, pred=lambda h: h.kind in ("ins_m2", "ins_m3", "bearing", "clr_m2")):
                mk.append((p, ""))
        add(f"leg_{i + 1:02d}", f"Leg {i + 1}: {' + '.join(LEG_PARTS[g][1] for g in grp)}",
            leg_text[grp[0]], shown, new, marks=mk, az=40, el=20)

    # torso
    base = []
    pel = ["pelvis_plate"]
    mk = [(p, "") for p in world_holes(body, leg, "pelvis_plate",
                                       pred=lambda h: h.kind in ("ins_m3_thru", "ins_m2", "ins_m2_thru"))]
    add("torso_01", "Torso 1: Heat-Set Inserts in Pelvis Plate P0",
        "4 tán M3 vào 4 lỗ góc Ø4.0 (ép từ mặt TRÊN, bằng mặt); "
        "4 tán M2 vào 2 bay PCA (lỗ xuyên, ép từ mặt trên, bằng mặt); "
        "6 tán M2 cho vành dưới vỏ thân (ép từ mặt DƯỚI, lỗ sâu 4). Mỏ hàn 220 °C, ấn thẳng, để nguội 1 phút.",
        pel, pel, marks=mk, az=35, el=55)
    base = pel + legs
    mk = [(p, "M2×6") if i == 0 else (p, "") for i, p in
          enumerate(world_holes(body, leg, "pelvis_plate", pred=lambda h: h.kind == "cb_m2"))]
    add("torso_02", "Torso 2: Mount Both Leg Modules to P0",
        "Đặt module chân trái dưới P0 (dây 5 servo luồn lên qua khe trái phía sau, khe 5 × 14 mm), 4 lỗ trên P0 trùng "
        "4 tán của đế gắn chân. 4 × M2×6 từ mặt trên (đầu ốc chìm trong lỗ khoét Ø4.4 sâu 2.2). Lặp lại bên phải. "
        "Chú ý: dùng M2×6, không dùng M2×8 như leg_v3 cũ — ốc dài hơn sẽ chạm đáy lỗ tán.",
        base, legs, marks=mk, az=-35, el=35, focus=["pelvis_plate", "L.leg.hip_mount", "R.leg.hip_mount"])
    so40 = [f"so40_{i + 1}" for i in range(4)]
    base += so40
    add("torso_03", "Torso 3: Four SO40 Standoffs",
        "Vặn 4 trụ nhựa M3 40 mm (đầu đực xuống) vào 4 tán M3 góc P0, tay vặn vừa chặt. Ren đực dài 6 nên lòi 1.5 mm "
        "dưới P0 — bình thường.", base, so40, az=35, el=25, focus=["pelvis_plate"] + so40)
    pca = ["L.sled_pca", "R.sled_pca", "board_PCA_PCA_L", "board_PCA_PCA_R"]
    base += pca
    mk = [(p, "") for s in ("L", "R") for p in world_holes(body, leg, "sled_pca", s, lambda h: h.kind == "cb_m2")]
    add("torso_04", "Torso 4: Two PCA9685 Boards on Sleds S1",
        "Cấy 4 tán M2 vào 4 trụ của mỗi đế S1 (bằng mặt đỉnh trụ). Bắt 2 ốc M2×6 giữ đế vào bay (lỗ khoét, đầu ốc "
        "chìm) TRƯỚC, sau đó đặt PCA9685 lên 4 trụ, 4 × M2×6. Hàng header servo quay ra mép ngoài (|y| lớn). "
        "PCA trái = #1, PCA phải = #2 (địa chỉ I²C 0x40 / 0x41 — hàn jumper A0 trên tấm phải).",
        base, pca, marks=mk, az=35, el=45, focus=["pelvis_plate"])
    base += ["battery"]
    add("torso_05", "Torso 5: 2S Battery",
        "Luồn dây velcro 20 mm qua 2 khe 21 × 3 mm hai bên hốc pin. Đặt pin (dây ra phía sau), kéo velcro chặt. "
        "Pin nằm giữa 2 tường đầu (khe 1 mm mỗi đầu) và 4 cột bên.", base, ["battery"], az=35, el=45,
        focus=["pelvis_plate"])
    d1 = ["deck_1", "sled_perf", "sled_ubec", "sled_buck", "L.sled_ina", "R.sled_ina", "board_PERF_PERF",
          "board_UBEC_UBEC", "board_BUCK_BUCK", "board_INA_INA_L", "board_INA_INA_R"]
    mk = [(p, "") for p in world_holes(body, leg, "deck_1", pred=lambda h: h.kind == "ins_m2_thru")]
    add("torso_06", "Torso 6: Deck 1 on the Bench",
        "Cấy 10 tán M2 xuyên tầng 1 (ép từ mặt trên, bằng mặt). Bắt các đế: S2 (mạch A+), S4 (UBEC), S5 (hạ áp), "
        "S6 ×2 (INA226) — mỗi đế 2 × M2×6. UBEC, mạch hạ áp, INA226: dán băng keo xốp 1 mm rồi siết 2 dây rút "
        "qua 4 khe. Mạch A+: 4 × M2×6 vào 4 trụ (khoan 4 lỗ Ø2.2 trên phíp 7×5 cách mép 3.5 mm; mặt dưới phíp để "
        "trống 2 vòng Ø6 tại 2 ốc giữ đế).", d1[:6], d1[:6], marks=mk, az=35, el=50)
    so35 = [f"so35_{i + 1}" for i in range(4)]
    base += d1 + so35
    add("torso_07", "Torso 7: Deck 1 onto Standoffs, Four SO35",
        "Nối dây nguồn pin → cầu chì → công tắc → UBEC trước khi đặt tầng. Đặt tầng 1 lên 4 trụ SO40 (lỗ Ø3.3). "
        "Vặn 4 trụ SO35: ren đực xuyên qua tầng, vặn vào đầu cái SO40 — tầng bị kẹp giữa 2 trụ.",
        base, ["deck_1"] + so35, az=35, el=25, focus=["pelvis_plate", "deck_1"] + so35)
    d2 = ["deck_2", "sled_radxa", "board_RADXA_RADXA"]
    base += d2
    mk = [(p, "M3×6") if i == 0 else (p, "") for i, p in
          enumerate(world_holes(body, leg, "deck_2", pred=lambda h: h.kind == "clr_so"))]
    add("torso_08", "Torso 8: Deck 2 and Radxa",
        "Cấy 20 tán M2 xuyên tầng 2 (2 bay Radxa, 8 giá vai, 4 giá cổ, 6 vành vỏ trên). Đế S3 vào bay (2 × M2×6), "
        "Radxa lên 4 trụ (4 × M2×6), cổng camera hướng ra sau-giữa. Đặt tầng 2 lên SO35, 4 × M3×6 từ trên.",
        base, d2, marks=mk, az=35, el=40, focus=["pelvis_plate", "deck_2"])
    # neck and head
    nk = ["neck_mount", "servo_neck_yaw"]
    base += nk
    mk = [(p, "") for p in world_holes(body, leg, "neck_mount", pred=lambda h: h.kind == "clr_m2")]
    add("head_01", "Head 1: Neck Mount N1 and Neck Yaw Servo",
        "Servo cắm từ trên xuống (trục hướng lên), dây ra phía sau qua rãnh, 2 × M2×6 qua tai. Đặt N1 lên tầng 2, "
        "4 × M2×6 qua mặt bích vào 4 tán.", base, nk, marks=mk, az=35, el=40)
    nl = ["neck_link", "horn_neck_yaw", "servo_head_pitch"]
    base += nl
    add("head_02", "Head 2: Neck Link N2 and Head Pitch Servo",
        "Bắt horn tròn vào đĩa dưới của N2 bằng 2 vít ST2×4 (từ phía servo). Cắm servo head pitch vào ổ của N2 "
        "(trục hướng +y, sang trái robot), 2 × M2×6. Servo cổ ở 1500 µs, đầu nhìn thẳng: ép N2 xuống trục; vít "
        "giữa horn cổ đi qua lỗ Ø5.2 xuyên 2 vách ổ — vì vậy lắp vít này TRƯỚC khi cắm servo head pitch, hoặc "
        "tháo servo head pitch ra khi cần vặn lại.", base, nl, az=35, el=30)
    hf = ["head_frame", "horn_head_pitch", "cam_mount", "board_DISP"]
    base += hf
    add("head_03", "Head 3: Head Frame N3, Camera Mount N4, Display",
        "Bạc đạn MR63ZZ ép vào đĩa idler (phía −y) của N3. Horn vào đĩa phía +y (2 × ST2×4). Ép N3 lên trục head "
        "pitch (sàn đầu nằm ngang), vít giữa horn; idler: W3 + M3×6 đầu dù vào tán M3 của N2. N4 lên sàn đầu, "
        "2 × M2×6. Camera Pi V2: 4 × M2×6 vào 4 trụ (cấy tán từ phía trước). ToF: 2 × M2×6. Màn 1.9″: băng keo "
        "xốp lên mặt trước khung + 2 dây rút qua 4 khe.", base, hf, az=35, el=20,
        focus=["neck_mount", "neck_link"] + hf)
    # arms (bench sub-assembly, then onto the shoulder)
    for side in ("L", "R"):
        sm = [f"{side}.shoulder_mount", f"{side}.servo_shoulder_pitch"]
        arm = [f"{side}.{n}" for n in ("shoulder_link", "horn_shoulder_pitch", "servo_shoulder_roll", "upper_arm",
                                       "horn_shoulder_roll", "servo_elbow", "forearm", "horn_elbow")]
        if side == "L":
            mk = [(p, "") for p in world_holes(body, leg, "shoulder_mount", "L", lambda h: h.kind == "clr_m2")]
            add("arm_01", "Arm 1: Shoulder Mount A1 and Shoulder Pitch Servo",
                "Servo nằm ngang, trục hướng ra ngoài (+y bên trái, −y bên phải), dây ra phía sau. 2 × M2×6 qua tai. "
                "A1 lên tầng 2, 4 × M2×6 qua mặt bích.", base + sm + ["R.shoulder_mount", "R.servo_shoulder_pitch"],
                sm + ["R.shoulder_mount", "R.servo_shoulder_pitch"], marks=mk, az=35, el=35, focus=["deck_2"])
            add("arm_02", "Arm 2: Arm Sub-Assembly on the Bench",
                "Trên bàn, với cả 3 servo ở 1500 µs: (1) servo roll SG90 vào ổ của khâu vai A2 (trục hướng ra trước), "
                "2 × M2×6; (2) servo khuỷu SG90 vào ổ của cánh tay trên A3 (trục hướng vào trong thân), 2 × M2×6; "
                "(3) mỗi đĩa horn: horn bắt 2 × ST2×4; (4) cẳng tay A4 lên trục khuỷu (tay thẳng xuống), vít giữa; "
                "(5) A3 lên trục roll (tay thẳng xuống), vít giữa từ phía trước.",
                [n for n in arm if not n.endswith("horn_shoulder_pitch")],
                [n for n in arm if not n.endswith("horn_shoulder_pitch")], az=55, el=15)
        base += sm + arm
    add("arm_03", "Arm 3: Arm onto the Shoulder",
        "Bắt horn shoulder pitch vào đĩa trong của A2 (2 × ST2×4). Shoulder pitch ở 1500 µs, tay thẳng xuống: ép "
        "cụm tay vào trục. Vít giữa horn đi qua lỗ Ø6 xuyên khâu vai (tua vít PH0 dài ≥ 40 mm). Lặp lại bên phải.",
        base, [n for n in names if n[:2] in ("L.", "R.") and ".leg." not in n and "sled" not in n and
               "shoulder_mount" not in n and "servo_shoulder_pitch" not in n and "ina" not in n.lower()
               and "pca" not in n.lower()], az=35, el=20)

    # overview pictures
    allc = [(s, colour(n), 1.0) for n, s in comps.items()]
    pic.render(allc, V4 / "steps" / "overview_front.png", az=20, el=12, size=(8, 9),
               labels=[((0, 0, B.Z0T), "P0 tấm hông + pin + 2 PCA"), ((10, 50, B.Z1T + 10), "P1: A+ / UBEC / INA"),
                       ((24, 0, B.Z2T + 10), "P2: Radxa"), ((0, 95, B.SH_Z - 20), "tay 3 DoF"),
                       ((-3, 0, B.HEAD_Z), "cổ 2 DoF"), ((0, 28, -60), "chân v3, 6 DoF")])
    pic.render(allc, V4 / "steps" / "overview_back.png", az=200, el=15, size=(8, 9))
    torso = [(s, colour(n), 1.0) for n, s in comps.items() if not (".leg." in n) and not n.startswith(("L.", "R."))
             or "sled" in n or "board" in n]
    pic.render(torso, V4 / "steps" / "overview_torso.png", az=35, el=30, size=(8, 7))

    # ---------------------------------------------------------------- counts
    cb, shell = count_body(body)
    cl = count_leg(leg)
    tot = Counter()
    for k, v in cb.items():
        tot[k] += v
    for k, v in cl.items():
        tot[k] += 2 * v
    shell_screws = shell
    write(body, leg, steps, holes_by, tot, cb, cl, shell_screws, S, floor, top)
    print("ok", len(steps), "steps")


# ------------------------------------------------------------------ writing
def write(body, leg, steps, holes_by, tot, cb, cl, shell_screws, S, floor, top):
    md = []
    a = md.append
    H = -floor
    a("# Mico v4 · Whole-Robot Assembly Manual")
    a("")
    a("Sinh tự động bởi `mechanical/gen_manual.py` từ `leg.py` + `body.py` — không sửa tay. Đơn vị mm. Hệ trục robot: "
      "gốc ở giữa hai điểm hông H, x ra trước, y sang TRÁI robot, z lên trên, robot đứng thẳng. Bên phải là ảnh gương "
      "của bên trái (file `R_*`).")
    a("")
    a("![tổng thể](v4/steps/overview_front.png)")
    a("")
    a("## 1. Key Dimensions")
    a("")
    a("| Đại lượng | Giá trị |")
    a("| --- | --- |")
    rows = [("Chiều cao (sàn → đỉnh giá camera, chưa vỏ)", f"{top + H:.1f}"),
            ("Điểm hông H trên sàn", f"{H:.2f}"),
            ("Mặt dưới tấm hông P0 (= đỉnh đế gắn chân) trên H", f"{B.Z0:.2f}"),
            ("Mặt trên P0 / tầng 1 / tầng 2 trên H", f"{B.Z0T:.2f} / {B.Z1T:.2f} / {B.Z2T:.2f}"),
            ("Tầng (cả 3 tầng) x × y",
             f"{B.DX[1] - B.DX[0]:.0f} × {B.DY[1] - B.DY[0]:.0f} (x {B.DX[0]:+.0f}…{B.DX[1]:+.0f})"),
            ("Trụ đứng: lưới 4 trụ", "x −44 / +35, y ±49"),
            ("Trục vai (pitch) trên H", f"{B.SH_Z:.2f} (y ±{B.SH_CT_Y:.1f} = mặt đầu servo)"),
            ("Trục roll vai / khuỷu trên H", f"{B.ROLL_Z:.2f} / {B.ELB_Z:.2f}"),
            ("Trục cổ (yaw) x / trục gật đầu trên H", f"{B.NECK_X:+.1f} / {B.HEAD_Z:.2f}"),
            ("Bề rộng qua 2 khuỷu (ngoài ổ servo)", f"{2 * (B.ELB_CT_Y + 26.5):.1f}"),
            ("Chân: L1, L2, khoảng cách 2 chân", f"{LEG.L1:.0f}, {LEG.L2:.0f}, y ±{LEG.HIP_Y:.0f}")]
    for k, v in rows:
        a(f"| {k} | {v} |")
    a("")
    a("So với bản 303 mm cũ: robot cao hơn khoảng 100 mm vì (1) chân v3 đặt mọi trục servo đúng trục khớp và tách "
      "trục roll khỏi trục pitch 20 mm, (2) thân thành 3 tầng để mỗi mạch có chỗ riêng và tháo được độc lập. "
      "Phải chạy lại mô hình URDF/khối lượng/mô-men trước khi mua servo (xem mục 11).")
    a("")
    a("## 2. Design Rules (Easy Motor and Board Swaps)")
    a("")
    a("1. **Một khớp = một hộp servo chuẩn.** Cả 20 khớp dùng cùng ổ servo 9 g (MG92B / MG90S / SG90 cùng khung vỏ), "
      "cùng đĩa horn, cùng idler MR63ZZ ở khớp chịu tải. Servo nào cũng lắp được vào khớp nào; thay 1 servo chỉ mở "
      "đúng khớp đó.")
    a("2. **Mạch không bao giờ bắt thẳng vào tầng.** Mỗi mạch nằm trên đế in riêng (4 trụ có tán M2 đúng lỗ của mạch); "
      "đế bắt vào tầng bằng 2 × M2×6 vào một *bay*: 2 tán cách nhau 40 mm (SL-40) hoặc 20 mm (SL-20). Đổi mạch "
      "khác = in đế mới 15 phút, không in lại tầng.")
    a("3. **Thân là chồng tầng**: P0 + tầng 1 + tầng 2 trên 4 trụ nhựa M3 mỗi tầng. Tháo từng tầng, không keo.")
    a("4. **Dây vào thân ở chỗ cố định**: dây chân qua 2 khe sau P0; dây tay + cổ + đầu qua khe sau tầng 2 và tầng 1.")
    a("")
    a("![thân](v4/steps/overview_torso.png)")
    a("")
    a("## 3. Fits and Tolerances (All Parts)")
    a("")
    a("In FDM, đầu 0.4, PLA. Kích thước lỗ trong file in đã cộng sẵn dung sai dưới đây — **không** chỉnh "
      "\"hole compensation\" trong slicer, trừ khi phiếu thử (bước 0.3) cho thấy lệch.")
    a("")
    a("| Mối ghép | Danh định | Lỗ in | Kiểu lắp | Kiểm tra sau in |")
    a("| --- | --- | --- | --- | --- |")
    tol = [("Vỏ servo trong ổ", "12.2 × 22.8", f"+{hw.CLEAR_SERVO} mỗi bên", "lỏng (trượt tay)",
            "servo trượt vào bằng tay, không lắc quá 0.3 mm"),
           ("Tán cấy M2", "OD 3.2", "Ø3.2", "ép nhiệt 220 °C", "tán bằng mặt ±0.2, không nghiêng"),
           ("Tán cấy M3", "OD 4.2", "Ø4.0", "ép nhiệt 220 °C", "tán bằng mặt ±0.2"),
           ("Lỗ xuyên ốc M2", "2.0", "Ø2.3", "lỏng 0.3", "ốc M2 lọt qua bằng tay"),
           ("Lỗ xuyên ren trụ M3", "3.0", "Ø3.3", "lỏng 0.3", "ren đực trụ lọt qua"),
           ("Khoét đầu ốc M2", "đầu Ø3.8 × 2.0", "Ø4.4 sâu 2.2", "đầu chìm 0.2", "đầu ốc không lồi"),
           ("Ổ bạc đạn MR63ZZ", "OD 6.00", "Ø6.05 sâu 2.5", "ép (in ra ~5.95)", "ép bằng ngón cái/kẹp, quay êm"),
           ("Lỗ mồi vít ST2", "2.2 ren", "Ø1.6", "tự ren", "vít ăn chặt, không nứt"),
           ("Lỗ vào vít horn", "đầu vít ~Ø4", "Ø5.2", "lỏng", "tua vít PH0 lọt"),
           ("Đĩa horn trong hốc", "Ø20.0", "Ø20.4", "lỏng 0.2", "horn nằm phẳng đáy hốc"),
           ("Khe giữa 2 chi tiết chuyển động", "—", f"≥ {hw.GAP_MOVING}", "khe hở", "quay hết hành trình không cọ"),
           ("Đĩa horn ↔ tháp servo", "—", "0.6", "khe hở", ""),
           ("Giá yaw ↔ vòng chặn tải", "—", "0.2", "khe hở + mỡ", "xoay nhẹ, không rơ dọc"),
           ("Trụ nhựa M3 lục giác", "5.5 AF", "—", "—", "vặn tay vừa chặt (≤ 0.4 N·m)")]
    for r_ in tol:
        a("| " + " | ".join(r_) + " |")
    a("")
    a("Độ sâu ốc (đã kiểm cho từng loại): ốc M2×6 qua tai servo dày 2.5 → ăn 3.5 vào lỗ tán sâu 4.0; qua đế board/"
      "mặt bích dày 3 → ăn 3.0 (tán dài 3); qua đế sled dày 4 khoét 2.2 → ăn 4.2 (tầng 1, 2 dày 3: mũi ốc lòi 1.2 mm "
      "dưới tầng, không chạm gì); qua PCB 1.6 vào trụ board → ăn 4.4 vào lỗ sâu 5.0; qua lỗ khoét P0 vào đế gắn "
      "chân → ăn 3.7 vào lỗ sâu 4.0. M3×6 qua tầng 2 dày 3 → ăn 3.0 vào trụ.")
    a("")
    a("## 4. Bill of Materials (Whole Robot)")
    a("")
    a("### 4.1 Servos")
    a("")
    a("| Servo | SL | Khớp |")
    a("| --- | --- | --- |")
    a("| MG92B | 10 | hip roll, hip pitch, knee, ankle pitch, ankle roll × 2 chân |")
    a("| MG90S | 4 | hip yaw × 2, shoulder pitch × 2 |")
    a("| SG90 | 6 | shoulder roll × 2, elbow × 2, neck yaw, head pitch |")
    a("")
    a("SG90 hẹp hơn MG92B 0.4 mm: nếu servo lắc trong ổ, dán 1 lớp băng keo giấy 0.1 mm lên 2 mặt bên vỏ servo. "
      "Lỗ tai SG90 cách nhau 27.6 (MG92B 27.8): nếu ốc M2 không vào, khoét lỗ tai Ø2.4.")
    a("")
    a("### 4.2 Screws, Inserts, Bearings, Standoffs (Both Legs Included)")
    a("")
    a("| Mã | Tổng | Trong 2 chân | Trong thân + tay + đầu | Hỏi cửa hàng |")
    a("| --- | ---: | ---: | ---: | --- |")
    for code in ("M2x6", "M3x6", "M3x6B", "ST2x4", "INS-M2", "INS-M3", "MR63ZZ", "W3", "SO40", "SO35", "TIE", "STRAP",
                 "FOAM"):
        t, l_, b_ = tot[code], 2 * cl.get(code, 0), cb.get(code, 0)
        a(f"| {code} | {t:g} | {l_:g} | {b_:g} | {FAST_VI[code]} |")
    a("")
    a(f"Cộng thêm cho vỏ (Design C, bắt sau): {shell_screws} × M2×6 vào các tán vỏ đã cấy (6 dưới P0, 6 trên tầng "
      "2, 4 trên sàn đầu). Mua dư 20 % mỗi loại ốc nhỏ — rơi mất là chuyện thường.")
    a("")
    a("### 4.3 Boards and Modules (MEASURE Before Printing Sleds)")
    a("")
    a("| Mã | Module | Kích thước | Lỗ bắt | Cao trên PCB | Đế | Bay |")
    a("| --- | --- | --- | --- | --- | --- | --- |")
    bay_of = {"PCA": "P0: PCA_L, PCA_R", "PERF": "P1: PERF", "UBEC": "P1: UBEC", "BUCK": "P1: BUCK",
              "INA": "P1: INA_L, INA_R", "RADXA": "P2: RADXA", "CAM": "N4 (4 trụ)", "TOF": "N4 (2 trụ)",
              "DISP": "N3 mặt trước (dây rút)", "BATT": "P0 hốc pin (velcro)"}
    sled_of = {"PCA": "S1 ×2", "PERF": "S2", "RADXA": "S3", "UBEC": "S4", "BUCK": "S5", "INA": "S6 ×2"}
    for k, bd in el.BOARDS.items():
        hl = f"{len(bd.holes)} × Ø{bd.hole_d}, " + (
            f"{2 * abs(bd.holes[0][0]):.2f} × {2 * abs(bd.holes[0][1]):.2f}" if len(bd.holes) == 4 else
            f"cách {2 * abs(bd.holes[0][0]):.1f}") if bd.holes else "không — dây rút + băng keo xốp"
        a(f"| {k} | {bd.name} | {bd.size[0]} × {bd.size[1]} | {hl} | {bd.top} | {sled_of.get(k, '—')} | "
          f"{bay_of.get(k, '—')} |")
    a("")
    a("### 4.4 Printed Parts")
    a("")
    a("| Mã | File | Tên | SL | Vật liệu | Khối lượng đặc (g) | Hướng in | Bản vẽ |")
    a("| --- | --- | --- | ---: | --- | ---: | --- | --- |")
    for part, (code, vi, _, note) in BODY_PARTS.items():
        n = 2 if part in SIDED else 1
        g = body.parts[part].Volume() / 1000 * PLA
        f = f"`L_{part}` + `R_{part}`" if part in SIDED else f"`L_{part}`"
        a(f"| {code} | {f} | {vi} | {n} | PLA | {g:.1f} | {note} | [bản vẽ](v4/drawings/{part}.png) |")
    for part, (code, vi) in LEG_PARTS.items():
        g = leg.parts[part].Volume() / 1000 * (TPU if part == "sole" else PLA)
        a(f"| {code} | `L_{part}` + `R_{part}` | {vi} | 2 | {'TPU 95A' if part == 'sole' else 'PLA'} | {g:.1f} | "
          f"xem leg_v3.md | [bản vẽ](drawings/{part}.png) |")
    a("")
    a("Cài đặt in đề xuất: lớp 0.2, 4 viền cho chi tiết khớp (A2–A4, N2–N3, L1–L9), 2 viền + 15 % gyroid cho 3 "
      "tầng và đế mạch, 40 % infill cho chi tiết khớp. Khối lượng in thực tế ≈ 60–70 % cột \"đặc\".")
    a("")
    a("## 5. Preparation (Step 0)")
    a("")
    a("1. **Đo** 1 servo mỗi loại và 1 horn bằng thước kẹp: dài, rộng, cao vỏ; dày và độ cao tai; "
      "khoảng cách 2 lỗ tai; "
      "khoảng cách trục đến đầu vỏ; Ø và dày đĩa horn; độ cao đĩa horn trên mặt vỏ. Đo từng module mục 4.3. Sửa "
      "`hardware.py` / `electronics.py`, chạy `python mechanical/robot.py` (phải báo 0 va chạm, mọi ốc vặn được) rồi "
      "`python mechanical/gen_manual.py`.")
    a("2. **In thử** `hip_block` (2 ổ servo) và đế S1: servo trượt vào bằng tay; PCA9685 khớp 4 trụ.")
    a("3. **Phiếu thử lỗ**: in tấm 30 × 15 × 3 có lỗ Ø2.3, Ø3.2, Ø3.3, Ø4.0, Ø6.05. Ø2.3 phải cho M2 lọt; Ø6.05 phải "
      "ép được MR63ZZ. Lệch > 0.1 mm → chỉnh XY hole compensation trong slicer, KHÔNG sửa file.")
    a("4. **Căn giữa servo**: cấp nguồn từng servo, đưa về 1500 µs (`mico_link.py … calibrate` hoặc bộ test servo). "
      "Chỉ ép horn khi servo đang ở 1500 µs và khâu ở tư thế 0 (thẳng) như mô tả ở từng bước. Dán nhãn tên khớp lên "
      "dây từng servo (bảng mục 9).")
    a("5. **Dụng cụ**: lục giác 1.5 / 2.0 / 2.5 mm, tua vít PH0 (dài ≥ 40 mm), mỏ hàn đầu cấy tán 220 °C, kìm, "
      "thước kẹp, mũi khoan tay 2.2 / 3.0 / 6.0 mm để làm sạch lỗ.")
    a("")
    a("## 6. Assembly")
    a("")
    a("Màu trong hình: **cam** = chi tiết mới của bước, xám = đã lắp, xanh navy = servo, xanh lá = mạch, chấm đỏ = vị "
      "trí ốc/tán của bước.")
    a("")
    for i, s in enumerate(steps):
        a(f"### Step {i + 1}. {s['title']}")
        a("")
        a(f"![{s['key']}]({s['img']})")
        a("")
        a(s["text"])
        a("")
    a("## 7. Hole Tables: Torso, Arms, Head")
    a("")
    a("Toạ độ = điểm vào của lỗ, so với điểm tham chiếu của chi tiết (dấu + đỏ trên bản vẽ), trục như hệ robot. "
      "Hướng = hướng khoan / hướng ốc đi vào. Sâu 0 = xuyên. Chi tiết chân: bảng lỗ trong `leg_v3.md`.")
    a("")
    for part, hs in holes_by.items():
        code, vi, refn, _ = BODY_PARTS[part]
        ref = ref_point(body, part)
        a(f"#### {code} · `{part}` — {vi}")
        a("")
        a(f"Điểm tham chiếu: {refn} = robot {fmt(ref)}.")
        a("")
        a(f"![{part}](v4/drawings/{part}.png)")
        a("")
        a("| # | Lỗ | Loại | Ø | Sâu | Vị trí (x, y, z) | Hướng | Dùng cho |")
        a("| --- | --- | --- | ---: | --- | --- | --- | --- |")
        for i, h in enumerate(hs):
            p = np.asarray(h.pos) - ref
            dep = f"{h.depth:.1f}" if h.depth else "xuyên"
            a(f"| H{i + 1} | `{h.id}` | {h.kind} | {h.d:.2f} | {dep} | {fmt(p)} | {dname(h.dir)} | {h.fastener} |")
        a("")
    a("## 8. Cable Routing")
    a("")
    a("| Nhóm | Ra khỏi khớp | Vào thân | Đến | Vòng chùng |")
    a("| --- | --- | --- | --- | --- |")
    a("| Chân trái (6) | rãnh cuối vỏ servo, dọc mặt sau chân, buộc qua khe dây rút ở đùi, ống chân, giá yaw | khe "
      "5 × 14 phía sau P0 bên trái (x −41…−36, y +22…+36) | PCA trái (#1) | 30 mm mỗi khớp đi qua |")
    a("| Chân phải (6) | như trên, đối xứng | khe P0 bên phải | PCA trái (#1), đi sau tường pin | 30 mm mỗi khớp |")
    a("| Tay (3 × 2) | rãnh cuối vỏ, mặt sau cánh tay, vòng 40 mm quanh trục vai | khe sau tầng 2 (x −50…−45, "
      "y ±10) rồi khe sau tầng 1 | PCA phải (#2) | 40 mm ở vai |")
    a("| Cổ + đầu (2) | ra sau giá cổ | khe sau tầng 2, tầng 1 | PCA phải (#2) | 60 mm ở cổ (cổ xoay ±90°) |")
    a("| Camera (FFC) | sau giá N4 | thẳng xuống Radxa (tầng 2) | Radxa CSI | 40 mm |")
    a("")
    a("Thứ tự kênh PCA giữ như `hardware_architecture.md` (PCA #1 = 12 servo chân, PCA #2 = tay và cổ). Mỗi servo có "
      "dây nối dài riêng, cắm/rút ở gần PCA — khi thay servo không phải đụng vào PCA.")
    a("")
    a("## 9. Replacing One Servo")
    a("")
    a("| Servo | Tháo trước | Rồi |")
    a("| --- | --- | --- |")
    a("| Chân (6 khớp) | theo bảng \"Replacing one servo\" trong leg_v3.md | — |")
    a("| Shoulder pitch | cụm tay (vít giữa horn qua lỗ Ø6 ở khâu vai) | 2 ốc tai, rút servo ra ngoài theo trục |")
    a("| Shoulder roll | cánh tay trên A3 (vít giữa horn phía trước) | 2 ốc tai trong A2, rút servo ra trước |")
    a("| Elbow | đưa tay ngang (roll 90°), tháo cẳng tay A4 | 2 ốc tai trong A3, rút servo vào phía thân |")
    a("| Head pitch | khung đầu N3 (vít horn + ốc idler M3) | 2 ốc tai trong N2, rút servo sang trái |")
    a("| Neck yaw | N3, servo head pitch, rồi vít horn cổ qua lỗ Ø5.2 xuyên N2, nhấc N2 | 2 ốc tai, rút servo lên |")
    a("")
    a("Mỗi servo: rút dây ở đầu nối gần PCA, thay servo đã căn 1500 µs, lắp ngược lại.")
    a("")
    a("## 10. CAD Checks Run")
    a("")
    a("- `robot.py`: toàn robot ở tư thế 0 không có 2 chi tiết nào chồng nhau (dung sai 0.05 mm³); quét shoulder pitch "
      "−180…60°, shoulder roll 0…160°, elbow −130…0°, neck yaw ±90°, head pitch −30…32° và các tư thế kết hợp: 0 va "
      "chạm; mọi ốc của tay/cổ/đầu vặn được bằng dụng cụ thẳng (elbow: ở tư thế roll 90°); mọi servo tay/cổ/đầu rút "
      "ra được theo trục khi tháo khâu phía sau nó.")
    a("- `check.py`: chân như leg_v3 (va chạm, rút servo, dụng cụ, 2 chân cạnh nhau).")
    a("- Roll vai bị giới hạn ≥ 0° (bản cũ −10°): dưới 0° cẳng tay chạm thân và đế gắn chân. Cập nhật giới hạn này "
      "trong URDF và firmware.")
    a("")
    a("## 11. Open Items")
    a("")
    a("1. Mọi kích thước MEASURE (servo, horn, module) — đo trước khi in.")
    a("2. Robot cao ~400 mm, nặng ước tính 0.85–0.95 kg có vỏ: chạy lại `mass_budget` / `torque_validation` và bench "
      "P0 với MG92B trước khi mua đủ servo (nguyên tắc A+: đo trên bàn trước, không nâng cấp theo mô phỏng).")
    a("3. URDF `mico.urdf.xacro` cần sinh lại từ `body.py` + `leg.py` (vị trí khớp mới, giới hạn roll vai 0…160°).")
    a("4. ToF lắp cùng mặt nghiêng 15° với camera (bản thiết kế cũ 20°): tia chạm sàn xa hơn ~20 %.")
    a("5. Vỏ Design C phải vẽ lại quanh khung này, bắt vào 16 tán vỏ đã chừa sẵn.")
    a("")
    text = "\n".join(md) + "\n"
    (DOCS / "mico_v4_lap_rap.md").write_text(text)
    (DOCS / "mico_v4_lap_rap.html").write_text(to_html(text))


def to_html(md):
    """A small markdown → HTML for this document (tables, headings, images inlined, lists, bold, code)."""
    out, table = [], []

    def inline(s):
        s = html.escape(s, quote=False)
        s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
        s = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", s)
        s = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: f'<a href="#{Path(m.group(2)).stem}">{m.group(1)}</a>'
                   if m.group(2).endswith(".png") else f"{m.group(1)}", s)
        return s

    def flush_table():
        if not table:
            return
        rows = [r for r in table if not re.match(r"^\|\s*-", r)]
        h = "<table>"
        for i, r in enumerate(rows):
            cells = [c.strip() for c in r.strip().strip("|").split("|")]
            tag = "th" if i == 0 else "td"
            h += "<tr>" + "".join(f"<{tag}>{inline(c)}</{tag}>" for c in cells) + "</tr>"
        out.append(h + "</table>")
        table.clear()

    for line in md.splitlines():
        if line.startswith("|"):
            table.append(line)
            continue
        flush_table()
        m = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)", line)
        if m:
            p = DOCS / m.group(2)
            b64 = base64.b64encode(p.read_bytes()).decode()
            out.append(f'<figure id="{p.stem}"><img src="data:image/png;base64,{b64}" alt="{html.escape(m.group(1))}">'
                       f"</figure>")
            continue
        m = re.match(r"^(#{1,4}) (.*)", line)
        if m:
            n = len(m.group(1))
            out.append(f"<h{n}>{inline(m.group(2))}</h{n}>")
            continue
        m = re.match(r"^(\d+)\. (.*)", line) or re.match(r"^- (.*)", line)
        if m:
            mark = "•" if line.startswith("-") else m.group(1) + "."
            out.append(f"<p class='li'>{mark} {inline(m.group(m.lastindex))}</p>")
            continue
        if line.strip():
            out.append(f"<p>{inline(line)}</p>")
    flush_table()
    css = """body{font-family:Inter,Segoe UI,Roboto,sans-serif;max-width:1100px;margin:24px auto;padding:0 20px;
color:#1C2B44;line-height:1.5}h1{font-size:28px;border-bottom:3px solid #F08C2E;padding-bottom:8px}
h2{margin-top:40px;border-bottom:1px solid #d6dce6;padding-bottom:4px}h3{margin-top:32px;background:#F3F5F8;
padding:6px 10px;border-left:4px solid #F08C2E}table{border-collapse:collapse;width:100%;font-size:13px;margin:10px 0}
td,th{border:1px solid #d6dce6;padding:4px 6px;vertical-align:top}th{background:#EEF2F7;text-align:left}
img{max-width:100%;border:1px solid #e3e7ee;border-radius:6px}code{background:#F3F5F8;padding:1px 4px;border-radius:3px;
font-size:12px}.li{margin:4px 0 4px 18px}figure{margin:10px 0;page-break-inside:avoid}
@media print{h3{page-break-before:always}body{max-width:none}}"""
    return (f"<!DOCTYPE html><html lang='vi'><head><meta charset='utf-8'><title>Mico v4 · Assembly Manual</title>"
            f"<style>{css}</style></head><body>{''.join(out)}</body></html>")


if __name__ == "__main__":
    sys.exit(main())
