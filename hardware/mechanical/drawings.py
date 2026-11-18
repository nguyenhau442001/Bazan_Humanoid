"""Engineering drawings of Bazan's parts, generated from the CAD: A3 sheets, first-angle projection (ISO E / TCVN),
hidden lines, 1:1 scale when the part fits, every hole and screw numbered with its coordinates.

    python drawings.py head        # docs/mechanical/drawings/head/*.pdf (+ .png previews)

One datum per assembly: every coordinate on every sheet of an assembly is measured from the same point O, so a
screw on two parts has the same numbers on both sheets. Head: O = head-pitch axis, middle of the servo pocket
(body.joints["head_pitch"].origin), axes = robot axes at the zero pose (x forward, y left, z up).

Sheet 1 of each part: front view (from +x), top view below it, left view on its right (first angle), an iso view,
overall dimensions, numbered holes, title block. Sheet 2: the hole and fastener table in full.
The assembly sheet: the parts together, item balloons, the bill of materials and every fastener with its place.
"""
import sys
from datetime import date
from pathlib import Path

import cadquery as cq
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.GCPnts import GCPnts_QuasiUniformDeflection  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt  # noqa: E402
from OCP.HLRAlgo import HLRAlgo_Projector  # noqa: E402
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402

import audit as A  # noqa: E402
import electronics as el  # noqa: E402
import shell as SH  # noqa: E402

HERE = Path(__file__).resolve().parent
DOCS = HERE.parents[1] / "docs" / "mechanical" / "drawings"
MM = 1 / 25.4                                  # inch per mm
A3 = (420.0, 297.0)
INK, HID, RED, BLUE = "#111111", "#555555", "#C0392B", "#1F5FA8"

# views: name, direction toward the eye, horizontal axis on the paper (first angle: top below, left view right)
VIEWS = {"front": ((1, 0, 0), (0, 1, 0), "Hình chiếu đứng (nhìn từ trước, +x)"),
         "top": ((0, 0, 1), (0, 1, 0), "Hình chiếu bằng (nhìn từ trên, +z)"),
         "left": ((0, -1, 0), (1, 0, 0), "Hình chiếu cạnh (nhìn từ bên phải robot, −y)"),
         "iso": ((1, -1, 1), (1, 1, 0), "Hình trục đo")}


# ------------------------------------------------------------------ projection
def _axes(view):
    d, x, _ = VIEWS[view]
    d, x = np.array(d, float), np.array(x, float)
    d, x = d / np.linalg.norm(d), x - d * np.dot(x, d)
    x /= np.linalg.norm(x)
    return d, x, np.cross(d, x)


def project(p, view):
    d, x, y = _axes(view)
    p = np.asarray(p, float)
    return np.array([p @ x, p @ y])


def hlr(shape, view):
    """Visible and hidden edges of shape seen along view, as 2D polylines (mm, paper axes)."""
    d, x, _ = _axes(view)
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d), gp_Dir(*x))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    out = {}
    for kind, getters in (("vis", (h.VCompound, h.OutLineVCompound, h.Rg1LineVCompound)),
                          ("hid", (h.HCompound, h.OutLineHCompound))):
        segs = []
        for g in getters:
            c = g()
            if c is None or c.IsNull():
                continue
            ex = TopExp_Explorer(c, TopAbs_EDGE)
            while ex.More():
                a = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
                q = GCPnts_QuasiUniformDeflection(a, 0.01)
                pts = [a.Value(q.Parameter(i)) for i in range(1, q.NbPoints() + 1)]
                segs.append(np.array([(p.X(), p.Y()) for p in pts]))
                ex.Next()
        out[kind] = segs
    return out


# ------------------------------------------------------------------ the head assembly
def head_parts():
    """{code: dict(name, solid, holes, material, qty, mass, print, note)}, all moved so the datum O is at 0."""
    sh = SH.BazanShell()
    body = sh.body
    O = np.array(body.joints["head_pitch"].origin, float)
    _, parts_g, _ = A.weigh(sh.leg, body, sh)
    holes = body.holes()

    def mv(s):
        return s.moved(cq.Location(cq.Vector(*(-O))))

    def own(part):
        return [dict(id=h.id.split(".")[-1], kind=h.kind, d=h.d, depth=h.depth, dir=tuple(np.round(h.dir, 3)),
                     pos=np.asarray(h.pos, float) - O, what=h.fastener) for h in holes if h.part == part]

    def shell_holes(part):
        out = []
        for k, (p, pt, d) in enumerate([s for s in sh.screws if s[0] == part]):
            out.append(dict(id=f"tab{k + 1}", kind="clr_m2", d=2.3, depth=0, dir=tuple(np.array(d, float)),
                            pos=np.asarray(pt, float) - O,
                            what="M2x6 xuống qua ống dẫn từ đỉnh đầu, vào tán N3.shell (đầu ốc nằm trên tab)"))
            out.append(dict(id=f"tube{k + 1}", kind="tube", d=2 * SH.TUBE_R[0], depth=float(SH.H_TOP - pt[2]),
                            dir=(0.0, 0.0, -1.0), pos=np.array([pt[0], pt[1], SH.H_TOP]) - O,
                            what="ống dẫn lục giác 1.5 + ốc, lỗ trên đỉnh đầu"))
        return out

    P = {
        "N3": dict(key="head_frame", name="Head Frame", solid=mv(body.parts["head_frame"]), holes=own("head_frame"),
                   material="PLA (PETG nếu servo nóng)", qty=1, mass=parts_g["head_frame"][0],
                   print="mặt sàn đầu úp xuống bàn, support dưới 2 đĩa"),
        "N4": dict(key="cam_mount", name="Camera + ToF Mount (nghiêng 15°)", solid=mv(body.parts["cam_mount"]),
                   holes=own("cam_mount"), material="PLA", qty=1, mass=parts_g["cam_mount"][0],
                   print="đế xuống bàn"),
        "C3": dict(key="head_front", name="Head Shell, Front", solid=mv(sh.parts["head_front"]),
                   holes=shell_holes("head_front"), material="PLA/PETG, thành 1.2", qty=1,
                   mass=parts_g["shell.head_front"][0], print="xem ghi chú: chưa chốt hướng in"),
        "C4": dict(key="head_back", name="Head Shell, Back", solid=mv(sh.parts["head_back"]),
                   holes=shell_holes("head_back"), material="PLA/PETG, thành 1.2", qty=1,
                   mass=parts_g["shell.head_back"][0], print="xem ghi chú: chưa chốt hướng in"),
        "C5": dict(key="visor", name="Visor", solid=mv(sh.parts["visor"]), holes=[],
                   material="tấm PETG/PC trong 0.5–1 (không in FDM)", qty=1, mass=parts_g["shell.visor"][0],
                   print="dán keo vào C3"),
    }
    bought = {"DISP": el.BOARDS["DISP"], "CAM": el.BOARDS["CAM"], "TOF": el.BOARDS["TOF"]}
    return P, O, body, sh, bought


KIND_VI = {"ins_m2_thru": "tán M2 xuyên", "ins_m2": "tán M2", "boss_m2": "tán M2 (trụ)", "ins_m3": "tán M3",
           "ins_m3_thru": "tán M3 xuyên", "cb_m2": "lỗ M2 + khoét chìm", "clr_m2": "lỗ thoát M2",
           "pilot_st2": "lỗ mồi ST2", "bearing": "ổ bạc đạn MR63ZZ", "tie": "khe dây rút", "access": "lỗ thao tác",
           "tube": "ống dẫn ốc"}


def dir_vi(d):
    d = tuple(np.round(np.asarray(d, float), 3))
    names = {(1, 0, 0): "+x", (-1, 0, 0): "−x", (0, 1, 0): "+y", (0, -1, 0): "−y", (0, 0, 1): "+z",
             (0, 0, -1): "−z"}
    return names.get(d, "(" + ", ".join(f"{v:+.2f}" for v in d) + ")")


# ------------------------------------------------------------------ sheet furniture
def frame(fig, title, code, sheet, sheets, scale, material="", mass=None, qty=None, print_=""):
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A3[0])
    ax.set_ylim(0, A3[1])
    ax.axis("off")
    ax.add_patch(plt.Rectangle((20, 10), A3[0] - 30, A3[1] - 20, fill=False, lw=1.0, color=INK))
    # title block 180 x 42 bottom right
    x0, y0, w = A3[0] - 10 - 180, 10, 180
    rows = [("Tên", f"{code} · {title}"), ("Vật liệu", material), ("SL / khối lượng",
            f"{qty if qty is not None else '–'} / {mass:.1f} g (ước tính)" if mass is not None else "–"),
            ("Hướng in", print_), ("Đơn vị / tỉ lệ", f"mm / {scale}  ·  dung sai chung ISO 2768-m  ·  "
                                                       "phép chiếu góc thứ nhất (E)"),
            ("Dự án", f"Bazan Humanoid · sinh từ CAD bằng drawings.py · {date.today().isoformat()} · "
                      f"tờ {sheet}/{sheets}")]
    h = 7.0
    for i, (k, v) in enumerate(rows):
        y = y0 + h * (len(rows) - 1 - i)
        ax.add_patch(plt.Rectangle((x0, y), w, h, fill=False, lw=0.6, color=INK))
        ax.plot([x0 + 30, x0 + 30], [y, y + h], color=INK, lw=0.6)
        ax.text(x0 + 1.5, y + h / 2, k, va="center", fontsize=6.5, color="#333")
        ax.text(x0 + 31.5, y + h / 2, v, va="center", fontsize=7.5 if i else 9.5,
                weight="bold" if i == 0 else "normal")
    # first-angle symbol
    sx, sy = x0 - 22, y0 + 6
    ax.add_patch(plt.Polygon([(sx, sy + 3), (sx + 9, sy), (sx + 9, sy + 12), (sx, sy + 9)], fill=False, lw=0.8))
    ax.add_patch(plt.Circle((sx + 15, sy + 6), 6, fill=False, lw=0.8))
    ax.add_patch(plt.Circle((sx + 15, sy + 6), 3, fill=False, lw=0.8))
    ax.text(sx + 7.5, sy - 3, "góc thứ nhất", ha="center", fontsize=5.5)
    return ax


def view_axes(fig, cx, cy, w, h, scale_n):
    """An axes on the sheet centred at (cx, cy) mm, w x h mm of paper, at 1:scale_n."""
    ax = fig.add_axes([(cx - w / 2) / A3[0], (cy - h / 2) / A3[1], w / A3[0], h / A3[1]])
    ax.set_xlim(-w / 2 * scale_n, w / 2 * scale_n)
    ax.set_ylim(-h / 2 * scale_n, h / 2 * scale_n)
    ax.set_aspect("equal")
    ax.axis("off")
    return ax


def dim(ax, p0, p1, off, text=None, horizontal=True, fs=6.5):
    """Linear dimension between two points, placed off (mm of model) away."""
    (x0, y0), (x1, y1) = p0, p1
    if horizontal:
        y = off
        ax.plot([x0, x0], [y0, y], color=BLUE, lw=0.4)
        ax.plot([x1, x1], [y1, y], color=BLUE, lw=0.4)
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", color=BLUE, lw=0.6, shrinkA=0, shrinkB=0))
        ax.text((x0 + x1) / 2, y + 0.8, text or f"{abs(x1 - x0):.1f}", ha="center", va="bottom", fontsize=fs,
                color=BLUE)
    else:
        x = off
        ax.plot([x0, x], [y0, y0], color=BLUE, lw=0.4)
        ax.plot([x1, x], [y1, y1], color=BLUE, lw=0.4)
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", color=BLUE, lw=0.6, shrinkA=0, shrinkB=0))
        ax.text(x - 0.8, (y0 + y1) / 2, text or f"{abs(y1 - y0):.1f}", ha="right", va="center", rotation=90,
                fontsize=fs, color=BLUE)


def draw_view(ax, solid, view, holes, title, number=True, dims=True):
    e = hlr(solid, view)
    for s in e["hid"]:
        ax.plot(s[:, 0], s[:, 1], color=HID, lw=0.35, ls=(0, (3, 2)))
    for s in e["vis"]:
        ax.plot(s[:, 0], s[:, 1], color=INK, lw=0.7)
    allp = np.vstack(e["vis"] + e["hid"]) if e["vis"] else np.zeros((1, 2))
    lo, hi = allp.min(0), allp.max(0)
    # datum O
    o = project((0, 0, 0), view)
    ax.plot(*o, marker="+", color=RED, ms=9, mew=1.0)
    ax.annotate("O", o, xytext=(3, -8), textcoords="offset points", fontsize=7, color=RED, weight="bold")
    d, _, _ = _axes(view)
    if number:
        for i, hl in enumerate(holes):
            p = project(hl["pos"], view)
            along = abs(np.dot(np.asarray(hl["dir"], float), d)) > 0.9
            if along:
                ax.add_patch(plt.Circle(p, hl["d"] / 2, fill=False, color=RED, lw=0.6))
                ax.plot([p[0] - hl["d"] * 0.8, p[0] + hl["d"] * 0.8], [p[1], p[1]], color=RED, lw=0.3, ls="-.")
                ax.plot([p[0], p[0]], [p[1] - hl["d"] * 0.8, p[1] + hl["d"] * 0.8], color=RED, lw=0.3, ls="-.")
            else:
                q = project(np.asarray(hl["pos"]) + np.asarray(hl["dir"]) * (hl["depth"] or 3.0), view)
                ax.plot([p[0], q[0]], [p[1], q[1]], color=RED, lw=0.5, ls="-.")
            ax.annotate(f"{i + 1}", p, xytext=(5, 5 + 7 * (i % 2)), textcoords="offset points", fontsize=6,
                        color=RED, arrowprops=dict(arrowstyle="-", color=RED, lw=0.3),
                        bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec=RED, lw=0.4))
    if dims:
        span = hi - lo
        pad = max(span.max() * 0.08, 4.0)
        dim(ax, (lo[0], lo[1]), (hi[0], lo[1]), lo[1] - pad, horizontal=True)
        dim(ax, (lo[0], lo[1]), (lo[0], hi[1]), lo[0] - pad, horizontal=False)
        # datum to the outline: where O sits in this view (only when O is on or near the part)
        if np.all(o >= lo - 5.0) and np.all(o <= hi + 5.0):
            dim(ax, (o[0], o[1]), (lo[0], o[1]), lo[1] - 2 * pad, text=f"O↔{abs(o[0] - lo[0]):.1f}",
                horizontal=True)
            dim(ax, (o[0], o[1]), (o[0], lo[1]), hi[0] + pad, text=f"O↔{abs(o[1] - lo[1]):.1f}", horizontal=False)
    ax.set_title(title, fontsize=7.5, loc="left")
    return lo, hi


def fit_scale(extents, boxes):
    """Smallest standard scale 1:n that fits every view's extent into its paper box."""
    for n in (0.5, 1, 1.5, 2, 2.5, 3, 4, 5):                # 0.5 = enlarged 2:1
        if all(e[0] / n <= b[0] * 0.86 and e[1] / n <= b[1] * 0.86 for e, b in zip(extents, boxes)):
            return n
    return 5


def part_sheets(pdf, code, p, png_dir):
    s, holes = p["solid"], p["holes"]
    ext = []
    for v in ("front", "top", "left"):
        bb = np.array([project(c, v) for c in _bbox_corners(s)])
        ext.append(bb.max(0) - bb.min(0))
    boxes = [(160, 130), (160, 130), (140, 130)]
    n = fit_scale(ext, boxes)
    scale = "2:1" if n == 0.5 else ("1:1" if n == 1 else f"1:{n:g}")
    fig = plt.figure(figsize=(A3[0] * MM, A3[1] * MM))
    frame(fig, p["name"], code, 1, 2, scale, p["material"], p["mass"], p["qty"], p["print"])
    # first-angle layout: front top-left, top below it, left view on the right of the front view, iso far right
    c = _centre(s)
    places = {"front": (105, 215, 160, 130), "top": (105, 85, 160, 130), "left": (258, 215, 140, 130)}
    for v, (cx, cy, w, h) in places.items():
        ax = view_axes(fig, cx, cy, w, h, n)
        cc = project(c, v)
        ax.set_xlim(cc[0] - w / 2 * n, cc[0] + w / 2 * n)
        ax.set_ylim(cc[1] - h / 2 * n, cc[1] + h / 2 * n)
        draw_view(ax, s, v, holes, VIEWS[v][2])
    ax = view_axes(fig, 268, 100, 120, 85, max(n, 1) * 1.2)
    cc = project(c, "iso")
    k = max(n, 1) * 1.2
    ax.set_xlim(cc[0] - 60 * k, cc[0] + 60 * k)
    ax.set_ylim(cc[1] - 42.5 * k, cc[1] + 42.5 * k)
    draw_view(ax, s, "iso", holes, "Hình trục đo (không ghi kích thước)", number=False, dims=False)
    # compact hole table, right column
    tab = fig.add_axes([333 / A3[0], 150 / A3[1], 82 / A3[0], 136 / A3[1]])
    tab.axis("off")
    tab.text(0, 1.0, "Bảng lỗ (toạ độ từ O, mm)", fontsize=7, weight="bold", va="top", transform=tab.transAxes)
    head = f"{'#':>2} {'X':>7} {'Y':>7} {'Z':>7} {'Ø':>5} {'sâu':>5} {'hướng':>6}"
    def depth(h):
        return f"{h['depth']:.1f}" if h["depth"] else "xuyên"

    lines = [head] + [f"{i + 1:>2} {h['pos'][0]:7.2f} {h['pos'][1]:7.2f} {h['pos'][2]:7.2f} {h['d']:5.2f} "
                      f"{depth(h):>5} {dir_vi(h['dir']):>6}" for i, h in enumerate(holes)]
    tab.text(0, 0.975, "\n".join(lines), fontsize=6.0, family="DejaVu Sans Mono", va="top", transform=tab.transAxes)
    tab.text(0, 0.0, "Hướng = chiều mũi khoan / chiều ốc đi vào.\nChi tiết từng lỗ: tờ 2.", fontsize=5.5,
             transform=tab.transAxes)
    pdf.savefig(fig)
    fig.savefig(png_dir / f"{code}_{p['key']}_1.png", dpi=110)
    plt.close(fig)
    # sheet 2: the full table
    fig = plt.figure(figsize=(A3[0] * MM, A3[1] * MM))
    frame(fig, p["name"], code, 2, 2, scale, p["material"], p["mass"], p["qty"], p["print"])
    ax = fig.add_axes([25 / A3[0], 58 / A3[1], 385 / A3[0], 228 / A3[1]])
    ax.axis("off")
    ax.text(0, 1, f"{code} · {p['name']} — bảng lỗ, tán cấy và ốc (mm). (O) = từ gốc cụm đầu O; (mép) = từ "
            "mặt sau (x nhỏ nhất), mặt phải (y nhỏ nhất) và mặt dưới (z nhỏ nhất) của chính chi tiết",
            fontsize=10, weight="bold", va="top", transform=ax.transAxes)
    bb = s.BoundingBox()
    cmin = np.array([bb.xmin, bb.ymin, bb.zmin])
    rows = [["#", "ID trong CAD", "Loại", "Ø", "Sâu", "Hướng", "X (O)", "Y (O)", "Z (O)", "X (mép)", "Y (mép)",
             "Z (mép)", "Ốc / tán / chức năng"]]
    for i, h in enumerate(holes):
        e = h["pos"] - cmin
        rows.append([str(i + 1), h["id"], KIND_VI.get(h["kind"], h["kind"]), f"{h['d']:.2f}",
                     f"{h['depth']:.1f}" if h["depth"] else "xuyên", dir_vi(h["dir"]), f"{h['pos'][0]:.2f}",
                     f"{h['pos'][1]:.2f}", f"{h['pos'][2]:.2f}", f"{e[0]:.2f}", f"{e[1]:.2f}", f"{e[2]:.2f}",
                     h["what"][:80]])
    if len(rows) == 1:
        rows.append(["–", "không có lỗ", "", "", "", "", "", "", "", "", "", "", "chi tiết dán, không bắt ốc"])
    t = ax.table(cellText=rows, cellLoc="left",
                 colWidths=[0.022, 0.07, 0.085, 0.035, 0.035, 0.1, 0.045, 0.045, 0.045, 0.045, 0.045, 0.045, 0.35],
                 loc="upper left", bbox=[0, 1 - 0.035 * (len(rows) + 1), 1, 0.035 * len(rows)])
    t.auto_set_font_size(False)
    t.set_fontsize(6.5)
    for (r, _), cell in t.get_celld().items():
        cell.set_linewidth(0.4)
        if r == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#eef2f7")
    notes = NOTES.get(p["key"], [])
    if notes:
        ax.text(0, 1 - 0.035 * (len(rows) + 2), "Ghi chú kỹ thuật:\n" + "\n".join(f"  {k + 1}. {s}" for k, s in
                                                                                    enumerate(notes)),
                fontsize=7.5, va="top", transform=ax.transAxes)
    pdf.savefig(fig)
    fig.savefig(png_dir / f"{code}_{p['key']}_2.png", dpi=110)
    plt.close(fig)


NOTES = {
    "head_frame": ["Tán cấy M2 (OD 3.2, dài 3) ép bằng mỏ hàn 220 °C, bằng mặt trên sàn đầu; sàn dày 3 nên kiểm tra "
                   "mặt dưới không phồng (phiếu thử T0, lỗ D).",
                   "Đĩa horn head pitch: horn tròn SG90 nằm phẳng trong hốc, 2 vít ST2×4 qua horn vào lỗ mồi.",
                   "Đĩa idler: bạc đạn MR63ZZ ép vào ổ, trục là ốc đầu dù M3×6 + long đền W3 vào tán M3 trên N2.",
                   "4 khe dây rút trên mặt trước giữ module màn hình; chưa có chỗ cho header dây màn hình (hở 0.8 "
                   "mm hai bên) — cần chỉnh."],
    "cam_mount": ["Mặt trước nghiêng 15° xuống. 4 tán M2 camera và 2 tán ToF ép TỪ MẶT TRƯỚC vào trụ.",
                  "Lỗ Ø9 giữa 4 trụ camera là chỗ ống kính và cáp FPC; cáp đi xuống sau đế.",
                  "Bắt vào N3 bằng 2 × M2×6 qua lỗ khoét chìm của đế (đầu ốc chìm)."],
    "head_front": ["Bắt bằng 2 ốc M2×6 thả xuống ống dẫn từ đỉnh đầu (lục giác 1.5 dài ≥ 45, nên có đầu nam châm).",
                   "Mép ghép với C4 tại x = −12 (từ gốc robot) — xem bản vẽ lắp.",
                   "Hướng in chưa chốt: cần quyết trước khi in (đỉnh đầu bo tròn → in ngược cần support)."],
    "head_back": ["Như C3: 2 ốc M2×6 qua ống dẫn; rãnh quanh trục gật đầu để mép sau không chạm servo cổ.",
                  "Tai tròn hai bên có lưới lỗ micro."],
    "visor": ["KHÔNG in FDM (PETG in ra đục mờ, màn hình mắt sẽ nhòe): cắt/uốn từ tấm PETG hoặc PC trong 0.5–1 mm.",
              "Dán vào C3 bằng keo UV hoặc băng keo 2 mặt trong."],
}


def _bbox_corners(s):
    b = s.BoundingBox()
    return [(x, y, z) for x in (b.xmin, b.xmax) for y in (b.ymin, b.ymax) for z in (b.zmin, b.zmax)]


def _centre(s):
    b = s.BoundingBox()
    return np.array([(b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, (b.zmin + b.zmax) / 2])


# ------------------------------------------------------------------ assembly sheet
def assembly_sheet(pdf, P, O, body, bought, png_dir):
    comp = cq.Compound.makeCompound([p["solid"] for k, p in P.items() if k != "C5"])
    fig = plt.figure(figsize=(A3[0] * MM, A3[1] * MM))
    frame(fig, "Cụm đầu — bản vẽ lắp", "H0", 1, 1, "1:1.5", "xem bảng kê", None, None, "–")
    for v, (cx, cy, w, h) in {"front": (90, 205, 130, 120), "left": (215, 205, 120, 120)}.items():
        ax = view_axes(fig, cx, cy, w, h, 1.5)
        c = project(_centre(comp), v)
        ax.set_xlim(c[0] - w / 2 * 1.5, c[0] + w / 2 * 1.5)
        ax.set_ylim(c[1] - h / 2 * 1.5, c[1] + h / 2 * 1.5)
        draw_view(ax, comp, v, [], VIEWS[v][2], number=False, dims=True)
        for k, p in P.items():
            if k == "C5":
                continue
            q = project(_centre(p["solid"]), v)
            ax.annotate(k, q, xytext=(12, 10), textcoords="offset points", fontsize=7, color=RED, weight="bold",
                        arrowprops=dict(arrowstyle="-", color=RED, lw=0.5),
                        bbox=dict(boxstyle="circle,pad=0.25", fc="white", ec=RED, lw=0.6))
    # bill of materials and fastener list
    ax = fig.add_axes([25 / A3[0], 56 / A3[1], 385 / A3[0], 82 / A3[1]])
    ax.axis("off")
    rows = [["Mục", "Tên", "SL", "Vật liệu / mua", "Ghi chú"]]
    for k, p in P.items():
        rows.append([k, p["name"], str(p["qty"]), p["material"], f"{p['mass']:.1f} g · in: {p['print']}"])
    for k, b in bought.items():
        rows.append([k, b.name, "1", b.shop, f"{b.size[0]:g} × {b.size[1]:g}, lỗ Ø{b.hole_d:g} · MEASURE"])
    rows.append(["SG90", "servo head pitch (thuộc cụm cổ N2) + horn tròn", "1", "SG90", "horn bắt vào N3"])
    t = ax.table(cellText=rows, cellLoc="left", colWidths=[0.05, 0.28, 0.04, 0.27, 0.36], loc="upper left",
                 bbox=[0, 1 - 0.09 * len(rows), 0.62, 0.09 * len(rows)])
    t.auto_set_font_size(False)
    t.set_fontsize(6.3)
    # fasteners grouped
    from collections import Counter
    cnt = Counter()
    for k, p in P.items():
        for h in p["holes"]:
            key = {"ins_m2_thru": "Tán cấy M2", "boss_m2": "Tán cấy M2", "ins_m2": "Tán cấy M2",
                   "cb_m2": "Ốc M2×6 (N4 → N3)", "clr_m2": "Ốc M2×6 (vỏ → N3)", "tie": "Dây rút 2.5 mm (màn hình)",
                   "pilot_st2": "Vít ST2×4 (horn)", "bearing": "MR63ZZ + M3×6 đầu dù + W3",
                   "ins_m3": "Tán cấy M3", "ins_m3_thru": "Tán cấy M3"}.get(h["kind"])
            if key:
                cnt[key] += 1
    cnt["Ốc M2×6 (camera, ToF → N4)"] = sum(1 for h in P["N4"]["holes"] if h["kind"] == "boss_m2")
    cnt["Dây rút 2.5 mm (màn hình)"] = cnt["Dây rút 2.5 mm (màn hình)"] // 2
    fr = [["Ốc / tán / phụ kiện (cụm đầu)", "SL"]] + [[k, str(v)] for k, v in sorted(cnt.items())]
    t2 = ax.table(cellText=fr, cellLoc="left", colWidths=[0.3, 0.05], loc="upper right",
                  bbox=[0.64, 1 - 0.09 * len(fr), 0.36, 0.09 * len(fr)])
    t2.auto_set_font_size(False)
    t2.set_fontsize(6.3)
    for tt in (t, t2):
        for (r, _), cell in tt.get_celld().items():
            cell.set_linewidth(0.4)
            if r == 0:
                cell.set_text_props(weight="bold")
                cell.set_facecolor("#eef2f7")
    o_world = np.round(O, 2)
    fig.text(300 / A3[0], 270 / A3[1],
             "Gốc O (mọi toạ độ của cụm đầu):\n"
             f"trục gật đầu, giữa ổ servo = ({o_world[0]}, {o_world[1]}, {o_world[2]}) mm\n"
             "trong hệ robot (gốc giữa 2 điểm hông H).\n\nThứ tự lắp:\n"
             "1. Cấy tán vào N3 (4 vỏ, 2 bay camera) và N4.\n"
             "2. Bắt camera, ToF vào N4; N4 vào N3 (2 × M2×6).\n"
             "3. Buộc màn hình vào mặt trước N3 (2 dây rút).\n"
             "4. Gắn N3 lên khớp head pitch: horn + 2 ST2×4,\n    idler MR63ZZ + M3×6 + W3.\n"
             "5. Đi cáp camera, màn hình xuống cổ (có vòng chùng).\n"
             "6. Úp C4 rồi C3; 4 × M2×6 qua ống dẫn từ đỉnh.\n7. Dán C5 (visor) vào C3.",
             fontsize=6.8, va="top")
    pdf.savefig(fig)
    fig.savefig(png_dir / "H0_head_assembly.png", dpi=110)
    plt.close(fig)


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "head"
    if which != "head":
        raise SystemExit("only 'head' so far")
    out = DOCS / "head"
    out.mkdir(parents=True, exist_ok=True)
    P, O, body, sh, bought = head_parts()
    with PdfPages(out / "head_drawings.pdf") as pdf:
        assembly_sheet(pdf, P, O, body, bought, out)
        for code, p in P.items():
            part_sheets(pdf, code, p, out)
            print("drew", code, p["key"], len(p["holes"]), "holes")
    print("wrote", out / "head_drawings.pdf")


if __name__ == "__main__":
    main()
