"""Manufacturing drawings of Bazan's parts, generated from the CAD (drawing engine v2).

    python drawings.py N3          # docs/mechanical/drawings/N3/: BZN-H-N3.pdf, sheet PNGs, validation report
    python drawings.py N3 --check  # exit 1 if the validation finds a defect

Rules this engine follows (references, not a compliance claim):
    ISO 128      line types and widths: visible 0.5, hidden / centre / dimension / hatching 0.25 (mm, A3)
    ISO 129-1    dimensions taken from functional datums; coordinate table for the hole pattern
    ISO 5455     standard scales only (5:1, 4:1, 2:1, 1:1, 1:2 ...)
    ISO 5456-2   first-angle projection: top view below the front view, view from the left on its right
    ISO 5457     A3 landscape, frame 20 mm left / 10 mm elsewhere, zone grid
    ISO 7200     title block fields; unknown fields are written TBD, never invented
    ISO 1101     a position tolerance only where a mating clearance fixes it (fixed-fastener formula)

Every number on a sheet comes from the CAD code (joint.py, body.py, hardware.py) and is checked against the
solid itself by validate(): each dimension's two end points must lie on the part's surface and the printed value
must equal their distance. Nothing is measured off the picture.
"""
import json
import sys
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

import cadquery as cq
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.path import Path as MPath  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.GCPnts import GCPnts_QuasiUniformDeflection  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt  # noqa: E402
from OCP.HLRAlgo import HLRAlgo_Projector  # noqa: E402
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.geometry import box as shp_box  # noqa: E402

HERE = Path(__file__).resolve().parent
DOCS = HERE.parents[1] / "docs" / "mechanical" / "drawings"
MM_IN = 1 / 25.4
A3 = (420.0, 297.0)
BORDER = (20.0, 10.0, 410.0, 287.0)               # x0, y0, x1, y1 of the frame (ISO 5457)
PT = 72 / 25.4                                     # points per mm
W_THICK, W_THIN = 0.5 * PT, 0.25 * PT              # ISO 128 line widths for A3
FS_DIM, FS_NOTE, FS_TAG, FS_TITLE = 8.0, 7.5, 7.5, 11.0
INK = "black"
DEPTH = "↧"

VIEWS = {  # name: (towards the eye, paper right)
    "front": ((1, 0, 0), (0, 1, 0)),     # from +x: robot left (+y) on the right
    "top": ((0, 0, 1), (0, 1, 0)),       # from +z, placed below the front view: +x points down the sheet
    "left": ((0, -1, 0), (1, 0, 0)),     # from -y (the viewer's left of the front view), placed on its right
    "iso": ((1, -1, 1), (1, 1, 0)),
}


def view_axes(view):
    d, x = (np.array(v, float) for v in VIEWS[view])
    d /= np.linalg.norm(d)
    x -= d * np.dot(x, d)
    x /= np.linalg.norm(x)
    return d, x, np.cross(d, x)


def project(p, view):
    d, x, y = view_axes(view)
    p = np.asarray(p, float)
    return np.array([p @ x, p @ y])


def _edges(compound, defl=0.005):
    segs = []
    if compound is None or compound.IsNull():
        return segs
    ex = TopExp_Explorer(compound, TopAbs_EDGE)
    while ex.More():
        a = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
        q = GCPnts_QuasiUniformDeflection(a, defl)
        segs.append(np.array([(a.Value(q.Parameter(i)).X(), a.Value(q.Parameter(i)).Y())
                              for i in range(1, q.NbPoints() + 1)]))
        ex.Next()
    return segs


def hlr(shape, view):
    d, x, _ = view_axes(view)
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d), gp_Dir(*x))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis = _edges(h.VCompound()) + _edges(h.OutLineVCompound()) + _edges(h.Rg1LineVCompound())
    hid = _edges(h.HCompound()) + _edges(h.OutLineHCompound())
    return vis, hid


def section(shape, point, view):
    """Cut shape by the plane through point normal to the view direction, keep the half away from the eye.
    Returns (visible edges of the kept half, cut faces as lists of 2D loops)."""
    d, _, _ = view_axes(view)
    point = np.asarray(point, float)
    big = 400.0
    box = cq.Solid.makeBox(2 * big, 2 * big, big, cq.Vector(-big, -big, 0))
    # orient the box so it fills the eye side of the plane
    z = cq.Vector(*d)
    plane = cq.Plane(origin=cq.Vector(*point), xDir=cq.Vector(*view_axes(view)[1]), normal=z)
    box = box.moved(cq.Location(plane))
    half = shape.cut(box).clean()
    vis, _ = hlr(half, view)
    loops = []
    for f in half.Faces():
        n = f.normalAt()
        c = f.Center()
        if abs(abs(np.dot([n.x, n.y, n.z], d)) - 1) < 1e-6 and abs(np.dot(np.array([c.x, c.y, c.z]) - point, d)) < 1e-4:
            rings = []
            for w in [f.outerWire()] + list(f.innerWires()):
                pts = np.array([[p.x, p.y, p.z] for p in w.positions(np.linspace(0, 1, 240))])
                rings.append([tuple(project(p, view)) for p in pts])
            loops.append(Polygon(rings[0], rings[1:]).buffer(0))
    return vis, loops, half


def clip(segs, polys, box):
    """Visible edges and section polygons cut to a detail window box = (u0, v0, u1, v1)."""
    win = shp_box(*box)
    out = []
    for sg in segs:
        if len(sg) < 2:
            continue
        g = LineString(sg).intersection(win)
        for part in getattr(g, "geoms", [g]):
            if not part.is_empty and part.geom_type == "LineString":
                out.append(np.asarray(part.coords))
    return out, [p.intersection(win) for p in polys]


# ------------------------------------------------------------------ the sheet
@dataclass
class Dim:
    """A dimension: two points on the part (model mm, relative to the datum) and the value shown."""
    p0: tuple
    p1: tuple
    axis: tuple            # model direction the value is measured along
    text: str
    value: float
    view: str
    on_surface: bool = True


def fmt(v):
    """ISO 129-1 style: no trailing zeros (6.6, 31.9, 6.05)."""
    t = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


@dataclass
class Sheet:
    number: int
    title: str
    fig: object = None
    ax: object = None
    texts: list = field(default_factory=list)       # (artist, role) for the collision check
    views: dict = field(default_factory=dict)       # name -> (cx, cy, uc, vc, k, model view)
    bbox: dict = field(default_factory=dict)        # name -> [x0, y0, x1, y1] of the drawn geometry (paper mm)
    used: dict = field(default_factory=dict)        # name -> {side: next free offset}
    dims: list = field(default_factory=list)
    boxes: list = field(default_factory=list)       # approximate text boxes placed so far (for tag placement)

    def __post_init__(self):
        self.fig = plt.figure(figsize=(A3[0] * MM_IN, A3[1] * MM_IN))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, A3[0])
        self.ax.set_ylim(0, A3[1])
        self.ax.axis("off")

    # paper primitives (mm)
    def line(self, xs, ys, w=W_THIN, ls="-", **kw):
        self.ax.plot(xs, ys, color=INK, lw=w, ls=ls, solid_capstyle="butt", **kw)

    def text(self, x, y, s, fs=FS_NOTE, role="note", **kw):
        t = self.ax.text(x, y, s, fontsize=fs, color=INK, family="DejaVu Sans", **kw)
        self.texts.append((t, role))
        return t

    def arrow(self, tip, frm, size=2.5):
        tip, frm = np.asarray(tip, float), np.asarray(frm, float)
        d = (tip - frm) / (np.linalg.norm(tip - frm) + 1e-12)
        nrm = np.array([-d[1], d[0]])
        base = tip - d * size
        self.ax.fill(*np.array([tip, base + nrm * size * 0.18, base - nrm * size * 0.18]).T, color=INK, lw=0)

    # views
    def place(self, name, model_view, cx, cy, uc, vc, k):
        self.views[name] = (cx, cy, uc, vc, k, model_view)
        self.used[name] = {"above": 0.0, "below": 0.0, "left": 0.0, "right": 0.0}

    def P(self, name, p3):
        cx, cy, uc, vc, k, mv = self.views[name]
        u, v = project(p3, mv)
        return np.array([cx + (u - uc) * k, cy + (v - vc) * k])

    def P2(self, name, uv):
        cx, cy, uc, vc, k, _ = self.views[name]
        return np.array([cx + (uv[0] - uc) * k, cy + (uv[1] - vc) * k])

    def _grow(self, name, q):
        b = self.bbox.setdefault(name, [np.inf, np.inf, -np.inf, -np.inf])
        b[0], b[1] = min(b[0], q[:, 0].min()), min(b[1], q[:, 1].min())
        b[2], b[3] = max(b[2], q[:, 0].max()), max(b[3], q[:, 1].max())

    def draw_edges(self, name, segs, w=W_THICK, ls="-", grow=True):
        for s in segs:
            q = np.array([self.P2(name, p) for p in s])
            if len(q) < 2:
                continue
            self.line(q[:, 0], q[:, 1], w=w, ls=ls)
            if grow:
                self._grow(name, q)

    def hatch(self, name, faces):
        """faces: list of shapely polygons in model view coordinates (u, v)."""
        for poly in faces:
            if poly.is_empty:
                continue
            for g in getattr(poly, "geoms", [poly]):
                verts, codes = [], []
                for i, ring in enumerate([g.exterior] + list(g.interiors)):
                    q = np.array([self.P2(name, p) for p in np.asarray(ring.coords)])
                    area = 0.5 * np.sum(q[:-1, 0] * q[1:, 1] - q[1:, 0] * q[:-1, 1])
                    if (i == 0) != (area > 0):
                        q = q[::-1]
                    verts += list(q)
                    codes += [MPath.MOVETO] + [MPath.LINETO] * (len(q) - 2) + [MPath.CLOSEPOLY]
                    self._grow(name, q)
                self.ax.add_patch(PathPatch(MPath(verts, codes), facecolor="none", edgecolor=INK, hatch="////",
                                            lw=W_THICK))

    # dimensions in paper space, always placed outside the view's outline, stacked per side
    def dim(self, name, p0, p1, axis, where, text=None, prefix="", on_surface=True, gap=8.0, tpos=0.5):
        a = np.asarray(axis, float)
        P0, P1 = self.P(name, p0), self.P(name, p1)
        val = float(abs(np.dot(np.asarray(p1, float) - np.asarray(p0, float), a)))
        text = text or f"{prefix}{fmt(val)}"
        x0, y0, x1, y1 = self.bbox[name]
        off = self.used[name][where] + gap
        self.used[name][where] = off
        if where in ("above", "below"):
            yb = y1 + off if where == "above" else y0 - off
            Q0, Q1 = np.array([P0[0], yb]), np.array([P1[0], yb])
            sgn = 1 if where == "above" else -1
            ext = [(P0, Q0), (P1, Q1)]
            nrm = np.array([0, sgn])
        else:
            xb = x1 + off if where == "right" else x0 - off
            Q0, Q1 = np.array([xb, P0[1]]), np.array([xb, P1[1]])
            sgn = 1 if where == "right" else -1
            ext = [(P0, Q0), (P1, Q1)]
            nrm = np.array([sgn, 0])
        for P_, Q_ in ext:
            gapv = 1.0 if np.linalg.norm(Q_ - P_) > 2 else 0.0
            self.line([P_[0] + nrm[0] * gapv, Q_[0] + nrm[0] * 2.0], [P_[1] + nrm[1] * gapv, Q_[1] + nrm[1] * 2.0])
        self.line([Q0[0], Q1[0]], [Q0[1], Q1[1]])
        L_ = np.linalg.norm(Q1 - Q0)
        d = (Q1 - Q0) / (L_ + 1e-12)
        tw = 0.18 * FS_DIM * len(text) / 1.0 * 0.62
        if L_ > 7 and L_ > tw + 6:
            self.arrow(Q0, Q1)
            self.arrow(Q1, Q0)
            mid = Q0 + (Q1 - Q0) * tpos
        else:                                         # arrows from outside, text beside the far arrow
            self.line([Q0[0] - d[0] * 5, Q0[0]], [Q0[1] - d[1] * 5, Q0[1]])
            self.line([Q1[0], Q1[0] + d[0] * (6 + tw)], [Q1[1], Q1[1] + d[1] * (6 + tw)])
            self.arrow(Q0, Q0 - d * 3)
            self.arrow(Q1, Q1 + d * 3)
            mid = Q1 + d * (3.5 + tw / 2)
        horizontal = where in ("above", "below")
        self.text(mid[0] + (0 if horizontal else -0.8), mid[1] + (0.8 if horizontal else 0), text, fs=FS_DIM,
                  role="dim", ha="center", va="bottom", rotation=0 if horizontal else 90, rotation_mode="anchor")
        self.dims.append(Dim(tuple(p0), tuple(p1), tuple(a), text, val, name, on_surface))

    def _free(self, x, y, w, h):
        for bx in self.boxes:
            if x < bx[2] and bx[0] < x + w and y < bx[3] and bx[1] < y + h:
                return False
        return True

    def tag(self, name, p3, label):
        """Feature tag next to its feature, short leader, placed where no other tag or note is."""
        P = self.P(name, p3)
        w, h = 1.9 * len(label) + 1.0, 3.2
        for r in (4.0, 6.0, 8.5, 11.0):
            for ang in (45, 135, -45, -135, 0, 180, 90, -90):
                d = np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))])
                c = P + d * r
                x, y = c[0] - w / 2, c[1] - h / 2
                if self._free(x, y, w, h):
                    self.boxes.append((x, y, x + w, y + h))
                    self.line([P[0], c[0] - d[0] * (w / 2 if abs(d[0]) > 0.3 else 0)],
                              [P[1], c[1] - d[1] * (h / 2 if abs(d[1]) > 0.3 else 0)], w=W_THIN * 0.8)
                    self.text(c[0], c[1], label, fs=FS_TAG, role="tag", ha="center", va="center")
                    return
        self.text(P[0] + 3, P[1] + 3, label, fs=FS_TAG, role="tag")

    def centermark(self, name, p3, r):
        P = self.P(name, p3)
        k = self.views[name][4]
        e = r * k + 1.5
        self.line([P[0] - e, P[0] + e], [P[1], P[1]], ls=(0, (8, 1.5, 1, 1.5)))
        self.line([P[0], P[0]], [P[1] - e, P[1] + e], ls=(0, (8, 1.5, 1, 1.5)))

    def centerline(self, name, p0, p1, ext=3.0):
        A_, B_ = self.P(name, p0), self.P(name, p1)
        d = (B_ - A_) / (np.linalg.norm(B_ - A_) + 1e-12)
        A_, B_ = A_ - d * ext, B_ + d * ext
        self.line([A_[0], B_[0]], [A_[1], B_[1]], ls=(0, (8, 1.5, 1, 1.5)))

    def caption(self, name, title, scale):
        x0, y0, x1, y1 = self.bbox[name]
        y = y1 + self.used[name]["above"] + 7.0
        self.text((x0 + x1) / 2, y, f"{title}   ({scale})", fs=9, role="caption", ha="center", va="bottom",
                  weight="bold")

    # frame, zones and the ISO 7200 title block
    def furniture(self, tb, total):
        x0, y0, x1, y1 = BORDER
        self.ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=0.7 * PT, color=INK))
        self.ax.add_patch(plt.Rectangle((x0 - 5, y0 - 5), x1 - x0 + 10, y1 - y0 + 10, fill=False, lw=W_THIN,
                                        color=INK)) if False else None
        # zone grid: 8 columns (1..8), 6 rows (A..F)
        for i in range(9):
            x = x0 + (x1 - x0) * i / 8
            self.line([x, x], [y1, y1 + 3]) if 0 < i < 8 else None
            self.line([x, x], [y0 - 3, y0]) if 0 < i < 8 else None
            if i < 8:
                xm = x0 + (x1 - x0) * (i + 0.5) / 8
                self.text(xm, y1 + 1.5, str(i + 1), fs=6, role="zone", ha="center", va="center")
                self.text(xm, y0 - 1.5, str(i + 1), fs=6, role="zone", ha="center", va="center")
        for j in range(7):
            y = y0 + (y1 - y0) * j / 6
            if 0 < j < 6:
                self.line([x0 - 3, x0], [y, y])
                self.line([x1, x1 + 3], [y, y])
            if j < 6:
                ym = y0 + (y1 - y0) * (j + 0.5) / 6
                lab = "FEDCBA"[j]
                self.text(x0 - 1.5, ym, lab, fs=6, role="zone", ha="center", va="center")
                self.text(x1 + 1.5, ym, lab, fs=6, role="zone", ha="center", va="center")
        # title block: 180 wide, bottom right
        w, rows = 180.0, 6
        bx, by, h = x1 - w, y0, 8.0
        self.ax.add_patch(plt.Rectangle((bx, by), w, h * rows, fill=False, lw=0.5 * PT, color=INK))
        cells = [  # (row from the bottom, x from, x to, label, value, big)
            (5, 0, 180, "Chủ sở hữu / dự án", f"{tb['owner']}  ·  {tb['project']}", False),
            (4, 0, 120, "Tên chi tiết (title)", tb["title"], True),
            (4, 120, 180, "Số bản vẽ", tb["number"], True),
            (3, 0, 60, "Vật liệu", tb["material"], False),
            (3, 60, 95, "Khối lượng", tb["mass"], False),
            (3, 95, 125, "Tỉ lệ", tb["scale"], False),
            (3, 125, 155, "Đơn vị", "mm", False),
            (3, 155, 180, "Phép chiếu", "", False),
            (2, 0, 60, "Thiết kế", tb["designed"], False),
            (2, 60, 120, "Kiểm tra", tb["checked"], False),
            (2, 120, 180, "Phê duyệt", tb["approved"], False),
            (1, 0, 60, "Ngày phát hành", tb["date"], False),
            (1, 60, 95, "Lần sửa (rev)", tb["rev"], False),
            (1, 95, 140, "Trạng thái", tb["status"], False),
            (1, 140, 180, "Tờ", f"{self.number} / {total}", False),
            (0, 0, 180, "Tiêu chuẩn tham chiếu", tb["standards"], False),
        ]
        for r, a, b, lab, val, big in cells:
            self.ax.add_patch(plt.Rectangle((bx + a, by + r * h), b - a, h, fill=False, lw=W_THIN, color=INK))
            self.text(bx + a + 1.0, by + r * h + h - 0.7, lab, fs=5.0, role="tb_label", va="top")
            if val:
                self.text(bx + a + 1.0, by + r * h + 0.8, val, fs=8.5 if big else 6.6, role="tb_value", va="bottom",
                          weight="bold" if big else "normal")
        # first-angle symbol in its cell: truncated cone, its thick end next to the circles (ISO 5456-2)
        sx, sy = bx + 158, by + 3 * h + 0.6
        self.ax.add_patch(plt.Polygon([(sx, sy + 1.4), (sx + 6, sy), (sx + 6, sy + 4.4), (sx, sy + 3.0)], fill=False,
                                      lw=W_THIN))
        self.ax.add_patch(plt.Circle((sx + 11.5, sy + 2.2), 2.2, fill=False, lw=W_THIN))
        self.ax.add_patch(plt.Circle((sx + 11.5, sy + 2.2), 0.9, fill=False, lw=W_THIN))
        self.tb = cells


# ------------------------------------------------------------------ N3 · head frame
def n3_model():
    """The N3 solid moved to its datum O and every feature, all from the CAD code."""
    import body as B
    import hardware as hw
    import joint as J
    body = B.Body()
    j = body.joints["head_pitch"]
    O = np.array(j.origin, float)
    s = body.parts["head_frame"].moved(cq.Location(cq.Vector(*(-O))))
    ya = -J.CENTER                                      # y of the stack point a = 0 (the servo case top)
    hx0, hx1 = B.Body.HEAD_X[0] - O[0], B.Body.HEAD_X[1] - O[0]
    hy0, hy1 = B.Body.HEAD_Y
    hd = B.HEAD_DECK - O[2]
    f = dict(
        O=O, ya=ya,
        horn=dict(y_in=J.HP[0] + ya, y_out=J.HP[1] + ya, r=J.HORN_R, recess_d=hw.HORN.disc_d + 0.4,
                  recess_y=J.HORN_TOP + ya, hub_d=hw.HORN.hub_d + 0.8, access_d=hw.HOLES["access"].d,
                  cap_d=2 * (J.CAP_R + 0.05), cap_y=J.HP[1] - J.CAP_T + ya, pilot_r=hw.HORN.screw_r,
                  pilot_d=hw.HOLES["pilot_st2"].d, pilot_y0=J.HORN_TOP - 0.1 + ya, pilot_y1=J.HP[1] - 0.4 + ya),
        idler=dict(y_out=J.IP[0] + ya, y_in=J.IP[1] + ya, r=J.IDLER_R, cap_d=2 * (J.IDLER_R - 1.0),
                   cap_y=J.IP[0] + J.CAP_T + ya, seat_d=hw.HOLES["bearing"].d, seat_y1=J.BRG[1] + ya, lip_d=4.6),
        bar=dict(x=(B.NECK_X - 6.0 - O[0], B.NECK_X + 6.0 - O[0]), z=(B.HEAD_Z - O[2], hd)),
        deck=dict(x=(hx0, hx1), y=(hy0, hy1), z=(hd, hd + 3.0)),
        face=dict(x=(hx1 - 3.0, hx1), z=(hd - 26.0, hd)),
    )
    holes = [h for h in body.holes() if h.part == "head_frame"]
    feats = []
    groups = {"shell": ("A", "Tán cấy nhiệt M2 (OD 3.2 × 3) xuyên sàn — bắt vỏ đầu C3/C4, ốc M2×6 từ trên",
                        "C3/C4 tab Ø2.30"),
              "bay_cam": ("B", "Tán cấy nhiệt M2 xuyên sàn — bắt N4 (cam_mount), ốc M2×6 từ trên", "N4 lỗ Ø2.30"),
              "disp_tie": ("C", "Khe dây rút 1.6 × 3.0 xuyên tấm mặt — buộc module màn hình", "dây rút 2.5 mm"),
              "horn": ("D", "Lỗ mồi vít ST2×4, mù — bắt horn tròn SG90 vào đĩa", "horn SG90 (Ø lỗ: TBD)"),
              "access": ("E", "Lỗ thao tác Ø5.2 xuyên — tua vít PH0 / lục giác tới ốc tâm horn", "—"),
              "bearing": ("F", "Ổ bạc đạn MR63ZZ (3×6×2.5) + hốc đầu ốc M3×6 đầu dù + W3", "MR63ZZ, N2 tán M3")}
    order = ["shell", "bay_cam", "disp_tie", "horn", "access", "bearing"]
    for g in order:
        mem = sorted([h for h in holes if h.id.split(".")[-1].rstrip("0123456789") == g],
                     key=lambda h: (round(h.pos[0], 2), round(h.pos[1], 2), round(h.pos[2], 2)))
        tag, what, mate = groups[g]
        for i, h in enumerate(mem):
            feats.append(dict(tag=f"{tag}{i + 1}", group=tag, id=h.id, kind=h.kind, d=h.d, depth=h.depth,
                              dir=np.round(np.asarray(h.dir, float), 4), pos=np.asarray(h.pos, float) - O, what=what,
                              mate=mate))
    return s, f, feats, holes


def size_text(ft):
    if ft["kind"] == "tie":
        return "1.60 × 3.00 xuyên (THRU)"
    if ft["depth"]:
        return f"Ø{ft['d']:.2f} {DEPTH}{ft['depth']:.2f} (mù)"
    return f"Ø{ft['d']:.2f} xuyên (THRU)"


def dir_text(d):
    names = {(1, 0, 0): "+X", (-1, 0, 0): "−X", (0, 1, 0): "+Y", (0, -1, 0): "−Y", (0, 0, 1): "+Z", (0, 0, -1): "−Z"}
    return names.get(tuple(np.round(d, 3) + 0.0), str(tuple(np.round(d, 2))))


# fixed-fastener position tolerance (ASME/ISO practice): an insert mated to a clearance hole H with screw F leaves
# T = H - F to share between the two parts; each part gets half
def pos_tol(group):
    if group in ("A", "B"):
        return 2.30 - 2.00
    return None


def print_analysis(s, f):
    """Bed contact and overhang for the six axis-aligned orientations; overhang inside critical features."""
    out = {}
    rot = {"+Z lên (như CAD)": ((1, 0, 0), 0), "mặt trên sàn úp xuống (+Z xuống)": ((1, 0, 0), 180),
           "+X xuống": ((0, 1, 0), 90), "−X xuống": ((0, 1, 0), -90), "+Y xuống": ((1, 0, 0), -90),
           "−Y xuống": ((1, 0, 0), 90)}
    for lab, (ax_, ang) in rot.items():
        r = s.rotate((0, 0, 0), ax_, ang)
        v, fc = r.tessellate(0.03, 0.15)
        V = np.array([(p.x, p.y, p.z) for p in v])
        T = V[np.array(fc)]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        a = np.linalg.norm(n, axis=1) / 2
        n = n / (2 * a[:, None] + 1e-12)
        c = T.mean(1)
        zmin = V[:, 2].min()
        bed = (n[:, 2] < -0.99) & (np.abs(c[:, 2] - zmin) < 0.05)
        over = (n[:, 2] < -0.707) & ~bed
        out[lab] = dict(bed=float(a[bed].sum()), over=float(a[over].sum()), h=float(np.ptp(V[:, 2])))
    return out


def critical_overhang(s, f, lab="mặt trên sàn úp xuống (+Z xuống)", ang=180):
    """Overhang area (mm2) inside the horn recess, the bearing seat and the pilot holes when printed deck-down."""
    r = s.rotate((0, 0, 0), (1, 0, 0), ang)
    v, fc = r.tessellate(0.03, 0.15)
    V = np.array([(p.x, p.y, p.z) for p in v])
    T = V[np.array(fc)]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    a = np.linalg.norm(n, axis=1) / 2
    n = n / (2 * a[:, None] + 1e-12)
    c = T.mean(1)
    c = c * np.array([1, -1, -1])          # back to the part frame (rotation 180 deg about X)
    over = n[:, 2] < -0.707
    rr = np.hypot(c[:, 0], c[:, 2])
    h, i = f["horn"], f["idler"]
    zones = {"hốc horn Ø20.4 (D)": (c[:, 1] > h["y_in"] - 0.05) & (c[:, 1] < h["recess_y"] + 0.05)
             & (rr < h["recess_d"] / 2 + 0.05),
             "ổ bạc đạn Ø6.05 (F)": (c[:, 1] > i["cap_y"] - 0.05) & (c[:, 1] < i["seat_y1"] + 0.05)
             & (rr < i["seat_d"] / 2 + 0.05),
             "lỗ thao tác Ø5.2 (E)": (c[:, 1] > h["recess_y"]) & (c[:, 1] < h["y_out"]) & (rr < h["access_d"] / 2 + 0.05)}
    return {k: float(a[over & m].sum()) for k, m in zones.items()}


def draw_n3(out):
    import audit as A
    s, f, feats, holes = n3_model()
    h, i_, bar, deck, face = f["horn"], f["idler"], f["bar"], f["deck"], f["face"]
    zt, xf = deck["z"][1], face["x"][1]
    mass = A.printed_g(s, "arm")
    pa = print_analysis(s, f)
    crit = critical_overhang(s, f)
    tb = dict(owner="Hau Nguyen Ngoc", project="Bazan Humanoid — cụm đầu (H)", title="N3 · Head Frame",
              number="BZN-H-N3", material="PLA (xem ghi chú M1)", mass=f"{mass:.1f} g (ước tính)", scale="2:1 (C, D 5:1)",
              designed="Hau Nguyen Ngoc", checked="TBD", approved="TBD", date=date.today().isoformat(), rev="P1",
              status="SƠ BỘ — chưa phát hành chế tạo",
              standards="ISO 128 · ISO 129-1 · ISO 5455 · ISO 5456-2 · ISO 5457 · ISO 7200 · ISO 1101 (tham chiếu)")
    K, KD = 2.0, 5.0
    c3 = np.array([(deck["x"][0] + deck["x"][1]) / 2, 0.0, (-h["r"] + zt) / 2])
    fc, tc, lc = project(c3, "front"), project(c3, "top"), project(c3, "left")
    sheets = []

    # ================= sheet 1: principal views (first angle), datums, feature tags
    s1 = Sheet(1, "views")
    s1.place("front", "front", 98, 218, fc[0], fc[1], K)
    s1.place("top", "top", 98, 88, fc[0], tc[1], K)          # same paper x as the front view
    s1.place("left", "left", 228, 218, lc[0], fc[1], K)      # same paper y as the front view
    for nm in ("front", "top", "left"):
        vis, hid = hlr(s, nm)
        s1.draw_edges(nm, hid, w=W_THIN, ls=(0, (3, 1.5)))
        s1.draw_edges(nm, vis, w=W_THICK)
    s1.centerline("front", (0, i_["y_out"], 0), (0, h["y_out"], 0))
    s1.centerline("top", (0, i_["y_out"], 0), (0, h["y_out"], 0))
    s1.centermark("left", (0, 0, 0), h["r"])
    for ft in feats:
        if ft["group"] in ("A", "B"):
            s1.centermark("top", ft["pos"], ft["d"] / 2)
    # front: width, height, face plate height and the tie slots from datum A
    s1.dim("front", (0, deck["y"][0], zt), (0, deck["y"][1], zt), (0, 1, 0), "above")
    s1.dim("front", (xf, deck["y"][0], face["z"][0]), (xf, deck["y"][0], zt), (0, 0, 1), "left")
    s1.dim("front", (0, h["y_out"], -h["r"]), (xf, deck["y"][1], zt), (0, 0, 1), "right", on_surface=False)
    c_feats = [ft for ft in feats if ft["group"] == "C"]
    zc = sorted({round(ft["pos"][2], 3) for ft in c_feats})
    ymax_c = max(ft["pos"][1] for ft in c_feats)
    for zz in zc[::-1]:
        s1.dim("front", (xf, ymax_c, zz), (xf, ymax_c, zt), (0, 0, 1), "right", on_surface=False)
    s1.dim("front", (xf, -ymax_c, zc[0]), (xf, ymax_c, zc[0]), (0, 1, 0), "below", on_surface=False)
    # left: depth, axis position from datums A and B, bar width, deck thickness
    s1.dim("left", (deck["x"][0], 0, zt), (xf, 0, zt), (1, 0, 0), "above")
    s1.dim("left", (0, 0, 0), (xf, 0, 0), (1, 0, 0), "below", on_surface=False)
    s1.dim("left", (bar["x"][0], i_["y_out"], bar["z"][0] + 8), (bar["x"][1], i_["y_out"], bar["z"][0] + 8),
           (1, 0, 0), "below")
    s1.dim("left", (0, 0, 0), (0, 0, zt), (0, 0, 1), "right", on_surface=False)
    s1.dim("left", (deck["x"][0], 0, deck["z"][0]), (deck["x"][0], 0, zt), (0, 0, 1), "left")
    # top: deck outline, hole pattern from datum B (front face) and about datum C (Y = 0)
    a_pts = [ft for ft in feats if ft["group"] == "A"]
    b_pts = [ft for ft in feats if ft["group"] == "B"]
    xs_a = sorted({round(ft["pos"][0], 3) for ft in a_pts})
    ya = max(ft["pos"][1] for ft in a_pts)
    yb = max(ft["pos"][1] for ft in b_pts)
    xb = b_pts[0]["pos"][0]
    s1.dim("top", (xs_a[0], -ya, zt), (xs_a[0], ya, zt), (0, 1, 0), "above", on_surface=False, tpos=0.3)
    s1.dim("top", (xb, -yb, zt), (xb, yb, zt), (0, 1, 0), "above", on_surface=False, tpos=0.3)
    s1.dim("top", (deck["x"][0], deck["y"][0], zt), (deck["x"][1], deck["y"][0], zt), (1, 0, 0), "left")
    for xv in [xb] + xs_a[::-1]:
        s1.dim("top", (xf, ya, zt), (xv, ya, zt), (1, 0, 0), "right", on_surface=False)
    # cutting planes on the top view
    k_ = s1.views["top"][4]
    u_ = s1.used["top"]
    plan = (((0, deck["y"][0] - (u_["left"] + 4) / k_, zt), (0, deck["y"][1] + (u_["right"] + 4) / k_, zt),
             (-1, 0, 0), "A"),
            ((deck["x"][1] + (u_["below"] + 4) / k_, 0, zt), (deck["x"][0] - (u_["above"] + 4) / k_, 0, zt),
             (0, 1, 0), "B"))
    for p0, p1, arrow_dir, lab in plan:
        P0, P1 = s1.P("top", p0), s1.P("top", p1)
        s1.line([P0[0], P1[0]], [P0[1], P1[1]], ls=(0, (8, 1.5, 1, 1.5)))
        dd = s1.P("top", np.array(p0) + np.array(arrow_dir) * 5) - P0
        dd = dd / np.linalg.norm(dd)
        for P_ in (P0, P1):
            s1.line([P_[0], P_[0] + dd[0] * 6], [P_[1], P_[1] + dd[1] * 6], w=W_THICK)
            s1.arrow(P_ + dd * 6, P_)
            s1.text(P_[0] + dd[0] * 8.5, P_[1] + dd[1] * 8.5, lab, fs=10, role="section", ha="center",
                    va="center", weight="bold")
    # feature tags: A, B on the top view; C on the front view; D, E, F on the left view
    for nm in ("top", "front", "left"):
        x0, y0, x1, y1 = s1.bbox[nm]
        s1.boxes.append((x0 - 1, y0 - 1, x1 + 1, y1 + 1)) if False else None
    view_of = {"A": "top", "B": "top", "C": "front", "D": "left", "E": "left", "F": "left"}
    for ft in feats:
        s1.tag(view_of[ft["group"]], ft["pos"], ft["tag"])
    for nm, t, sc in (("front", "Hình chiếu đứng (nhìn từ +X)", "2:1"), ("left", "Hình chiếu cạnh (nhìn từ −Y)", "2:1"),
                      ("top", "Hình chiếu bằng (nhìn từ +Z)", "2:1")):
        s1.caption(nm, t, sc)
    # iso
    iv, _ = hlr(s, "iso")
    ic = project(c3, "iso")
    s1.place("iso", "iso", 236, 103, ic[0], ic[1], 1.5)
    s1.draw_edges("iso", iv, w=W_THICK * 0.8)
    s1.caption("iso", "Hình trục đo", "minh hoạ, không đo")
    # legend (no leaders) and datum definition
    xl = 302
    s1.text(xl, 281, "Ký hiệu đặc tính", fs=9, role="head", weight="bold", va="top")
    yy = 274
    for g, what in (("A", "tán M2 bắt vỏ C3/C4"), ("B", "tán M2 bắt N4"), ("C", "khe dây rút màn hình"),
                    ("D", "lỗ mồi ST2 (horn)"), ("E", "lỗ thao tác ốc tâm horn"), ("F", "ổ bạc đạn MR63ZZ")):
        mem = [ft for ft in feats if ft["group"] == g]
        s1.text(xl, yy, f"{g}1–{g}{len(mem)}" if len(mem) > 1 else f"{g}1", fs=7.5, role="legend", va="top",
                weight="bold")
        s1.text(xl + 14, yy, f"{len(mem)}× {size_text(mem[0])}", fs=7.5, role="legend", va="top")
        s1.text(xl + 14, yy - 3.6, what, fs=6.8, role="legend", va="top")
        yy -= 8.6
    s1.text(xl, yy - 2, "Gốc toạ độ và chuẩn đo", fs=9, role="head", weight="bold", va="top")
    s1.text(xl, yy - 8, "O = trục gật đầu ∩ mặt phẳng giữa chồng khớp\n(tư thế 0). X tới trước, Y sang trái, Z lên.\n"
            f"A = mặt trên sàn đầu (Z = +{zt:.2f})\nB = mặt trước tấm mặt (X = +{xf:.2f})\n"
            "C = mặt phẳng đối xứng Y = 0\nToạ độ đầy đủ mọi đặc tính: tờ 4.\nMặt cắt A–A, B–B: tờ 2. Chi tiết C, D: tờ 3.",
            fs=7.2, role="note", va="top")
    sheets.append(s1)

    # ================= sheet 2: sections A-A, B-B and details C, D
    s2 = Sheet(2, "sections")
    av, apol, _ = section(s, (0, 0, 0), "front")
    bv, bpol, _ = section(s, (0, 0, 0), "left")
    s2.place("A", "front", 98, 205, fc[0], fc[1], K)
    s2.place("B", "left", 242, 205, lc[0], fc[1], K)
    s2.draw_edges("A", av)
    s2.hatch("A", apol)
    s2.draw_edges("B", bv)
    s2.hatch("B", bpol)
    s2.centerline("A", (0, i_["y_out"], 0), (0, h["y_out"], 0))
    s2.centermark("B", (0, 0, 0), h["r"])
    s2.dim("A", (0, i_["y_out"], -7.2), (0, h["y_out"], -8.0), (0, 1, 0), "above")
    s2.dim("A", (0, i_["y_out"], -i_["r"]), (0, i_["y_in"], -i_["r"]), (0, 1, 0), "below")
    s2.dim("A", (0, h["y_in"], -h["r"]), (0, h["y_out"], -h["r"]), (0, 1, 0), "below")
    s2.dim("A", (0, i_["y_in"], -5.0), (0, h["y_in"], -11.0), (0, 1, 0), "below")
    s2.dim("A", (0, h["y_out"], -h["r"]), (0, h["y_out"], h["r"]), (0, 0, 1), "right", prefix="Ø")
    s2.dim("A", (0, i_["y_out"], -i_["r"]), (0, i_["y_out"], i_["r"]), (0, 0, 1), "left", prefix="Ø")
    s2.dim("A", (0, deck["y"][0], deck["z"][0]), (0, deck["y"][0], zt), (0, 0, 1), "left")
    s2.dim("B", (face["x"][0], 0, zt - 10), (xf, 0, zt - 10), (1, 0, 0), "below")
    s2.dim("B", (xf, 0, face["z"][0]), (xf, 0, zt), (0, 0, 1), "right")
    s2.dim("B", (deck["x"][0], 0, deck["z"][0]), (deck["x"][0], 0, zt), (0, 0, 1), "left")
    # detail windows (on A-A) and the details themselves at 4:1
    wins = {"C": (h["y_in"] - 1.0, -h["r"] - 1.0, h["y_out"] + 1.0, h["r"] + 1.0),
            "D": (i_["y_out"] - 1.0, -i_["r"] - 1.0, i_["y_in"] + 1.0, i_["r"] + 1.0)}
    for nm, (u0, v0, u1, v1) in wins.items():
        q0, q1 = s2.P2("A", (u0, v0)), s2.P2("A", (u1, v1))
        s2.ax.add_patch(plt.Rectangle(q0, *(q1 - q0), fill=False, lw=W_THIN, ls=(0, (6, 2))))
        s2.text(q1[0] + 1, q1[1] + 1, nm, fs=10, role="section", weight="bold")
    s2.text(300, 281, "Ghi chú mặt cắt", fs=9, role="head", weight="bold", va="top")
    s2.text(300, 274, "Mặt cắt không vẽ nét khuất; gạch 45°, nét mảnh.\nØ trên hình cắt: đường kính quanh trục gật\n"
            "đầu (trục Y).\nKhung nét đứt C, D: vùng phóng to ở tờ 3 (5:1).\n\n"
            f"Khe giữa hai đĩa {fmt(h['y_in'] - i_['y_in'])}: chứa servo SG90 + stack N2\n— kích thước lắp ghép (K2).",
            fs=7.2, role="note", va="top")
    for nm, t in (("A", "Mặt cắt A–A (X = 0, qua trục gật đầu)"), ("B", "Mặt cắt B–B (Y = 0, mặt phẳng giữa)")):
        s2.caption(nm, t, "2:1")
    sheets.append(s2)

    # ================= sheet 3: details C and D at 5:1 (ISO 5455)
    s2 = Sheet(3, "details")
    s2.place("C", "front", 95, 145, (wins["C"][0] + wins["C"][2]) / 2, 0.0, KD)
    s2.place("D", "front", 250, 160, (wins["D"][0] + wins["D"][2]) / 2, 0.0, KD)
    for nm in ("C", "D"):
        segs, pols = clip(av, apol, wins[nm])
        s2.draw_edges(nm, segs)
        s2.hatch(nm, pols)
        s2.centerline(nm, (0, wins[nm][0], 0), (0, wins[nm][2], 0), ext=1.0)
    # detail C: horn plate
    s2.dim("C", (0, h["y_in"], h["r"]), (0, h["y_out"], h["r"]), (0, 1, 0), "above")
    s2.dim("C", (0, h["y_in"], h["recess_d"] / 2), (0, h["recess_y"], h["recess_d"] / 2), (0, 1, 0), "above",
           text=f"{DEPTH}{fmt(h['recess_y'] - h['y_in'])}")
    s2.dim("C", (0, h["cap_y"], -h["cap_d"] / 2), (0, h["y_out"], -h["cap_d"] / 2), (0, 1, 0), "below",
           text=f"{DEPTH}{fmt(h['y_out'] - h['cap_y'])}")
    s2.dim("C", (0, h["y_in"], -h["recess_d"] / 2), (0, h["y_in"], h["recess_d"] / 2), (0, 0, 1), "left", prefix="Ø")
    s2.dim("C", (0, h["y_out"], -h["cap_d"] / 2), (0, h["y_out"], h["cap_d"] / 2), (0, 0, 1), "right", prefix="Ø")
    s2.dim("C", (0, h["y_out"] - 0.5, -h["access_d"] / 2), (0, h["y_out"] - 0.5, h["access_d"] / 2), (0, 0, 1),
           "right", prefix="Ø", on_surface=False)
    s2.dim("C", (0, h["pilot_y1"], -h["pilot_r"]), (0, h["pilot_y1"], h["pilot_r"]), (0, 0, 1), "right",
           on_surface=False)
    s2.leader_note = None
    pc = s2.P("C", (0, h["pilot_y1"], h["pilot_r"]))
    tx = np.array([pc[0] + 30, pc[1] + 30])
    s2.line([pc[0], tx[0]], [pc[1], tx[1]])
    s2.line([tx[0], tx[0] + 40], [tx[1], tx[1]])
    s2.text(tx[0] + 1, tx[1] + 0.8, f"D1, D2: 2× Ø{h['pilot_d']:.2f} {DEPTH}{h['pilot_y1'] - h['recess_y']:.2f} "
            f"từ đáy hốc", fs=7.2, role="note", va="bottom")
    s2.text(tx[0] + 1, tx[1] - 0.8, f"vỏ còn lại tới mặt ngoài: {h['y_out'] - h['pilot_y1']:.2f} (thành mỏng)", fs=7.2,
            role="note", va="top")
    # detail D: idler plate
    s2.dim("D", (0, i_["y_out"], i_["r"]), (0, i_["y_in"], i_["r"]), (0, 1, 0), "above")
    s2.dim("D", (0, i_["cap_y"], i_["seat_d"] / 2), (0, i_["seat_y1"], i_["seat_d"] / 2), (0, 1, 0), "above",
           text=f"{DEPTH}{fmt(i_['seat_y1'] - i_['cap_y'])}")
    s2.dim("D", (0, i_["y_out"], -i_["cap_d"] / 2), (0, i_["cap_y"], -i_["cap_d"] / 2), (0, 1, 0), "below",
           text=f"{DEPTH}{fmt(i_['cap_y'] - i_['y_out'])}")
    s2.dim("D", (0, i_["seat_y1"], -i_["lip_d"] / 2), (0, i_["y_in"], -i_["lip_d"] / 2), (0, 1, 0), "below",
           text=f"{DEPTH}{fmt(i_['y_in'] - i_['seat_y1'])}")
    s2.dim("D", (0, i_["cap_y"], -i_["seat_d"] / 2), (0, i_["cap_y"], i_["seat_d"] / 2), (0, 0, 1), "left",
           prefix="Ø", on_surface=False)
    s2.dim("D", (0, i_["y_out"], -i_["cap_d"] / 2), (0, i_["y_out"], i_["cap_d"] / 2), (0, 0, 1), "left", prefix="Ø")
    s2.dim("D", (0, i_["y_in"], -i_["lip_d"] / 2), (0, i_["y_in"], i_["lip_d"] / 2), (0, 0, 1), "right", prefix="Ø",
           on_surface=False)
    s2.dim("D", (0, i_["y_in"], -i_["r"]), (0, i_["y_in"], i_["r"]), (0, 0, 1), "right", prefix="Ø")
    for nm, t in (("C", "Chi tiết C — đĩa horn (từ mặt cắt A–A, tờ 2)"),
                  ("D", "Chi tiết D — đĩa idler (từ mặt cắt A–A, tờ 2)")):
        s2.caption(nm, t, "5:1")
    s2.text(300, 281, "Ghi chú chi tiết", fs=9, role="head", weight="bold", va="top")
    s2.text(300, 274, "Kích thước Ø: quanh trục gật đầu (trục Y).\n↧ = chiều sâu đo dọc trục Y.\n\n"
            "Vùng thành mỏng (kiểm tra khi in):\n"
            f"• đĩa horn, vỏ ngoài lỗ D: {h['y_out'] - h['pilot_y1']:.2f}\n"
            f"• đĩa horn, thành quanh hốc Ø{fmt(h['recess_d'])}: {h['r'] - h['recess_d'] / 2:.2f}\n"
            f"• đĩa idler, thành quanh hốc Ø{fmt(i_['cap_d'])}: {i_['r'] - i_['cap_d'] / 2:.2f}\n\n"
            "Mặt lắp ghép quan trọng:\n• hốc horn Ø20.4: horn SG90 nằm phẳng\n• ổ Ø6.05: ép MR63ZZ (K1)\n"
            "• đồng trục hốc horn và ổ F (K3)", fs=7.2, role="note", va="top")
    sheets.append(s2)

    # ================= sheet 4: feature table and notes
    import textwrap
    s3 = Sheet(4, "table")
    s3.text(25, 281, "BẢNG TOẠ ĐỘ LỖ / ĐẶC TÍNH — N3 Head Frame", fs=FS_TITLE, role="head", weight="bold", va="top")
    s3.text(25, 274, "Toạ độ X, Y, Z từ gốc O (trục gật đầu ∩ mặt phẳng giữa chồng khớp, tư thế 0); X tới trước, Y "
            "sang trái, Z lên. ΔA, ΔB, ΔC: khoảng cách tới chuẩn đo A (mặt trên sàn),\nB (mặt trước tấm mặt), C (Y = 0). "
            "Mã A1…F1 là mã ghi trên tờ 1. Hướng = chiều mũi khoan / chiều ốc đi vào. Dung sai vị trí chỉ ghi khi có "
            "ràng buộc lắp ghép (ghi chú K4).", fs=7.2, role="note", va="top")
    cols = [("Mã", 11), ("Đặc tính", 104), ("Kích thước", 40), ("Hướng", 13), ("X", 15), ("Y", 15), ("Z", 15),
            ("ΔA", 14), ("ΔB", 14), ("ΔC", 14), ("Dung sai vị trí", 37), ("Lắp với", 38), ("ID CAD", 24)]
    x0, y0, rh = 25.0, 262.0, 6.0
    xs = np.cumsum([0] + [w for _, w in cols]) + x0
    rows = []
    for ft in feats:
        p = ft["pos"]
        t = pos_tol(ft["group"])
        rows.append([ft["tag"], ft["what"], size_text(ft), dir_text(ft["dir"]), fmt2(p[0]), fmt2(p[1]), fmt2(p[2]),
                     fmt2(zt - p[2]), fmt2(xf - p[0]), fmt2(p[1]),
                     f"vị trí Ø{t / 2:.2f} | A | B | C" if t else "—", ft["mate"], ft["id"].split(".", 1)[-1]])
    for r, row in enumerate([[c for c, _ in cols]] + rows):
        y = y0 - r * rh
        for c, val in enumerate(row):
            s3.ax.add_patch(plt.Rectangle((xs[c], y - rh), xs[c + 1] - xs[c], rh, fill=r == 0,
                                          facecolor="#e8e8e8", edgecolor=INK, lw=W_THIN))
            num = c >= 4 and c <= 9 and r > 0
            s3.text(xs[c + 1] - 1.0 if num else xs[c] + 1.0, y - rh / 2, str(val), fs=6.5 if c == 1 else 7.0,
                    role="table", va="center", ha="right" if num else "left", weight="bold" if r == 0 else "normal")
    yy = y0 - (len(rows) + 1) * rh - 6
    best = max(pa, key=lambda k: pa[k]["bed"] - 0.5 * pa[k]["over"])
    notes = [
        ("VẬT LIỆU", [
            "M1. PLA. Tiêu chí đổi sang PETG: nhiệt độ vỏ servo head pitch (SG90) đo được > 45 °C khi chạy liên tục "
            "(PLA mềm ở khoảng 55–60 °C). Chưa có số đo → giữ PLA."]),
        ("IN 3D (FDM)", [
            f"P1. Hướng in: {best} — tiếp xúc bàn {pa[best]['bed']:.0f} mm², mặt treo >45° {pa[best]['over']:.0f} mm²,"
            f" cao {pa[best]['h']:.1f} mm (tính từ CAD trên 6 hướng trục; bảng đủ trong báo cáo kiểm tra).",
            "P2. Lớp in song song mặt A: sàn và tấm mặt chịu kéo dọc lớp. Không in theo ±Y: chân hai cột đỡ đĩa sẽ chịu "
            "kéo vuông góc lớp.",
            "P3. Support cần dưới phần đĩa nhô ra ngoài cột. CẤM support bên trong: "
            + "; ".join(f"{k} ({v:.0f} mm² mặt treo)" for k, v in crit.items())
            + " — đặt support blocker; chỗ võng sửa bằng dao/mũi doa.",
            "P4. Lỗ D, E, F có trục nằm ngang khi in: đỉnh lỗ có thể võng; kiểm tra độ tròn ổ F bằng chính bạc đạn.",
            "P5. Thông số máy in và dung sai đạt được: TBD — xác định bằng phiếu thử T0 (docs/mechanical/test_print.md)."]),
        ("LẮP RÁP", [
            "L1. A1–A4, B1–B2: tán cấy nhiệt M2 ép từ mặt A ở 220 °C, bằng mặt A. Sàn dày 3 = chiều dài tán → kiểm tra "
            "mặt dưới không phồng.",
            "L2. D1–D2: horn tròn SG90 nằm phẳng đáy hốc Ø20.4, 2 vít ST2×4 từ phía servo xuyên horn vào lỗ mồi. Ø lỗ "
            "trên horn: TBD (đo horn thật).",
            "L3. F1: ép MR63ZZ; ốc đầu dù M3×6 + long đền W3 vào tán M3 trên N2. E1: lối tua vít tới ốc tâm horn.",
            "L4. Tháo: vỏ C3/C4 (4 ốc qua đỉnh đầu) → N4 → ốc idler → nhấc N3 khỏi horn."]),
        ("KIỂM TRA", [
            "K1. Ổ F: MR63ZZ ép tay vào được, lật ngược không rơi, quay êm.",
            "K2. Khe trong giữa hai đĩa 31.9 (tờ 2): servo SG90 + stack N2 lọt, không kẹt, không rơ dọc trục.",
            "K3. Đồng trục hốc horn (D) và ổ F quanh trục gật đầu: bắt buộc về chức năng; giá trị dung sai TBD.",
            "K4. Dung sai vị trí A, B theo công thức ốc cố định: lỗ đối tiếp Ø2.30, ốc M2 Ø2.00 → T = 0.30 chia đều hai "
            "chi tiết → Ø0.15 cho mỗi chi tiết, so với A | B | C."]),
    ]
    for head, items in notes:
        s3.text(25, yy, head, fs=8.5, role="head", weight="bold", va="top")
        yy -= 5.0
        for it in items:
            wrapped = textwrap.wrap(it, 175)
            s3.text(28, yy, "\n".join(wrapped), fs=7.0, role="note", va="top", linespacing=1.35)
            yy -= 3.4 * len(wrapped) + 1.4
        yy -= 1.5
    sheets.append(s3)
    total = len(sheets)
    for sh in sheets:
        sh.furniture(tb, total)
    return dict(sheets=sheets, solid=s, feats=feats, holes=holes, f=f, tb=tb, print=pa, crit=crit, scale=K)


def fmt2(v):
    t = f"{v:.2f}"
    return "0.00" if t == "-0.00" else t


# ------------------------------------------------------------------ validation (phase 8)
def validate(res):
    s, feats, holes, sheets = res["solid"], res["feats"], res["holes"], res["sheets"]
    rep, bad = [], 0

    def item(name, ok, detail):
        nonlocal bad
        bad += not ok
        rep.append(dict(check=name, ok=bool(ok), detail=detail))

    # 1. every dimension against the solid
    for sh in sheets:
        for d in sh.dims:
            p0, p1 = np.array(d.p0), np.array(d.p1)
            measured = abs(np.dot(p1 - p0, d.axis))
            shown = float("".join(ch for ch in d.text.split(" ")[0] if ch in "0123456789."))
            ok = abs(measured - shown) < 0.006
            dist = []
            if d.on_surface:
                for p in (p0, p1):
                    dist.append(cq.Vertex.makeVertex(*p).distance(s))
                ok = ok and max(dist) < 0.02
            item(f"kích thước tờ {sh.number} [{d.view}] {d.text}", ok,
                 f"đo trên CAD {measured:.3f}, ghi {shown}" + (f", điểm cách mặt {max(dist):.3f}" if dist else
                                                               " (điểm chuẩn/trục, không nằm trên mặt)"))
    # 2. feature table = CAD holes
    ids_t = sorted(ft["id"] for ft in feats)
    ids_c = sorted(h.id for h in holes)
    item("mọi lỗ CAD có trong bảng, không thừa", ids_t == ids_c, f"{len(ids_t)} trong bảng, {len(ids_c)} trong CAD")
    O = res["f"]["O"]
    worst = max(np.linalg.norm(ft["pos"] - (np.asarray(h.pos) - O))
                for ft in feats for h in holes if h.id == ft["id"])
    item("toạ độ bảng = toạ độ CAD", worst < 1e-6, f"lệch lớn nhất {worst:.2e} mm")
    tags = [ft["tag"] for ft in feats]
    item("mã đặc tính không trùng", len(tags) == len(set(tags)), ", ".join(tags))
    # 3. view alignment (first angle: front/top share paper x, front/left share paper y)
    s1 = sheets[0]
    a = s1.P("front", (0, 0, 0))
    b = s1.P("top", (0, 0, 0))
    c = s1.P("left", (0, 0, 0))
    item("thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm)", abs(a[0] - b[0]) < 1e-6, f"Δ = {a[0] - b[0]:.2e} mm")
    item("thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm)", abs(a[1] - c[1]) < 1e-6, f"Δ = {a[1] - c[1]:.2e} mm")
    item("bằng nằm DƯỚI đứng, cạnh nằm PHẢI đứng (góc thứ nhất)", b[1] < a[1] and c[0] > a[0], "")
    # 4. scale: paper length / model length
    for sh in sheets:
        for nm, (cx, cy, uc, vc, k, mv) in sh.views.items():
            p, q = sh.P(nm, (0, 0, 0)), sh.P(nm, (0, 10, 0) if mv != "left" else (10, 0, 0))
            r = np.linalg.norm(q - p) / 10
            ok = mv == "iso" or abs(r - k) < 1e-9
            item(f"tỉ lệ tờ {sh.number} hình {nm}", ok, f"{r:g}:1 (khai báo {k:g}:1)")
    # 5. text collisions, clipping and page bounds
    for sh in sheets:
        sh.fig.canvas.draw()
        rnd = sh.fig.canvas.get_renderer()
        boxes = []
        for t, role in sh.texts:
            if not t.get_text().strip():
                continue
            bb = t.get_window_extent(rnd).transformed(sh.ax.transData.inverted())
            boxes.append((bb, t.get_text()[:30], role))
        inside = [(txt, role) for bb, txt, role in boxes if role != "zone" and not (
            bb.x0 >= BORDER[0] - 0.01 and bb.x1 <= BORDER[2] + 0.01 and bb.y0 >= BORDER[1] - 0.01
            and bb.y1 <= BORDER[3] + 0.01)]
        item(f"tờ {sh.number}: chữ nằm trong khung", not inside, "; ".join(f"{t}" for t, _ in inside[:6]) or "ok")
        hits = []
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                b1, b2 = boxes[i][0], boxes[j][0]
                if b1.x0 < b2.x1 - 0.2 and b2.x0 < b1.x1 - 0.2 and b1.y0 < b2.y1 - 0.2 and b2.y0 < b1.y1 - 0.2:
                    hits.append(f"'{boxes[i][1]}' × '{boxes[j][1]}'")
        item(f"tờ {sh.number}: chữ không chồng nhau", not hits, "; ".join(hits[:8]) or "ok")
    # 6. title block and sheet numbering
    tb = res["tb"]
    empty = [k for k, v in tb.items() if not str(v).strip()]
    item("khung tên đủ trường (TBD được phép, rỗng thì không)", not empty, ", ".join(empty) or "đủ")
    tbd = [k for k, v in tb.items() if "TBD" in str(v)]
    rep.append(dict(check="trường còn TBD (cần người duyệt)", ok=True, detail=", ".join(tbd)))
    nums = [sh.number for sh in sheets]
    item("đánh số tờ liên tục", nums == list(range(1, len(sheets) + 1)), f"{nums}")
    # 7. references: section/detail labels exist on the parent and as views
    item("mặt cắt A–A, B–B và chi tiết C, D có hình tương ứng",
         all(k in sheets[1].views for k in "AB") and all(k in sheets[2].views for k in "CD"), "")
    return rep, bad


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "N3"
    if which != "N3":
        raise SystemExit("drawing engine v2: only N3 so far")
    out = DOCS / "N3"
    out.mkdir(parents=True, exist_ok=True)
    res = draw_n3(out)
    rep, bad = validate(res)
    with PdfPages(out / "BZN-H-N3.pdf") as pdf:
        for sh in res["sheets"]:
            pdf.savefig(sh.fig)
            sh.fig.savefig(out / f"BZN-H-N3_sheet{sh.number}.png", dpi=110)
    lines = ["# BZN-H-N3 · kiểm tra bản vẽ tự động", "", f"Sinh bởi `hardware/mechanical/drawings.py N3`, "
             f"{date.today().isoformat()}. Lỗi: **{bad}**.", "", "| Kiểm tra | Kết quả | Chi tiết |", "| --- | --- | --- |"]
    lines += [f"| {r['check']} | {'đạt' if r['ok'] else '**LỖI**'} | {r['detail']} |" for r in rep]
    lines += ["", "## Hướng in (6 hướng trục, tính từ CAD)", "", "| Hướng | Tiếp xúc bàn mm² | Mặt treo >45° mm² | Cao mm |",
              "| --- | ---: | ---: | ---: |"]
    lines += [f"| {k} | {v['bed']:.0f} | {v['over']:.0f} | {v['h']:.1f} |" for k, v in res["print"].items()]
    (out / "BZN-H-N3_validation.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    json.dump(rep, open(out / "BZN-H-N3_validation.json", "w"), ensure_ascii=False, indent=1)
    print(f"N3: {len(res['sheets'])} sheets, {sum(len(s.dims) for s in res['sheets'])} dimensions, "
          f"{len(res['feats'])} features, validation defects {bad}")
    for r in rep:
        if not r["ok"]:
            print("  LỖI:", r["check"], "—", r["detail"])
    return 1 if (bad and "--check" in sys.argv) else 0


if __name__ == "__main__":
    sys.exit(main())
