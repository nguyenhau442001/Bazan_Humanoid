"""Bazan (BZN-01) shell v1: the printed skin over the body v4 frame, fastened only at the 16 shell inserts.

    python shell.py              # build, check against the frame over the arm/neck/head sweeps, write out/shell_*
    python shell.py --quick      # fewer sweep poses
    python shell.py --no-export  # check only

Parts (FDM, 1.2 mm walls = 3 perimeters of a 0.4 nozzle, 0.4 mm between mating edges):

    chest_shell  torso front, clamshell half x > T_SPLIT: 2 tabs on deck 2 + 2 under the pelvis plate, flag badge
    back_shell   torso back: 4 + 4 tabs, vents; the chest's seam tongue slides inside it
    head_front   helmet front: 2 tabs on the head deck (screwed through guide tubes from the crown), ears, visor window
    head_back    helmet back: 2 tabs, neck notch so the head pitches over the neck servos
    visor        clear PETG, glued from inside on its 2.5 mm lip; open windows for the camera and the ToF

Torso tabs: M2x6 from above through the top tabs into deck 2, and from below through the bottom tabs into the
pelvis plate. The torso stops at deck 2 (shoulder and neck mounts stay outside) and 8 mm below the pelvis plate, and
never reaches |y| = 63 where the forearm hangs. Head tabs: M2x6 dropped down the tube from the crown hole.
"""
import itertools
import sys
from pathlib import Path

import cadquery as cq
import numpy as np

import body as B
import electronics as el
import leg as LEG
import robot as R

OUT = Path(__file__).resolve().parent / "out"
WALL = 1.2
GAP = 0.4
CLR_M2 = 1.2                  # radius of the M2 clearance hole
TAB_R = 3.5
BIG = 500.0

# torso: outer skin, inner = outer - WALL
T_X = (B.DX[0] - 4.0 - WALL, B.DX[1] + 4.0 + WALL)      # 4 mm behind / in front of the decks
T_Y = (B.DY[0] - 3.0 - WALL, B.DY[1] + 3.0 + WALL)      # 3 mm beside them: outer |y| 58.2, forearm at 63
T_RV, T_RH = 11.2, 4.0                                  # outer fillets, vertical and top edges
T_TOP = B.Z2T + 2.0                                     # top tabs 2.0 thick on deck 2
T_BOT = B.Z0 - 8.0                                      # skirt over the hip mounts
T_LIP = 2.4                                             # bottom tabs under the pelvis plate (3.6 into the insert)
T_RING = 3.0
T_SPLIT = -4.0
BOT_INSERTS = [(26.0, 48.0), (26.0, -48.0), (-30.0, 48.0), (-30.0, -48.0), (-46.0, 25.0), (-46.0, -25.0)]
TOP_INSERTS = [(26.0, 51.0), (26.0, -51.0), (-32.0, 51.0), (-32.0, -51.0), (-46.0, 25.0), (-46.0, -25.0)]
BADGE_Z = 138.0               # flag badge: 24 x 16 pocket, 0.6 deep, in a frame 1.0 proud

# head (zero pose, moves with head_frame)
H_X = (-34.0, 45.2)           # back: the head deck corners (-27, ±24) clear the 14.8 inner corner fillet
H_Y = (-29.2, 29.2)
H_BOT, H_TOP = 199.0, 270.0
H_RV, H_RT, H_RB = 16.0, 14.0, 6.0
H_SPLIT = -12.0
HD3 = B.HEAD_DECK + 3.0
HEAD_INSERTS = [(-23.0, 20.0), (-23.0, -20.0), (2.0, 20.0), (2.0, -20.0)]
TUBE_R = (2.2, 3.4)           # guide tube: an M2 head (3.8) passes, 1.2 wall
NECK_R, NECK_Y = 30.0, 20.0   # notch around the pitch axis: the neck parts stay inside this radius
EAR = (8.0, 246.0, 8.0)       # x, z, radius
CHIN = 28.5                   # raked chin: at head pitch +32 its face is level CHIN below the pitch axis (z 179),
#                               above the shoulder pockets when the head is also turned 90°
WINDOW = ((-22.0, 22.0), (205.0, 260.0), 8.0)
LIP = 2.5
CAM_HOLE, TOF_HOLE = 6.5, 5.0
TILT = np.radians(15.0)
N = np.array([np.cos(TILT), 0.0, -np.sin(TILT)])       # camera and ToF look along N
UP = np.array([np.sin(TILT), 0.0, np.cos(TILT)])
FACE0 = np.array([22.0, 0.0, HD3]) + 3.0 * N            # cam_mount face (body._cam_mount)
CAM_C, TOF_C = FACE0 + 24.0 * UP, FACE0 + 7.0 * UP

bx, cyl = LEG.bx, B.cyl
# empty space inside the torso: within the top/bottom rings, between the lips, inside the corner fillets
T_FREE = ((T_X[0] + WALL + T_RING, T_X[1] - WALL - T_RING), (T_Y[0] + WALL + T_RING, T_Y[1] - WALL - T_RING),
          (B.Z0, B.Z2T))


def inside(bb, box, eps=1e-6):
    (x0, x1), (y0, y1), (z0, z1) = box
    return (bb.xmin >= x0 - eps and bb.xmax <= x1 + eps and bb.ymin >= y0 - eps and bb.ymax <= y1 + eps
            and bb.zmin >= z0 - eps and bb.zmax <= z1 + eps)


def rbox(x, y, z, rv, rt, rb=0.0, t=0.0):
    """Box inset by t with vertical edges filleted rv - t, top edges rt - t, bottom edges rb - t (if rb)."""
    w = cq.Workplane().add(bx((x[0] + t, x[1] - t), (y[0] + t, y[1] - t), (z[0] + t, z[1] - t)))
    w = w.edges("|Z").fillet(rv - t).faces(">Z").edges().fillet(rt - t)
    if rb:
        w = w.faces("<Z").edges().fillet(rb - t)
    return w.val()


def prism(x, y, z, r, axis="Z"):
    return cq.Workplane().add(bx(x, y, z)).edges(f"|{axis}").fillet(r).val()


def slab(x=(-BIG, BIG), y=(-BIG, BIG), z=(-BIG, BIG)):
    return bx(x, y, z)


def tab(x, y, z0, z1):
    """Disc on the insert plus an arm out to the nearest wall (clip with the inner skin)."""
    if abs(y) >= 18.0:
        arm = bx((x - TAB_R, x + TAB_R), (y, y + 40.0) if y > 0 else (y - 40.0, y), (z0, z1))
    else:
        arm = bx((x - 40.0, x), (y - TAB_R, y + TAB_R), (z0, z1))
    return cyl(TAB_R, (x, y, z0), (x, y, z1)).fuse(arm)


def seam_tongue(skin, split, keep):
    """Band inside the seam, owned by the +x half: 1.0 thick, GAP clear of the -x half's inner skin."""
    tongue = skin(WALL + GAP).cut(skin(WALL + GAP + 1.0)).intersect(slab(x=(split - 5.0, split + GAP / 2)))
    bridge = skin(WALL).cut(skin(WALL + GAP + 1.0)).intersect(slab(x=(split + GAP / 2, split + 3.0)))
    return tongue.fuse(bridge).intersect(keep)


class BazanShell:
    def __init__(self, body=None, leg=None):
        self.body = body or B.Body()
        self.leg = leg or LEG.Leg()
        self.parts = {}
        self.screws = []      # (part, point, direction the screw goes in)
        self._torso()
        self._head()

    # ------------------------------------------------------------------ torso
    def _torso(self):
        def skin(t=0.0):
            return rbox(T_X, T_Y, (T_BOT - 30.0, T_TOP), T_RV, T_RH, t=t)

        inner = skin(WALL)
        s = skin().cut(inner).intersect(slab(z=(T_BOT, BIG)))
        ring = (T_X[0] + WALL + T_RING, T_X[1] - WALL - T_RING), (T_Y[0] + WALL + T_RING, T_Y[1] - WALL - T_RING)
        r_ring = T_RV - WALL - T_RING
        # top: open over deck 2, a ring under the crown edge and 6 tabs on the deck
        s = s.cut(prism(*ring, (B.Z2T - 1.0, T_TOP + 1.0), r_ring))
        tabs = [tab(x, y, B.Z2T, T_TOP) for x, y in TOP_INSERTS]
        s = s.fuse(LEG.fuse(*tabs).intersect(inner))
        # bottom: a ring and 6 tabs under the pelvis plate
        lip = inner.intersect(slab(z=(B.Z0 - T_LIP, B.Z0))).cut(prism(*ring, (B.Z0 - 5.0, B.Z0 + 1.0), r_ring))
        tabs = [tab(x, y, B.Z0 - T_LIP, B.Z0) for x, y in BOT_INSERTS]
        s = s.fuse(lip.fuse(LEG.fuse(*tabs).intersect(inner)))
        for x, y in TOP_INSERTS:
            s = s.cut(cyl(CLR_M2, (x, y, B.Z2T - 1.0), (x, y, T_TOP + 1.0)))
        for x, y in BOT_INSERTS:
            s = s.cut(cyl(CLR_M2, (x, y, B.Z0 - 5.0), (x, y, B.Z0 + 1.0)))
        # shoulder mount flanges sit on deck 2 out to |y| = 54
        for sy in (1, -1):
            s = s.cut(bx((-28.6, 19.6), (33.0, 55.5) if sy > 0 else (-55.5, -33.0), (B.Z2T - 0.5, T_TOP + 1.0)))
        # flag badge on the chest
        fx = T_X[1]
        badge = prism((fx - 0.5, fx + 1.0), (-13.5, 13.5), (BADGE_Z - 9.5, BADGE_Z + 9.5), 2.0, "X")
        s = s.fuse(badge).cut(bx((fx + 0.4, fx + 2.0), (-12.0, 12.0), (BADGE_Z - 8.0, BADGE_Z + 8.0)))
        chest = s.intersect(slab(x=(T_SPLIT + GAP / 2, BIG)))
        keep = slab(z=(B.Z0 + 1.0, B.Z2T - 1.0)).cut(slab(y=(-50.0, 50.0)))
        chest = chest.fuse(seam_tongue(skin, T_SPLIT, keep))
        back = s.intersect(slab(x=(-BIG, T_SPLIT - GAP / 2)))
        for z in np.arange(124.0, 141.0, 4.0):          # vents behind the UBEC and the Black Pill
            back = back.cut(prism((T_X[0] - 1.0, T_X[0] + WALL + 1.0), (-15.0, 15.0), (z, z + 2.0), 0.9, "X"))
        self.parts["chest_shell"] = chest.clean()
        self.parts["back_shell"] = back.clean()
        for x, y in TOP_INSERTS:
            self.screws.append(("chest_shell" if x > T_SPLIT else "back_shell", (x, y, T_TOP), (0, 0, -1)))
        for x, y in BOT_INSERTS:
            self.screws.append(("chest_shell" if x > T_SPLIT else "back_shell", (x, y, B.Z0 - T_LIP), (0, 0, 1)))

    # ------------------------------------------------------------------ head
    def _head(self):
        def skin(t=0.0):
            return rbox(H_X, H_Y, (H_BOT, H_TOP), H_RV, H_RT, H_RB, t=t)

        inner = skin(WALL)
        s = skin().cut(inner)
        # open bottom, a 3 mm rolled rim left at the front and sides
        s = s.cut(prism((H_X[0] - 1.0, H_X[1] - WALL - 3.0), (H_Y[0] + WALL + 3.0, H_Y[1] - WALL - 3.0),
                        (H_BOT - 1.0, H_BOT + H_RB), 4.0))
        # tabs on the head deck, each under a guide tube up to a hole in the crown
        for x, y in HEAD_INSERTS:
            t = tab(x, y, HD3, HD3 + 2.0).fuse(cyl(TUBE_R[1], (x, y, HD3), (x, y, H_TOP)))
            s = s.fuse(t.intersect(inner))
        for x, y in HEAD_INSERTS:
            s = s.cut(cyl(CLR_M2, (x, y, HD3 - 1.0), (x, y, HD3 + 3.0)))
            s = s.cut(cyl(TUBE_R[0], (x, y, HD3 + 2.0), (x, y, H_TOP + 1.0)))
            self.screws.append(("head_front" if x > H_SPLIT else "head_back", (x, y, HD3 + 2.0), (0, 0, -1)))
        # ears with a mic grille
        ex, ez, er = EAR
        for sy in (1, -1):
            y0, y1 = sy * (H_Y[1] - 0.6), sy * (H_Y[1] + 2.4)
            s = s.fuse(cyl(er, (ex, y0, ez), (ex, y1, ez))).fuse(cyl(er - 2.5, (ex, y1, ez), (ex, y1 + sy, ez)))
            for dx in (-3.0, 0.0, 3.0):
                s = s.cut(prism((ex + dx - 0.6, ex + dx + 0.6), (sy * 26.0, sy * 34.0) if sy > 0 else (-34.0, -26.0),
                                (ez - 4.5, ez + 4.5), 0.5, "Y"))
        # notch round the pitch axis, so the back skirt clears the neck servos
        neck = cq.Solid.makeCylinder(NECK_R, 2 * NECK_Y, cq.Vector(B.NECK_X, -NECK_Y, B.HEAD_Z), cq.Vector(0, 1, 0))
        s = s.cut(neck.intersect(slab(z=(-BIG, B.HEAD_DECK))))
        axis = (cq.Vector(B.NECK_X, 0, B.HEAD_Z), cq.Vector(B.NECK_X, 1, B.HEAD_Z))
        chin = slab(z=(-BIG, B.HEAD_Z - CHIN)).rotate(*axis, -32.0)
        s = s.cut(chin)

        def window(t):
            (y0, y1), (z0, z1), r = WINDOW
            return prism((20.0, 60.0), (y0 - t, y1 + t), (z0 - t, z1 + t), r + t, "X")

        holes = cyl(CAM_HOLE, CAM_C, CAM_C + 40 * N).fuse(cyl(TOF_HOLE, TOF_C, TOF_C + 40 * N))
        visor = skin().cut(inner).intersect(window(-GAP))
        visor = visor.fuse(inner.cut(skin(WALL + 1.2)).intersect(window(LIP))).cut(holes).cut(chin)
        front = s.intersect(slab(x=(H_SPLIT + GAP / 2, BIG))).cut(window(0.0))
        keep = slab(z=(H_BOT + H_RB + 2.0, BIG))
        front = front.fuse(seam_tongue(skin, H_SPLIT, keep))
        self.parts["head_front"] = front.clean()
        self.parts["head_back"] = s.intersect(slab(x=(-BIG, H_SPLIT - GAP / 2))).clean()
        self.parts["visor"] = visor.clean()

    # ------------------------------------------------------------------ placing and checking
    HEAD = ("head_front", "head_back", "visor")

    def optics(self):
        """Camera and ToF boards on cam_mount (not in robot.assemble): PCB + parts, the camera with its lens block."""
        out = {}
        for code, c in (("CAM", CAM_C), ("TOF", TOF_C)):
            b = el.BOARDS[code]
            pcb = cq.Solid.makeBox(b.pcb + 1.0, b.size[0], b.size[1], cq.Vector(2.5, -b.size[0] / 2, -b.size[1] / 2))
            if code == "CAM":
                pcb = pcb.fuse(cq.Solid.makeBox(b.top, 9.0, 9.0, cq.Vector(2.5 + b.pcb, -4.5, -4.5)))
            out[f"board_{code}"] = pcb.rotate(cq.Vector(), cq.Vector(0, 1, 0), 15.0).moved(cq.Location(cq.Vector(*c)))
        return out

    def placed(self, q=None):
        q = q or {}
        loc = R.chain_T(self.body, R.HEAD_CHAIN, {k: v for k, v in q.items() if k in ("neck_yaw", "head_pitch")})
        return {n: (s.moved(loc["head_frame"]) if n in self.HEAD else s) for n, s in self.parts.items()}

    def collisions(self, quick=False, verbose=True):
        """Shell vs the robot (robot.assemble) at the zero pose and over robot.sweep_poses: {(shell, part): (mm3, q)}.
        Fixed shell vs fixed frame is checked once; every pose checks what moves."""
        worst = {}
        poses = [{}] + R.sweep_poses(quick)
        for i, q in enumerate(poses):
            comps = R.assemble(self.body, self.leg, q)
            loc = R.chain_T(self.body, R.HEAD_CHAIN, {k: v for k, v in q.items() if k in ("neck_yaw", "head_pitch")})
            for n, s in self.optics().items():
                comps[n] = s.moved(loc["head_frame"])
            fixed = {n for n in comps if not n.startswith("board_CAM") and not n.startswith("board_TOF")
                     and R.link_of(n, self.body, self.leg) == "torso"}
            shells = self.placed(q)
            pairs = [(a, b) for a in shells for b in comps if not (q and a not in self.HEAD and b in fixed)]
            pairs += list(itertools.combinations(shells, 2))
            for a, b in pairs:
                sb = comps.get(b, shells.get(b))
                if a in ("chest_shell", "back_shell") and inside(sb.BoundingBox(), T_FREE):
                    continue
                v = R.overlap(shells[a], sb)
                if v > R.TOL and v > worst.get((a, b), (0.0,))[0]:
                    worst[(a, b)] = (round(v, 2), q)
            if verbose:
                print(f"\r  pose {i + 1}/{len(poses)}  colliding pairs so far: {len(worst)}", end="", flush=True)
        if verbose:
            print()
        return worst

    def export(self, out=OUT):
        out.mkdir(exist_ok=True)
        for n, s in self.parts.items():
            cq.exporters.export(s, str(out / f"shell_{n}.step"))
            cq.exporters.export(s, str(out / f"shell_{n}.stl"), tolerance=0.05, angularTolerance=0.1)
        return out


def main():
    sh = BazanShell()
    bad = 0
    print("Bazan shell v1")
    for n, s in sh.parts.items():
        bb = s.BoundingBox()
        ok = s.isValid()
        bad += not ok
        print(f"  {n:12s} {s.Volume() / 1000:6.2f} cm3 ({s.Volume() * 1.27e-3:5.1f} g PETG)"
              f"  x {bb.xmin:6.1f}..{bb.xmax:5.1f}  y {bb.ymin:6.1f}..{bb.ymax:5.1f}  z {bb.zmin:6.1f}..{bb.zmax:5.1f}"
              f"  valid {ok}")
    print(f"  {len(sh.screws)} x M2x6 into the shell inserts")
    hits = sh.collisions("--quick" in sys.argv)
    for (a, b), (v, q) in sorted(hits.items()):
        print(f"    {a} x {b}: {v} mm3 at {q}")
    print(f"  {len(hits)} colliding pairs")
    bad += len(hits)
    if "--no-export" not in sys.argv:
        print(f"  wrote {sh.export()}/shell_*.step, .stl")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
