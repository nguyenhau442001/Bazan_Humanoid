"""One servo joint, the way every Mico joint is built.

    holder link   carries the servo in a pocket: the servo slides in from the shaft side and is held by its two
                  ears (2 × M2×6 into heat-set inserts). The pocket wall behind the servo carries an M3 insert
                  for the idler screw.
    driven link   two plates on the joint axis: the horn plate on the shaft side (the servo's round horn sits in a
                  recess, 2 self-tapping screws), and the idler plate on the other side with an MR63ZZ bearing on
                  an M3 button-head screw that threads into the holder. Both plates take a press-fit navy cap.

Local frame of a joint: origin on the joint axis in the plane of the servo case top, a = shaft direction,
b = from the shaft toward the far end of the case, w = a × b. Every stack position below is along a, in mm.
To replace a servo: pull the caps, undo the horn screw and the idler screw, lift the driven link off, undo the
two ear screws, unplug, slide the servo out along a.
"""
from dataclasses import dataclass, field

import cadquery as cq
import numpy as np

import hardware as hw

S = hw.MG92B
CB = S.length - S.shaft_off                        # case far end along b (16.9)
CB0 = -S.shaft_off                                 # case near end along b (-5.9)
CMID = (CB + CB0) / 2                              # case centre along b (5.5)
HOLE_B = (CMID - S.hole_pitch / 2, CMID + S.hole_pitch / 2)
TAB_A = (-(S.case_h - S.tab_low), -(S.case_h - S.tab_low) + S.tab_t)   # ear underside, top (-6.8, -4.3)
C = hw.CLEAR_SERVO

# stack along a (case top = 0)
CASE = (-S.case_h, 0.0)
IW = (CASE[0] - C - 3.6, CASE[0] - C)               # idler wall with the M3 insert
GAP = 0.8                                           # holder to driven plate
IP_IN = IW[0] - GAP                                 # idler plate inner face
LIP = 0.5
BRG = (IP_IN - LIP - 2.5, IP_IN - LIP)              # MR63ZZ
BOSS = (BRG[1], IW[0])                              # holder boss that clamps the inner race
WASHER = 0.5
BUTTON_H = 1.65                                     # ISO 7380 M3 head
CAP_T = 1.0
IP = (BRG[0] - WASHER - BUTTON_H - 0.15 - CAP_T, IP_IN)
HP = (S.tower_h + 0.6, None)                        # horn plate starts 0.6 above the tower
HORN_TOP = hw.HORN.disc_low + hw.HORN.disc_t       # case top to the top face of the horn disc (8.4)
HEAD_TOP = HORN_TOP + 1.6                          # horn centre-screw head
HP = (HP[0], HEAD_TOP + 0.2 + CAP_T)
CENTER = (HP[1] + IP[0]) / 2                        # middle of the stack: the link's centre plane
HALF = (HP[1] - IP[0]) / 2
HORN_R = 12.0
CAP_R = 5.0                                        # navy cap over the horn centre screw
IDLER_R = 7.5


def frame(origin, a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    w = np.cross(a, b)
    R = np.column_stack([b, w, a])
    return R, np.asarray(origin, float)


def to_location(R, p):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*(float(v) for v in R[0]), float(p[0]), *(float(v) for v in R[1]), float(p[1]),
                *(float(v) for v in R[2]), float(p[2]))
    return cq.Location(t)


def box(b, w, a):
    """Axis-aligned box in local coords from ranges (lo, hi)."""
    sz = [hi - lo for lo, hi in (b, w, a)]
    c = [(lo + hi) / 2 for lo, hi in (b, w, a)]
    return cq.Solid.makeBox(*sz, cq.Vector(*[ci - si / 2 for ci, si in zip(c, sz)]))


def cyl(r, a0, a1, b=0.0, w=0.0):
    return cq.Solid.makeCylinder(r, a1 - a0, cq.Vector(b, w, a0), cq.Vector(0, 0, 1))


@dataclass
class Hole:
    part: str
    id: str
    kind: str          # key of hardware.HOLES
    fastener: str      # what goes in it
    pos: tuple         # entry point, leg frame (mm)
    dir: tuple         # drilling direction, unit
    d: float
    depth: float       # 0 = through
    joint: str


@dataclass
class Joint:
    name: str
    holder: str
    driven: str
    origin: tuple            # a point on the joint axis at the middle of the stack (leg frame)
    a: tuple                 # shaft direction
    b: tuple                 # servo body direction
    servo: hw.Servo = S
    idler: bool = True
    holes: list = field(default_factory=list)

    def __post_init__(self):
        R, p = frame(self.origin, self.a, self.b)
        self.R = R
        self.p = p - CENTER * R[:, 2]          # local origin: case top plane on the axis
        self.L = to_location(self.R, self.p)

    # ---------------------------------------------------------- helpers
    def world(self, v):
        return tuple(np.round(self.R @ np.asarray(v, float) + self.p, 3))

    def wdir(self, v):
        return tuple(np.round(self.R @ np.asarray(v, float), 6))

    def place(self, s):
        return s.moved(self.L)

    def axis_point(self, a):
        return self.world((0, 0, a))

    def a_range(self, rng):
        """A stack range along a, as world coordinates of its two ends on the axis."""
        return self.axis_point(rng[0]), self.axis_point(rng[1])

    def _hole(self, part, hid, kind, fastener, pos_l, dir_l, d, depth):
        self.holes.append(Hole(part, f"{self.name}.{hid}", kind, fastener, self.world(pos_l), self.wdir(dir_l), d,
                               depth, self.name))

    # ---------------------------------------------------------- the servo itself
    def servo_solid(self):
        s = box((CB0, CB), (-S.width / 2, S.width / 2), CASE)
        s = s.fuse(box((CMID - S.tab_span / 2, CMID + S.tab_span / 2), (-S.width / 2, S.width / 2), TAB_A))
        s = s.fuse(cyl(S.tower_d / 2, 0, S.tower_h))
        s = s.fuse(cyl(S.spline_d / 2, S.tower_h, S.spline_top))
        for hb in HOLE_B:
            s = s.cut(cyl(S.hole_d / 2, TAB_A[0] - 1, TAB_A[1] + 1, hb))
        return self.place(s)

    def horn_solid(self):
        h = cyl(hw.HORN.disc_d / 2, hw.HORN.disc_low, HORN_TOP)
        h = h.fuse(cyl(hw.HORN.hub_d / 2, S.tower_h + 0.4, hw.HORN.disc_low))
        return self.place(h)

    # ---------------------------------------------------------- holder: the pocket
    def holder_add(self):
        top = TAB_A[0]
        s = box((CB0 - C - hw.WALL, CB + C + hw.WALL), (-S.width / 2 - C - hw.WALL, S.width / 2 + C + hw.WALL),
                (IW[0], top))
        for hb in HOLE_B:
            s = s.fuse(cyl(3.2, IW[0], top, hb))
        if self.idler:
            s = s.fuse(cyl(2.1, BOSS[0], BOSS[1]))
        return self.place(s)

    def holder_cut(self, part):
        big = 60
        cav = box((CB0 - C, CB + C), (-S.width / 2 - C, S.width / 2 + C), (CASE[0] - C, big))
        ears = box((CMID - S.tab_span / 2 - C, CMID + S.tab_span / 2 + C), (-S.width / 2 - C, S.width / 2 + C),
                   (TAB_A[0], big))
        cable = box((CB, CB + C + hw.WALL + 5), (-S.cable_w / 2 - 0.3, S.cable_w / 2 + 0.3),
                    (CASE[0] + S.cable_z[0] - 0.5, CASE[0] + S.cable_z[1] + 0.5))
        s = cav.fuse(ears).fuse(cable)
        spec = hw.HOLES["ins_m2"]
        for i, hb in enumerate(HOLE_B):
            s = s.fuse(cyl(spec.d / 2, TAB_A[0] - spec.depth, TAB_A[0], hb))
            self._hole(part, f"ear{i + 1}", "ins_m2", "INS-M2 + M2x6 (servo ear)", (hb, 0, TAB_A[0]), (0, 0, -1),
                       spec.d, spec.depth)
        if self.idler:
            spec = hw.HOLES["ins_m3"]
            s = s.fuse(cyl(spec.d / 2, BOSS[0], BOSS[0] + spec.depth))
            self._hole(part, "idler_insert", "ins_m3", "INS-M3 (idler pivot)", (0, 0, BOSS[0]), (0, 0, 1),
                       spec.d, spec.depth)
        return self.place(s)

    # ---------------------------------------------------------- driven: horn plate and idler plate
    def horn_disc(self):
        return self.place(cyl(HORN_R, HP[0], HP[1]))

    def idler_disc(self):
        return self.place(cyl(IDLER_R, IP[0], IP[1]))

    def driven_cut(self, part):
        h = hw.HORN
        s = cyl(h.disc_d / 2 + 0.2, HP[0] - 1, HORN_TOP)                      # horn disc recess (from inside)
        s = s.fuse(cyl(h.hub_d / 2 + 0.4, HP[0] - 1, HORN_TOP))
        s = s.fuse(cyl(hw.HOLES["access"].d / 2, HP[0] - 1, HP[1] + 1))       # horn screw access
        s = s.fuse(cyl(CAP_R + 0.05, HP[1] - CAP_T, HP[1] + 1))               # navy cap recess
        self._hole(part, "access", "access", "horn centre screw (comes with the servo)", (0, 0, HP[1]), (0, 0, -1),
                   hw.HOLES["access"].d, 0)
        # the horn is screwed to the plate from the servo side (screw heads between horn and case top), blind holes
        spec = hw.HOLES["pilot_st2"]
        for i, sw in enumerate((-1, 1)):
            s = s.fuse(cyl(spec.d / 2, HORN_TOP - 0.1, HP[1] - 0.4, 0, sw * h.screw_r))
            self._hole(part, f"horn{i + 1}", "pilot_st2", "ST2x4 through the horn (plate to horn)",
                       (0, sw * h.screw_r, HORN_TOP), (0, 0, 1), spec.d, round(HP[1] - 0.4 - HORN_TOP, 2))
        if self.idler:
            s = s.fuse(cyl(2.3, IP[1] - LIP - 0.01, IP[1] + 1))                # lip hole: holder boss passes
            s = s.fuse(cyl(hw.HOLES["bearing"].d / 2, IP[0] + CAP_T - 0.01, BRG[1]))   # bearing + head pocket
            s = s.fuse(cyl(IDLER_R - 1.0, IP[0] - 1, IP[0] + CAP_T))             # cap recess
            self._hole(part, "bearing", "bearing", "MR63ZZ + W3 + M3x6 button head (idler)",
                       (0, 0, IP[0] + CAP_T), (0, 0, 1), hw.HOLES["bearing"].d, BRG[1] - IP[0] - CAP_T)
        return self.place(s)

    def tool_paths(self):
        """Straight tool paths to every screw of this joint, as (label, solid, part that must be off first).
        A path starts at the screw head and runs 35 mm out along the screw axis."""
        out = []
        for i, hb in enumerate(HOLE_B):                 # ear screws: 1.5 mm hex key, ball end Ø1.5 + clearance
            out.append((f"{self.name}: ear screw {i + 1} (1.5 mm key)",
                        self.place(cyl(1.1, TAB_A[1] + 2.0, TAB_A[1] + 37.0, hb)), self.driven))
        out.append((f"{self.name}: horn centre screw (PH0)", self.place(cyl(1.5, HEAD_TOP, HEAD_TOP + 35.0)), None))
        if self.idler:
            head = BRG[0] - WASHER - BUTTON_H
            out.append((f"{self.name}: idler screw (2.0 mm key)", self.place(cyl(1.3, head - 35.0, head)), None))
        return out

    def caps(self):
        c = [self.place(cyl(CAP_R - 0.05, HP[1] - CAP_T, HP[1]))]
        if self.idler:
            c.append(self.place(cyl(IDLER_R - 1.05, IP[0], IP[0] + CAP_T)))
        return c


def stack_summary():
    return dict(case_top_from_centre=-CENTER, stack_width=2 * HALF, horn_plate=HP, idler_plate=IP,
                idler_wall=IW, bearing=BRG, hole_b=HOLE_B, tab_a=TAB_A)


if __name__ == "__main__":
    for k, v in stack_summary().items():
        print(k, v)
