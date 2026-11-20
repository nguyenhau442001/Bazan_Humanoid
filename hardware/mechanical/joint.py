"""One servo joint, the way every Mico joint is built.

    holder link   carries the servo in a pocket: the servo slides in from the shaft side and is held by its two
                  ears (2 × M2×6 into heat-set inserts). The pocket wall behind the servo carries an M3 insert
                  for the idler screw. The lead leaves through a window in the end wall, wide enough for its
                  plug, so a servo goes in and out with the plug on.
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


class Pocket:
    """The stack of one joint built around one servo type: every position along a (case top = 0), along b
    (shaft = 0) and the joint's printed sizes. One pocket per servo size class (hardware.Servo.size): every
    9 g servo (MG92B, MG90S, SG90) shares the MG92B pocket, every standard servo the DS3218 pocket."""

    def __init__(self, s: hw.Servo, horn: hw.Horn):
        self.S, self.horn = s, horn
        self.CB = s.length - s.shaft_off                          # case far end along b
        self.CB0 = -s.shaft_off                                   # case near end along b
        self.CMID = (self.CB + self.CB0) / 2                      # case centre along b
        self.HOLE_B = (self.CMID - s.hole_pitch / 2, self.CMID + s.hole_pitch / 2)
        # ear holes: one per ear on the case centre line (9 g), or a pair across w (standard servos)
        self.EAR_HOLES = [(hb, ww) for hb in self.HOLE_B
                          for ww in ((-s.hole_w / 2, s.hole_w / 2) if s.hole_w else (0.0,))]
        self.TAB_A = (-(s.case_h - s.tab_low), -(s.case_h - s.tab_low) + s.tab_t)   # ear underside, top
        self.CASE = (-s.case_h, 0.0)
        self.IW = (self.CASE[0] - C - 3.6, self.CASE[0] - C)     # idler wall with the M3 insert
        self.EAR_INSERT = "ins_m3" if s.ear_screw.startswith("M3") else "ins_m2"
        self.EAR_BOSS_R = 3.2 if self.EAR_INSERT == "ins_m2" else 4.0
        self.EAR_KEY_R = 1.1 if self.EAR_INSERT == "ins_m2" else 1.5     # hex key ball end + clearance
        hp0 = s.tower_h + 0.6                                     # horn plate starts 0.6 above the tower
        self.HORN_TOP = horn.disc_low + horn.disc_t               # case top to the top face of the horn disc
        self.HEAD_TOP = self.HORN_TOP + 1.6                       # horn centre-screw head
        self.HP = (hp0, self.HEAD_TOP + 0.2 + CAP_T)
        brg0 = self.IW[0] - GAP - LIP - 2.5                       # MR63ZZ, default gap
        self.IP = (brg0 - WASHER - BUTTON_H - 0.15 - CAP_T, self.IW[0] - GAP)   # idler plate, default gap
        self.CENTER = (self.HP[1] + self.IP[0]) / 2               # middle of the stack: the link's centre plane
        self.HALF = (self.HP[1] - self.IP[0]) / 2
        self.HORN_R = max(12.0, horn.disc_d / 2 + 2.0)            # horn plate disc: ≥ 1.8 mm rim round the recess
        self.WINDOW = (max(s.cable_w, hw.PLUG[0]) + 0.6, s.cable_z[1] - s.cable_z[0] + 1.0)
        self.CAP_R = 5.0                                          # navy cap over the horn centre screw
        self.IDLER_R = 7.5


# stack along a, shared by every pocket (case top = 0; the idler side hangs off the idler wall)
C = hw.CLEAR_SERVO
GAP = 0.8                                           # holder to driven plate
LIP = 0.5
LIP_D = 5.0                                         # lip hole round the Ø4.0 spacer tube: 0.5 radial
SPACER_D = 4.0                                      # brass tube Ø4.0 × 0.45: clamps the inner race (land ~Ø3.9)
INSERT_M3_DEPTH = 3.3                               # M3 short insert (L3) in the 3.6 wall: 0.3 skin to the pocket
LEG_GAP = 4.6                                       # leg joints: ≥ 3.9 horn travel off the spline + 0.7
WASHER = 0.5
BUTTON_H = 1.65                                     # ISO 7380 M3 head
CAP_T = 1.0

# the 9 g pocket (MG92B): module-level names kept for the parts built around it
S = hw.MG92B
CB = S.length - S.shaft_off                        # case far end along b (16.9)
CB0 = -S.shaft_off                                 # case near end along b (-5.9)
CMID = (CB + CB0) / 2                              # case centre along b (5.5)
HOLE_B = (CMID - S.hole_pitch / 2, CMID + S.hole_pitch / 2)
TAB_A = (-(S.case_h - S.tab_low), -(S.case_h - S.tab_low) + S.tab_t)   # ear underside, top (-6.8, -4.3)
CASE = (-S.case_h, 0.0)
IW = (CASE[0] - C - 3.6, CASE[0] - C)               # idler wall with the M3 insert
IP_IN = IW[0] - GAP                                 # idler plate inner face
BRG = (IP_IN - LIP - 2.5, IP_IN - LIP)              # MR63ZZ
BOSS = (BRG[1], IW[0])                              # spacer tube on the inner race (default gap)
POCKETS = {}


def pocket(servo: hw.Servo) -> Pocket:
    """The pocket a servo goes into: the one built for its size class."""
    base = hw.POCKET_SERVO[servo.size]
    if base.name not in POCKETS:
        POCKETS[base.name] = Pocket(base, hw.HORNS[base.name])
    return POCKETS[base.name]


P9 = pocket(S)
IP = P9.IP
HP = P9.HP
HORN_TOP = P9.HORN_TOP                             # case top to the top face of the horn disc (8.4)
HEAD_TOP = P9.HEAD_TOP
CENTER = P9.CENTER
HALF = P9.HALF
HORN_R = P9.HORN_R
WINDOW = P9.WINDOW                                 # cable window in the end wall (across w, along a)
CAP_R = P9.CAP_R
IDLER_R = P9.IDLER_R


def idler_screw(head_underside, wall=None):
    """Shortest ISO 7380 M3 button head that reaches 1.7..3.0 mm into the wall insert (wall: idler wall face)."""
    wall = IW[0] if wall is None else wall
    for n in (6, 8, 10, 12, 14, 16):
        tip = head_underside + n
        if wall + 1.7 <= tip <= wall + 3.0:
            return f"M3x{n}"
    raise ValueError(f"no M3 button head length fits head at {head_underside}")


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
    idler_part: str = ""     # the idler plate is a separate part (bolted to the driven link), see body.head_idler
    gap: float = GAP         # holder wall to idler plate; ≥ the horn's travel off the spline to take the link off
    keep_idler: bool = False  # a wider gap moves the servo and horn plate out instead of the idler plate in
    holes: list = field(default_factory=list)

    def __post_init__(self):
        self.P = P = pocket(self.servo)
        IW = P.IW
        # idler stack of this joint (the horn side and the servo do not depend on the gap)
        self.IP_IN = IW[0] - self.gap
        self.BRG = (self.IP_IN - LIP - 2.5, self.IP_IN - LIP)
        self.IP = (self.BRG[0] - WASHER - BUTTON_H - 0.15 - CAP_T, self.IP_IN)
        self.SPACER = (self.BRG[1], IW[0])             # brass tube on the inner race (replaces the printed boss)
        self.idler_screw = idler_screw(self.BRG[0] - WASHER, IW[0])
        R, p = frame(self.origin, self.a, self.b)
        self.R = R
        self.p = p - P.CENTER * R[:, 2]        # local origin: case top plane on the axis
        if self.keep_idler:
            self.p = self.p + (self.gap - GAP) * R[:, 2]
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

    def ntc_cut(self, part, side=1, r=1.8, out=0.0):
        """Hole through a side wall of the pocket at the case middle, for an NTC bead pressed on the servo case
        (Kapton tape over it). side = +1 / -1 along w; out extends the cylinder past the wall (keep-out checks)."""
        P = self.P
        a = (P.CASE[0] + P.TAB_A[0]) / 2
        w0 = P.S.width / 2 - 0.5
        w1 = P.S.width / 2 + C + hw.WALL + 0.5 + out
        c = cq.Solid.makeCylinder(r, w1 - w0, cq.Vector(P.CMID, side * w0, a), cq.Vector(0, side, 0))
        if not out:
            self._hole(part, "ntc", "ntc", "NTC 10k bead on the servo case (Kapton), wire out", (P.CMID, side * w1, a),
                       (0, -side, 0), 2 * r, 0)
        return self.place(c)

    # ---------------------------------------------------------- the servo itself
    def servo_solid(self, cots=False):
        P = self.P
        if cots:
            try:
                import cots_loader
                return cots_loader.place_servo_in_joint(self)
            except Exception:
                pass
        s = box((P.CB0, P.CB), (-P.S.width / 2, P.S.width / 2), P.CASE)
        s = s.fuse(box((P.CMID - P.S.tab_span / 2, P.CMID + P.S.tab_span / 2), (-P.S.width / 2, P.S.width / 2),
                       P.TAB_A))
        s = s.fuse(cyl(P.S.tower_d / 2, 0, P.S.tower_h))
        s = s.fuse(cyl(P.S.spline_d / 2, P.S.tower_h, P.S.spline_top))
        for hb, ww in P.EAR_HOLES:
            s = s.cut(cyl(P.S.hole_d / 2, P.TAB_A[0] - 1, P.TAB_A[1] + 1, hb, ww))
        return self.place(s)

    def horn_solid(self):
        P = self.P
        h = cyl(P.horn.disc_d / 2, P.horn.disc_low, P.HORN_TOP)
        h = h.fuse(cyl(P.horn.hub_d / 2, P.S.tower_h + 0.4, P.horn.disc_low))
        return self.place(h)

    # ---------------------------------------------------------- holder: the pocket
    def holder_add(self):
        P = self.P
        top = P.TAB_A[0]
        s = box((P.CB0 - C - hw.WALL, P.CB + C + hw.WALL), (-P.S.width / 2 - C - hw.WALL, P.S.width / 2 + C + hw.WALL),
                (P.IW[0], top))
        for hb, ww in P.EAR_HOLES:
            s = s.fuse(cyl(P.EAR_BOSS_R, P.IW[0], top, hb, ww))
        return self.place(s)

    def holder_cut(self, part):
        P = self.P
        big = 60
        cav = box((P.CB0 - C, P.CB + C), (-P.S.width / 2 - C, P.S.width / 2 + C), (P.CASE[0] - C, big))
        ears = box((P.CMID - P.S.tab_span / 2 - C, P.CMID + P.S.tab_span / 2 + C),
                   (-P.S.width / 2 - C, P.S.width / 2 + C), (P.TAB_A[0], big))
        cable = box((P.CB, P.CB + C + hw.WALL + 5), (-P.WINDOW[0] / 2, P.WINDOW[0] / 2),
                    (P.CASE[0] + P.S.cable_z[0] - 0.5, P.CASE[0] + P.S.cable_z[1] + 0.5))
        s = cav.fuse(ears).fuse(cable)
        spec = hw.HOLES[P.EAR_INSERT]
        ins = "INS-M3" if P.EAR_INSERT == "ins_m3" else "INS-M2"
        for i, (hb, ww) in enumerate(P.EAR_HOLES):
            s = s.fuse(cyl(spec.d / 2, P.TAB_A[0] - spec.depth, P.TAB_A[0], hb, ww))
            self._hole(part, f"ear{i + 1}", P.EAR_INSERT, f"{ins} + {P.S.ear_screw} (servo ear)", (hb, ww, P.TAB_A[0]),
                       (0, 0, -1), spec.d, spec.depth)
        if self.idler:
            # M3 insert pressed into the wall face from outside (no boss: the Ø4.6 knurl never fitted the Ø4.2
            # boss), 0.3 mm skin left to the pocket; the spacer tube clamps the inner race against it
            spec = hw.HOLES["ins_m3"]
            s = s.fuse(cyl(spec.d / 2, P.IW[0] - 0.01, P.IW[0] + INSERT_M3_DEPTH))
            self._hole(part, "idler_insert", "ins_m3",
                       f"INS-M3 + spacer tube Ø4×0.45 L{self.SPACER[1] - self.SPACER[0]:.1f} + "
                       f"{self.idler_screw} (idler pivot)", (0, 0, P.IW[0]), (0, 0, 1), spec.d, INSERT_M3_DEPTH)
        return self.place(s)

    # ---------------------------------------------------------- driven: horn plate and idler plate
    def horn_disc(self):
        P = self.P
        return self.place(cyl(P.HORN_R, P.HP[0], P.HP[1]))

    def idler_disc(self):
        P = self.P
        return self.place(cyl(P.IDLER_R, self.IP[0], self.IP[1]))

    def spacer_solid(self):
        """Brass tube Ø4.0 × 0.45 (bought) between the holder wall and the bearing inner race."""
        tube = cyl(SPACER_D / 2, *self.SPACER)
        return self.place(tube.cut(cyl(SPACER_D / 2 - 0.45, self.SPACER[0] - 1, self.SPACER[1] + 1)))

    def driven_cut(self, part, side="both"):
        """Horn-plate features (side "horn"), idler-plate features (side "idler") or both."""
        P = self.P
        h = P.horn
        if side == "idler":
            s = cyl(0.01, -500, -499)                                          # empty start, far away
            return self.place(self._idler_cut(s, part))
        s = cyl(h.disc_d / 2 + 0.2, P.HP[0] - 1, P.HORN_TOP)                      # horn disc recess (from inside)
        s = s.fuse(cyl(h.hub_d / 2 + 0.4, P.HP[0] - 1, P.HORN_TOP))
        s = s.fuse(cyl(hw.HOLES["access"].d / 2, P.HP[0] - 1, P.HP[1] + 1))       # horn screw access
        s = s.fuse(cyl(P.CAP_R + 0.05, P.HP[1] - CAP_T, P.HP[1] + 1))               # navy cap recess
        self._hole(part, "access", "access", "horn centre screw (comes with the servo)", (0, 0, P.HP[1]), (0, 0, -1),
                   hw.HOLES["access"].d, 0)
        # the horn is screwed to the plate from the servo side (screw heads between horn and case top), blind holes
        spec = hw.HOLES["pilot_st2"]
        for i, sw in enumerate((-1, 1)):
            s = s.fuse(cyl(spec.d / 2, P.HORN_TOP - 0.1, P.HP[1] - 0.4, 0, sw * h.screw_r))
            self._hole(part, f"horn{i + 1}", "pilot_st2", "ST2x3 through the horn (plate to horn)",
                       (0, sw * h.screw_r, P.HORN_TOP), (0, 0, 1), spec.d, round(P.HP[1] - 0.4 - P.HORN_TOP, 2))
        if self.idler and side == "both":
            s = self._idler_cut(s, part)
        return self.place(s)

    def _idler_cut(self, s, part):
        P = self.P
        IP, BRG = self.IP, self.BRG
        s = s.fuse(cyl(LIP_D / 2, IP[1] - LIP - 0.01, IP[1] + 1))             # lip hole: the spacer tube passes
        s = s.fuse(cyl(hw.HOLES["bearing"].d / 2, IP[0] + CAP_T - 0.01, BRG[1]))   # bearing + head pocket
        s = s.fuse(cyl(P.IDLER_R - 1.0, IP[0] - 1, IP[0] + CAP_T))                 # cap recess
        self._hole(part, "bearing", "bearing", f"MR63ZZ + W3 + {self.idler_screw} button head (idler)",
                   (0, 0, IP[0] + CAP_T), (0, 0, 1), hw.HOLES["bearing"].d, BRG[1] - IP[0] - CAP_T)
        return s

    def tool_paths(self):
        """Straight tool paths to every screw of this joint, as (label, solid, part that must be off first).
        A path starts at the screw head and runs 35 mm out along the screw axis."""
        P = self.P
        out = []
        key = "1.5" if P.EAR_INSERT == "ins_m2" else "2.5"
        for i, (hb, ww) in enumerate(P.EAR_HOLES):     # ear screws: hex key, ball end + clearance
            out.append((f"{self.name}: ear screw {i + 1} ({key} mm key)",
                        self.place(cyl(P.EAR_KEY_R, P.TAB_A[1] + 2.0, P.TAB_A[1] + 37.0, hb, ww)), self.driven))
        out.append((f"{self.name}: horn centre screw (PH0)", self.place(cyl(1.5, P.HEAD_TOP, P.HEAD_TOP + 35.0)), None))
        if self.idler:
            head = self.BRG[0] - WASHER - BUTTON_H
            out.append((f"{self.name}: idler screw (2.0 mm key)", self.place(cyl(1.3, head - 35.0, head)), None))
        return out

    def caps(self):
        P = self.P
        c = [self.place(cyl(P.CAP_R - 0.05, P.HP[1] - CAP_T, P.HP[1]))]
        if self.idler:
            c.append(self.place(cyl(P.IDLER_R - 1.05, self.IP[0], self.IP[0] + CAP_T)))
        return c


def stack_summary():
    return dict(case_top_from_centre=-CENTER, stack_width=2 * HALF, horn_plate=HP, idler_plate=IP,
                idler_wall=IW, bearing=BRG, hole_b=HOLE_B, tab_a=TAB_A)


if __name__ == "__main__":
    for k, v in stack_summary().items():
        print(k, v)
