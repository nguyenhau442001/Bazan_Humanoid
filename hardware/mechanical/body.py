"""Mico body v4: pelvis, the three-deck torso stack, board sleds, arms, neck and head, every hole placed.

Robot frame: origin midway between the two hip points H (leg.py), x forward, y left, z up, standing straight.
The legs are leg.py's leg v3 unchanged: the left leg is the leg frame moved to y = +HIP_Y, the right its mirror.

Design rules that make it easy to change motors and boards later:

    R1  One joint, one cartridge. All 20 joints use joint.py's 9 g pocket (MG92B / MG90S / SG90 share the case),
        horn plate and, where loaded, the MR63ZZ idler. Any servo can go into any joint; a servo is replaced by
        opening only its own joint.
    R2  Boards never touch a deck. Each board sits on its own printed sled (4 bosses with M2 inserts at the board's
        holes), and each sled is held to its deck by 2 × M2×6 into a bay: two inserts 40 mm (SL-40) or 20 mm
        (SL-20) apart. A new board means a new sled (a 15-minute print), never a new deck.
    R3  The torso is a stack: pelvis plate (deck 0) + deck 1 + deck 2 on 4 nylon M3 standoffs per level. Decks
        come apart level by level; nothing is glued.
    R4  Every lead leaves its joint through the notch at the far end of the servo case and enters the torso at a
        fixed place: leg leads through the two slots in deck 0, arm and head leads through the back notch of
        decks 2 and 1.

Left side parts are drawn; the right side is the mirror in y (R_ print files).
"""
import cadquery as cq
import numpy as np

import electronics as el
import joint as J
import leg as LEG

# ------------------------------------------------------------------ the stack
HIP_Y = LEG.HIP_Y
Z0 = 73.032            # pelvis plate underside = top of the hip mounts (leg.py summary 'pelvis_floor')
T0 = 4.5               # pelvis plate
TD = 3.0               # decks 1 and 2
SO_A = 40.0            # standoff deck 0 → deck 1 (PCA9685 with plugs underneath)
SO_B = 35.0            # standoff deck 1 → deck 2 (Black Pill in its headers underneath)
Z0T = Z0 + T0
Z1 = Z0T + SO_A
Z1T = Z1 + TD
Z2 = Z1T + SO_B
Z2T = Z2 + TD
DX = (-49.0, 40.0)     # deck outline, all three decks
DY = (-54.0, 54.0)
SO_XY = [(-44.0, 49.0), (-44.0, -49.0), (35.0, 49.0), (35.0, -49.0)]
SLED_T = 4.0           # sled plate
BOSS_H = 5.0           # board boss above the sled
FLANGE_T = 3.0         # flange of the shoulder and neck mounts

# ------------------------------------------------------------------ joints above the pelvis (left side)
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
NX, NY, NZ = (-1, 0, 0), (0, -1, 0), (0, 0, -1)
SH_CT_Y = 60.5                              # shoulder pitch case top (shaft face) at y
SH_Z = Z2T + FLANGE_T + 8.3                 # shoulder pitch axis: pocket floor on the flange
SH_X = 0.0
ARM_OUT = 6.0                               # arm moved out so it clears the torso shell (Mico v4 look)
ROLL_Y = SH_CT_Y + J.HP[1] + 8.3 + ARM_OUT  # roll pocket outboard of the shoulder horn plate
ROLL_Z = SH_Z - 14.0                        # low enough to leave the shoulder horn screw reachable
ROLL_CT_X = 9.5
ELB_CT_Y = 69.7 + ARM_OUT                   # forearm plate inner face at y = ELB_CT_Y - 11.2 = 64.5
ELB_Z = ROLL_Z - 35.5                       # elbow pocket clears the roll pocket at any roll angle
NECK_X = -3.0
NECK_CT_Z = Z2T + FLANGE_T + 26.5
HEAD_Z = NECK_CT_Z + J.HP[1] + 8.3          # head pitch axis: its pocket floor on the neck horn plate
HEAD_DECK = HEAD_Z + 20.0                   # underside of the head deck


def ct_joint(name, holder, driven, ct, a, b, idler):
    """A joint given the point where its shaft crosses the servo case top."""
    o = np.asarray(ct, float) + J.CENTER * np.asarray(a, float)
    return J.Joint(name, holder, driven, tuple(o), a, b, idler=idler)


def bx(x, y, z):
    return LEG.bx(x, y, z)


def cyl(r, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    return cq.Solid.makeCylinder(r, float(np.linalg.norm(d)), cq.Vector(*p0), cq.Vector(*(d / np.linalg.norm(d))))


fuse = LEG.fuse


class Body:
    def __init__(self):
        self.joints = {
            "shoulder_pitch": ct_joint("shoulder_pitch", "shoulder_mount", "shoulder_link", (SH_X, SH_CT_Y, SH_Z),
                                       Y, NX, False),
            "shoulder_roll": ct_joint("shoulder_roll", "shoulder_link", "upper_arm", (ROLL_CT_X, ROLL_Y, ROLL_Z),
                                      X, NZ, False),
            "elbow": ct_joint("elbow", "upper_arm", "forearm", (0.0, ELB_CT_Y, ELB_Z), NY, NZ, False),
            "neck_yaw": ct_joint("neck_yaw", "neck_mount", "neck_link", (NECK_X, 0.0, NECK_CT_Z), Z, NX, False),
            "head_pitch": J.Joint("head_pitch", "neck_link", "head_frame", (NECK_X, 0.0, HEAD_Z), Y, NX, idler=True),
        }
        self.extra = []          # holes that are not joint features
        self.boards = {}         # placed board envelopes, for the checks and the pictures
        self.bays = {}           # bay name -> (deck, centre, axis, pitch, sled)
        self.parts = {}
        self.parts["pelvis_plate"] = self._pelvis()
        self.parts["deck_1"] = self._deck("deck_1", Z1)
        self.parts["deck_2"] = self._deck("deck_2", Z2)
        self._sleds()
        self.parts["shoulder_mount"] = self._finish("shoulder_mount", self._shoulder_mount())
        self.parts["shoulder_link"] = self._finish("shoulder_link", self._shoulder_link())
        self.parts["upper_arm"] = self._finish("upper_arm", self._upper_arm())
        self.parts["forearm"] = self._finish("forearm", self._forearm())
        self.parts["neck_mount"] = self._finish("neck_mount", self._neck_mount())
        self.parts["neck_link"] = self._finish("neck_link", self._neck_link())
        self.parts["head_frame"] = self._finish("head_frame", self._head_frame())
        self.parts["cam_mount"] = self._cam_mount()
        # access to the horn screws hidden under the next joint
        sl = self.parts["shoulder_link"].cut(cyl(3.0, (SH_X, SH_CT_Y + 5, SH_Z), (SH_X, ROLL_Y + 12, SH_Z)))
        self.parts["shoulder_link"] = sl.clean()
        nl = self.parts["neck_link"].cut(cyl(2.6,
                                             (NECK_X, 0, NECK_CT_Z + 5), (NECK_X, 0, HEAD_Z + 10.0)))
        self.parts["neck_link"] = nl.clean()
        self.extra.append(J.Hole("neck_link", "neck_yaw.service", "access",
                                 "horn screw of neck yaw: straight down through both walls of the empty "
                                 "head-pitch pocket",
                                 (NECK_X, 0.0, round(HEAD_Z + 8.3, 3)), (0, 0, -1), 5.2, 0, "neck_yaw"))
        self.extra.append(J.Hole("shoulder_link", "shoulder_pitch.service", "access",
                                 "horn screw of shoulder pitch, under the roll pocket",
                                 (SH_X, round(ROLL_Y + 8.3, 3), SH_Z), (0, -1, 0), 6.0, 0, "shoulder_pitch"))

    # ------------------------------------------------------------------ helpers
    def _hole(self, part, hid, kind, fastener, pos, d, depth, direction=(0, 0, -1), joint="body"):
        self.extra.append(J.Hole(part, hid, kind, fastener, tuple(round(float(v), 3) for v in pos), direction, d, depth,
                                 joint))

    def _finish(self, name, body):
        for j in self.joints.values():
            if j.holder == name:
                body = body.cut(j.holder_cut(name))
            if j.driven == name:
                body = body.cut(j.driven_cut(name))
        return body.clean()

    def holes(self):
        out = [h for j in self.joints.values() for h in j.holes] + self.extra
        return sorted(out, key=lambda h: (h.part, h.id))

    # ------------------------------------------------------------------ deck 0: the pelvis plate
    BATT_C = (-4.5, 0.0)
    PCA_C = (-3.5, 36.5)
    HIP_SCREWS = [(x, s * (HIP_Y + d)) for s in (1, -1) for x in (-20.0, 10.0) for d in (-10.0, 10.0)]

    def _pelvis(self):
        p = "pelvis_plate"
        b = el.BOARDS["BATT"]
        bx0, bx1 = self.BATT_C[0] - b.size[0] / 2 - 1.0, self.BATT_C[0] + b.size[0] / 2 + 1.0
        s = bx(DX, DY, (Z0, Z0T))
        walls = [bx((bx0 - 2.0, bx0), (-12.0, 12.0), (Z0T, Z0T + 8.0)),
                 bx((bx1, bx1 + 2.0), (-12.0, 12.0), (Z0T, Z0T + 8.0))]
        for sy in (1, -1):
            for x0, x1 in ((-30.0, -24.0), (16.0, 22.0)):
                walls.append(bx((x0, x1), (16.0, 18.0) if sy > 0 else (-18.0, -16.0), (Z0T, Z0T + 8.0)))
        s = fuse(s, *walls)
        # standoffs: M3 insert through the plate, flush with the top
        for i, (x, y) in enumerate(SO_XY):
            s = s.cut(cyl(2.0, (x, y, Z0 - 1), (x, y, Z0T + 1)))
            self._hole(p, f"so{i + 1}", "ins_m3_thru", "INS-M3, then a SO40 standoff screwed in from the top",
                       (x, y, Z0T), 4.0, 0)
        # leg modules: 4 × M2×6 each, head sunk in a counterbore, into hip_mount's inserts
        for i, (x, y) in enumerate(self.HIP_SCREWS):
            s = s.cut(cyl(1.15, (x, y, Z0 - 1), (x, y, Z0T + 1))).cut(cyl(2.2, (x, y, Z0T - 2.2), (x, y, Z0T + 1)))
            side = "L" if y > 0 else "R"
            self._hole(p, f"leg{side}{i % 4 + 1}", "cb_m2", f"M2x6 into the {side} hip_mount insert (head flush)",
                       (x, y, Z0T), 2.3, 0)
        # leg cable slots and battery strap slots
        for sy in (1, -1):
            s = s.cut(bx((-41.0, -36.0), (22.0, 36.0) if sy > 0 else (-36.0, -22.0), (Z0 - 1, Z0T + 1)))
            s = s.cut(bx((-14.5, 6.5), (18.5, 21.5) if sy > 0 else (-21.5, -18.5), (Z0 - 1, Z0T + 1)))
        # PCA9685 bays (SL-40, holes on the line |y| = 30 to stay clear of the leg screws)
        for side, sy in (("L", 1), ("R", -1)):
            for k, dx in enumerate((-20.0, 20.0)):
                x, y = self.PCA_C[0] + dx, sy * 30.0
                s = s.cut(cyl(1.6, (x, y, Z0 - 1), (x, y, Z0T + 1)))
                self._hole(p, f"bay_pca{side}{k + 1}", "ins_m2_thru", f"INS-M2 for sled_pca_{side} (M2x6)",
                           (x, y, Z0T), 3.2, 0)
            self.bays[f"PCA_{side}"] = ("pelvis_plate", (self.PCA_C[0], sy * self.PCA_C[1]), "x", 40, "sled_pca")
        # shell bottom ring: inserts pressed from below
        for i, (x, y) in enumerate([(26.0, 48.0), (26.0, -48.0), (-30.0, 48.0), (-30.0, -48.0), (-46.0, 25.0),
                                    (-46.0, -25.0)]):
            s = s.cut(cyl(1.6, (x, y, Z0 - 0.01), (x, y, Z0 + 4.0)))
            self._hole(p, f"shell{i + 1}", "ins_m2", "INS-M2 from below, shell bottom ring (M2x6)", (x, y, Z0), 3.2,
                       4.0, (0, 0, 1))
        return s.clean()

    # ------------------------------------------------------------------ decks 1 and 2
    DECK1_BAYS = {  # name: (centre, long axis, pitch, offset across, sled)
        "PERF": ((13.0, 0.0), "y", 40, -22.0, "sled_perf"),
        "UBEC": ((-29.5, 12.5), "y", 40, 0.0, "sled_ubec"),
        "BUCK": ((-27.0, -23.5), "x", 20, 0.0, "sled_buck"),
        "INA_L": ((8.0, 45.5), "x", 20, 0.0, "sled_ina"),
        "INA_R": ((8.0, -45.5), "x", 20, 0.0, "sled_ina"),
    }
    DECK2_BAYS = {"RADXA": ((24.5, 0.0), "y", 40, -12.5, "sled_radxa")}
    SH_FLANGE_HOLES = [(-25.0, 37.0), (-25.0, 50.0), (15.5, 37.0), (15.5, 50.0)]
    NECK_FLANGE_HOLES = [(-26.0, 12.5), (-26.0, -12.5), (2.0, 12.5), (2.0, -12.5)]

    @staticmethod
    def bay_points(c, axis, pitch, off):
        u = np.array([1.0, 0.0]) if axis == "x" else np.array([0.0, 1.0])
        v = np.array([0.0, 1.0]) if axis == "x" else np.array([1.0, 0.0])
        c = np.asarray(c, float)
        return [tuple(c + d * pitch / 2 * u + off * v) for d in (-1, 1)]

    def _deck(self, p, z):
        zt = z + TD
        s = bx(DX, DY, (z, zt))
        for i, (x, y) in enumerate(SO_XY):
            s = s.cut(cyl(1.65, (x, y, z - 1), (x, y, zt + 1)))
            what = "SO35 male thread down into SO40" if p == "deck_1" else "M3x6 from the top into SO35"
            self._hole(p, f"so{i + 1}", "clr_so", what, (x, y, zt), 3.3, 0)
        s = s.cut(bx((-50.0, -45.0), (-10.0, 10.0), (z - 1, zt + 1)))            # back cable notch
        bays = self.DECK1_BAYS if p == "deck_1" else self.DECK2_BAYS
        for name, (c, axis, pitch, off, sled) in bays.items():
            for k, (x, y) in enumerate(self.bay_points(c, axis, pitch, off)):
                s = s.cut(cyl(1.6, (x, y, z - 1), (x, y, zt + 1)))
                self._hole(p, f"bay_{name.lower()}{k + 1}", "ins_m2_thru", f"INS-M2 for {sled} (M2x6)", (x, y, zt),
                           3.2, 0)
            self.bays[name] = (p, c, axis, pitch, sled)
        if p == "deck_2":
            for k, (x, y) in enumerate(self.SH_FLANGE_HOLES + [(x, -y) for x, y in self.SH_FLANGE_HOLES]):
                s = s.cut(cyl(1.6, (x, y, z - 1), (x, y, zt + 1)))
                side = "L" if y > 0 else "R"
                self._hole(p, f"shoulder{side}{k % 4 + 1}", "ins_m2_thru", f"INS-M2 for shoulder_mount {side} (M2x6)",
                           (x, y, zt), 3.2, 0)
            for k, (x, y) in enumerate(self.NECK_FLANGE_HOLES):
                s = s.cut(cyl(1.6, (x, y, z - 1), (x, y, zt + 1)))
                self._hole(p, f"neck{k + 1}", "ins_m2_thru", "INS-M2 for neck_mount (M2x6)", (x, y, zt), 3.2, 0)
            for i, (x, y) in enumerate([(26.0, 51.0), (26.0, -51.0), (-32.0, 51.0), (-32.0, -51.0), (-46.0, 25.0),
                                        (-46.0, -25.0)]):
                s = s.cut(cyl(1.6, (x, y, z - 1), (x, y, zt + 1)))
                self._hole(p, f"shell{i + 1}", "ins_m2_thru", "INS-M2, shell top ring (M2x6)", (x, y, zt), 3.2, 0)
        return s.clean()

    # ------------------------------------------------------------------ sleds
    SLEDS = {  # sled: (board, length along u, width across v, attach offset across, tie-mounted)
        "sled_pca": ("PCA", 64.0, 29.0, -6.5, False),
        "sled_perf": ("PERF", 72.0, 52.0, -22.0, False),
        "sled_radxa": ("RADXA", 67.0, 31.0, -12.5, False),
        "sled_ubec": ("UBEC", 49.0, 29.0, 0.0, True),
        "sled_buck": ("BUCK", 26.0, 21.0, 0.0, True),
        "sled_ina": ("INA", 25.0, 17.0, 0.0, True),
    }

    def _sleds(self):
        """Each sled is built where it sits (the left one where there are two); its drawing uses the bay centre."""
        placed = {"sled_pca": "PCA_L", "sled_perf": "PERF", "sled_radxa": "RADXA", "sled_ubec": "UBEC",
                  "sled_buck": "BUCK", "sled_ina": "INA_L"}
        for sled, bay in placed.items():
            self.parts[sled] = self._sled(sled, bay)
        for bay in ("PCA_R", "INA_R"):                          # the right ones: board envelopes only
            self._sled(self.bays[bay][4], bay, envelope_only=True)

    def _sled(self, sled, bay, envelope_only=False):
        deck, c, axis, pitch, _ = self.bays[bay]
        code, L, W, off, tie = self.SLEDS[sled]
        b = el.BOARDS[code]
        mirror = c[1] < 0 and bay.endswith("_R")
        zt = Z0T if deck == "pelvis_plate" else (Z1T if deck == "deck_1" else Z2T)
        u = np.array([1.0, 0.0, 0.0]) if axis == "x" else np.array([0.0, 1.0, 0.0])
        v = np.array([0.0, 1.0, 0.0]) if axis == "x" else np.array([1.0, 0.0, 0.0])
        if mirror:
            v = -v if axis == "x" else v
            off_v = off
        else:
            off_v = off
        C = np.array([c[0], c[1], zt])

        def at(du, dv, dz=0.0):
            return C + du * u + dv * v + np.array([0, 0, dz])

        def rect(u0, u1, v0, v1, z0, z1):
            p0, p1 = at(u0, v0), at(u1, v1)
            return bx(sorted((p0[0], p1[0])), sorted((p0[1], p1[1])), (zt + z0, zt + z1))

        # board envelope above the sled
        bz0 = SLED_T + (0.0 if tie else BOSS_H) + (1.0 if tie else 0.0)
        env = rect(-b.size[0] / 2, b.size[0] / 2, -b.size[1] / 2, b.size[1] / 2, bz0 - (0 if tie else b.bottom * 0),
                   bz0 + b.pcb + b.top)
        self.boards[f"{code}_{bay}"] = env
        if envelope_only:
            return None
        s = rect(-L / 2, L / 2, -W / 2, W / 2, 0.0, SLED_T)
        p = sled
        for k, du in enumerate((-pitch / 2, pitch / 2)):
            q = at(du, off_v)
            s = s.cut(cyl(1.15, q + [0, 0, -1], q + [0, 0, SLED_T + 1])).cut(
                cyl(2.2, q + [0, 0, SLED_T - 2.2], q + [0, 0, SLED_T + 1]))
            self._hole(p, f"attach{k + 1}", "cb_m2", f"M2x6 into the {deck} bay {bay}", q + [0, 0, SLED_T], 2.3, 0,
                       joint=bay)
        if not tie:
            for k, (hu, hv) in enumerate(b.holes):
                q = at(hu, hv, SLED_T)
                s = s.fuse(cyl(3.0, q, q + [0, 0, BOSS_H]))
                s = s.cut(cyl(1.6, q + [0, 0, BOSS_H - 5.0], q + [0, 0, BOSS_H + 1]))
                self._hole(p, f"boss{k + 1}", "boss_m2", f"INS-M2 in the boss, M2x6 through the {code} board",
                           q + [0, 0, BOSS_H], 3.2, 5.0, joint=bay)
            if code == "PERF":     # wires under the perfboard: keep the attach screws in the open
                pass
        else:
            # two ties around the module, slots just outside its long sides, and four corner stops 2 high
            for k, du in enumerate((-b.size[0] / 2 + 6.0, b.size[0] / 2 - 6.0)):
                for sv in (-1, 1):
                    q = at(du, sv * (b.size[1] / 2 + 1.6))
                    s = s.cut(bx((q[0] - (1.5 if axis == "x" else 0.8), q[0] + (1.5 if axis == "x" else 0.8)),
                                 (q[1] - (0.8 if axis == "x" else 1.5), q[1] + (0.8 if axis == "x" else 1.5)),
                                 (zt - 1, zt + SLED_T + 1)))
                    self._hole(p, f"tie{k + 1}{'ab'[sv > 0]}", "tie", "cable tie around the module",
                               q + [0, 0, SLED_T], 1.6, 0, joint=bay)
            for su in (-1, 1):
                for sv in (-1, 1):
                    q0 = (su * (b.size[0] / 2 + 0.5), su * (b.size[0] / 2 + 2.0))
                    s = s.fuse(rect(min(q0), max(q0), sv * (b.size[1] / 2 - 4.0), sv * (b.size[1] / 2 + 0.5),
                                    SLED_T, SLED_T + 2.0) if sv > 0 else
                               rect(min(q0), max(q0), sv * (b.size[1] / 2 + 0.5), sv * (b.size[1] / 2 - 4.0),
                                    SLED_T, SLED_T + 2.0))
        return s.clean()

    # ------------------------------------------------------------------ shoulder (left)
    def _shoulder_mount(self):
        j = self.joints["shoulder_pitch"]
        f = bx((-28.0, 19.0), (34.0, 54.0), (Z2T, Z2T + FLANGE_T))
        s = fuse(j.holder_add(), f)
        for k, (x, y) in enumerate(self.SH_FLANGE_HOLES):
            s = s.cut(cyl(1.15, (x, y, Z2T - 1), (x, y, Z2T + FLANGE_T + 1)))
            self._hole("shoulder_mount", f"flange{k + 1}", "clr_m2", "M2x6 into deck_2 (head on the flange)",
                       (x, y, Z2T + FLANGE_T), 2.3, 0)
        return s

    def _shoulder_link(self):
        jp, jr = self.joints["shoulder_pitch"], self.joints["shoulder_roll"]
        y0 = SH_CT_Y + J.HP[0]
        web = bx((-12.0, 2.7), (y0, ROLL_Y - 8.3), (ROLL_Z + 2.0, SH_Z - 4.0))
        return fuse(jp.horn_disc(), jr.holder_add(), web)

    def _upper_arm(self):
        jr, je = self.joints["shoulder_roll"], self.joints["elbow"]
        hp = sorted(p[0] for p in jr.a_range(J.HP))
        bar = bx((hp[0], hp[1]), (ROLL_Y - 3.5, ROLL_Y + 10.0), (ELB_Z - 18.0, ROLL_Z))
        web = bx((8.3 - 0.01, hp[0] + 0.01), (ROLL_Y - 3.5, ROLL_Y + 10.0), (ELB_Z - 18.0, ELB_Z + 7.0))
        return fuse(jr.horn_disc(), je.holder_add(), bar, web)

    def _forearm(self):
        je = self.joints["elbow"]
        y = sorted(p[1] for p in je.a_range(J.HP))
        bar = bx((-6.0, 6.0), (y[0], y[1]), (ELB_Z - 42.0, ELB_Z))
        mitten = cq.Workplane().box(18.0, 10.0, 30.0).edges().fillet(3.0).val() \
            .moved(cq.Location(cq.Vector(0.0, y[0] + 3.5, ELB_Z - 55.0)))
        thumb = cyl(3.5, (9.0, y[0] + 3.5, ELB_Z - 50.0), (12.0, y[0] + 3.5, ELB_Z - 50.0))
        return fuse(je.horn_disc(), bar, mitten, thumb)

    # ------------------------------------------------------------------ neck and head
    def _neck_mount(self):
        j = self.joints["neck_yaw"]
        f = bx((-30.0, 6.0), (-16.0, 16.0), (Z2T, Z2T + FLANGE_T))
        s = fuse(j.holder_add(), f)
        for k, (x, y) in enumerate(self.NECK_FLANGE_HOLES):
            s = s.cut(cyl(1.15, (x, y, Z2T - 1), (x, y, Z2T + FLANGE_T + 1)))
            self._hole("neck_mount", f"flange{k + 1}", "clr_m2", "M2x6 into deck_2 (head on the flange)",
                       (x, y, Z2T + FLANGE_T), 2.3, 0)
        return s

    def _neck_link(self):
        jy, jp = self.joints["neck_yaw"], self.joints["head_pitch"]
        return fuse(jy.horn_disc(), jp.holder_add())

    HEAD_X = (-27.0, 27.0)
    HEAD_Y = (-24.0, 24.0)

    def _head_frame(self):
        jp = self.joints["head_pitch"]
        hp = sorted(p[1] for p in jp.a_range(J.HP))
        ip = sorted(p[1] for p in jp.a_range(J.IP))
        hd = HEAD_DECK
        bars = [bx((NECK_X - 6.0, NECK_X + 6.0), (hp[0], hp[1]), (HEAD_Z, hd)),
                bx((NECK_X - 6.0, NECK_X + 6.0), (ip[0], ip[1]), (HEAD_Z, hd))]
        deck = bx(self.HEAD_X, self.HEAD_Y, (hd, hd + 3.0))
        face = bx((self.HEAD_X[1] - 3.0, self.HEAD_X[1]), self.HEAD_Y, (hd - 26.0, hd))
        s = fuse(jp.horn_disc(), jp.idler_disc(), *bars, deck, face)
        # camera mount bay (SL-20 across y) and display ties on the face plate
        for k, y in enumerate((-10.0, 10.0)):
            x = 12.0
            s = s.cut(cyl(1.6, (x, y, hd - 1), (x, y, hd + 4)))
            self._hole("head_frame", f"bay_cam{k + 1}", "ins_m2_thru", "INS-M2 for cam_mount (M2x6)", (x, y, hd + 3.0),
                       3.2, 0)
        for k, (y, z) in enumerate([(-27.0, hd - 6.0), (27.0, hd - 6.0), (-27.0, hd - 20.0), (27.0, hd - 20.0)]):
            yy = np.sign(y) * 21.0
            s = s.cut(bx((self.HEAD_X[1] - 4, self.HEAD_X[1] + 1), (yy - 0.8, yy + 0.8), (z - 1.5, z + 1.5)))
            self._hole("head_frame", f"disp_tie{k + 1}", "tie", "cable tie over the display module",
                       (self.HEAD_X[1], yy, z), 1.6, 0, (-1, 0, 0))
        # shell head: 4 inserts in the deck corners, from the top
        for k, (x, y) in enumerate([(-23.0, 20.0), (-23.0, -20.0), (2.0, 20.0), (2.0, -20.0)]):
            s = s.cut(cyl(1.6, (x, y, hd - 1), (x, y, hd + 4)))
            self._hole("head_frame", f"shell{k + 1}", "ins_m2_thru", "INS-M2, head shell (M2x6)", (x, y, hd + 3.0),
                       3.2, 0)
        # display envelope behind the visor (front face, outside), camera and ToF on cam_mount
        d = el.BOARDS["DISP"]
        self.boards["DISP"] = bx((self.HEAD_X[1] + 1.0, self.HEAD_X[1] + 1.0 + d.top + 1.5),
                                 (-d.size[0] / 2, d.size[0] / 2), (hd - 26.0, hd - 26.0 + d.size[1]))
        return s

    def _cam_mount(self):
        """Wedge: base on the head deck (SL-20), front face tilted 15° down for the camera, ToF below it."""
        hd = HEAD_DECK + 3.0
        t = np.radians(15.0)
        n = np.array([np.cos(t), 0, -np.sin(t)])     # face normal, looking forward and down
        up = np.array([np.sin(t), 0, np.cos(t)])
        base = bx((6.0, 24.0), (-16.0, 16.0), (hd, hd + 3.0))
        o = np.array([22.0, 0.0, hd])                  # bottom edge of the face, inner side
        pts = [o, o + 3.0 * n, o + 3.0 * n + 34.0 * up, o + 34.0 * up]
        wp = cq.Workplane("XZ").polyline([(p[0], p[2]) for p in pts]).close().extrude(16.0, both=True)
        s = base.fuse(wp.val())
        rib = cq.Workplane("XZ").polyline([(6.0, hd + 3.0), (22.0, hd + 3.0), (22.0 + 34 * up[0], hd + 34 * up[2]),
                                           (19.0 + 34 * up[0] * 0.6, hd + 34 * up[2] * 0.6)]).close() \
            .extrude(1.5, both=True).val()
        s = s.fuse(rib.moved(cq.Location(cq.Vector(0, 14.0, 0)))).fuse(rib.moved(cq.Location(cq.Vector(0, -14.0, 0))))
        for k, y in enumerate((-10.0, 10.0)):
            q = np.array([12.0, y, hd])
            s = s.cut(cyl(1.15, q - [0, 0, 1], q + [0, 0, 4])).cut(cyl(2.2, q + [0, 0, 0.8], q + [0, 0, 4]))
            self._hole("cam_mount", f"attach{k + 1}", "cb_m2", "M2x6 into the head deck", q + [0, 0, 3.0], 2.3, 0)
        face0 = o + 3.0 * n
        cam = el.BOARDS["CAM"]
        cc = face0 + 24.0 * up
        for k, (hy, hz) in enumerate(cam.holes):
            q = cc + hy * np.array([0, 1.0, 0]) + hz * up
            s = s.fuse(cyl(2.5, q, q + 2.5 * n)).cut(cyl(1.6, q - 2.5 * n, q + 3.0 * n))
            self._hole("cam_mount", f"cam{k + 1}", "boss_m2", "INS-M2 (pressed from the front), M2x6 through the "
                       "camera", tuple(q + 2.5 * n), 3.2, 5.0, tuple(np.round(-n, 4)))
        s = s.cut(cyl(4.5, cc - 4 * n, cc + 4 * n))                       # lens and ribbon clearance
        tof = el.BOARDS["TOF"]
        tc = face0 + 7.0 * up
        for k, (hy, hz) in enumerate(tof.holes):
            q = tc + hy * np.array([0, 1.0, 0]) + hz * up
            s = s.fuse(cyl(2.5, q, q + 2.5 * n)).cut(cyl(1.6, q - 2.5 * n, q + 3.0 * n))
            self._hole("cam_mount", f"tof{k + 1}", "boss_m2", "INS-M2 (pressed from the front), M2x6 through the "
                       "ToF board", tuple(q + 2.5 * n), 3.2, 5.0, tuple(np.round(-n, 4)))
        return s.clean()

    # ------------------------------------------------------------------ bought parts placed
    def servos(self):
        return {f"servo_{n}": j.servo_solid() for n, j in self.joints.items()}

    def horns(self):
        return {f"horn_{n}": j.horn_solid() for n, j in self.joints.items()}

    def standoffs(self):
        out = {}
        for i, (x, y) in enumerate(SO_XY):
            out[f"so40_{i + 1}"] = cq.Solid.makeCylinder(3.17, SO_A, cq.Vector(x, y, Z0T))
            out[f"so35_{i + 1}"] = cq.Solid.makeCylinder(3.17, SO_B, cq.Vector(x, y, Z1T))
        return out

    def battery(self):
        b = el.BOARDS["BATT"]
        c = self.BATT_C
        return bx((c[0] - b.size[0] / 2, c[0] + b.size[0] / 2), (-b.size[1] / 2, b.size[1] / 2),
                  (Z0T + 0.01, Z0T + b.top))

    def summary(self):
        return dict(Z0=Z0, Z0T=round(Z0T, 2), Z1=round(Z1, 2), Z1T=round(Z1T, 2), Z2=round(Z2, 2), Z2T=round(Z2T, 2),
                    shoulder_axis_z=round(SH_Z, 2), roll_axis=(ROLL_CT_X, round(ROLL_Y, 2), round(ROLL_Z, 2)),
                    elbow_z=round(ELB_Z, 2), neck_case_top=round(NECK_CT_Z, 2), head_pitch_z=round(HEAD_Z, 2),
                    head_deck=round(HEAD_DECK, 2))


if __name__ == "__main__":
    b = Body()
    print(b.summary())
    for n, p in b.parts.items():
        bb = p.BoundingBox()
        print(f"{n:15s} vol {p.Volume() / 1000:6.2f} cm3  x {bb.xmin:6.1f}..{bb.xmax:6.1f}"
              f"  y {bb.ymin:6.1f}..{bb.ymax:6.1f}"
              f"  z {bb.zmin:6.1f}..{bb.zmax:6.1f}  valid {p.isValid()}")
    print(len(b.holes()), "holes")
