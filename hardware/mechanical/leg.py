"""Mico leg v3: six servo joints, six printed links, every hole placed.

Leg frame (left leg; the right leg is its mirror in y): origin at the hip point H on the hip-pitch axis, x forward,
y outboard, z up, legs straight. Decisions D1 (wide legs) and D2B (hip and ankle roll axes offset from the pitch
axes) give:

    hip roll axis    HIP_ROLL_UP above H
    knee             L1 below H
    ankle pitch      L1 + L2 below H (point A)
    ankle roll axis  ANKLE_ROLL_DOWN below A
    hip yaw horn     above the hip block, high enough that the block swings ±HIP_ROLL_MAX inside the bracket
    sole             below the ankle block, low enough that it swings ±ANKLE_ROLL_MAX above the sole plate

Links and who carries which servo:
    yaw_bracket  driven by hip yaw (servo in the pelvis); carries nothing
    hip_block    carries hip roll and hip pitch; driven by hip roll through the yaw bracket
    thigh        driven by hip pitch; carries the knee servo
    shin         driven by the knee and by ankle pitch: a frame with no servo, sides closed (see _shin_closed)
    ankle_block  carries ankle pitch and ankle roll
    foot         driven by ankle roll
"""
import cadquery as cq
import numpy as np

import hardware as hw
import joint as J

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
NX, NY, NZ = (-1, 0, 0), (0, -1, 0), (0, 0, -1)

# Every leg joint uses the pocket of the servo hardware.JOINT_SERVO puts in the leg; the layout below is derived
# from that pocket (v3: 9 g MG92B, v4: DS3218). Pocket outer faces from the joint axis:
LEG_SERVO = hw.SERVOS[hw.JOINT_SERVO["knee"]]
LP = J.pocket(LEG_SERVO)
PW = round(LEG_SERVO.width / 2 + hw.CLEAR_SERVO + hw.WALL, 2)   # across the case (w), each side
PB0 = round(-LP.CB0 + hw.CLEAR_SERVO + hw.WALL, 2)        # along the case (b), shaft side
PB1 = round(LP.CB + hw.CLEAR_SERVO + hw.WALL, 2)          # along the case (b), far end
EAR_ROOM = 3.6           # pitch pocket wall to roll pocket wall: room for the ear screws
V4 = LEG_SERVO.size == "standard"
HIP_ROLL_UP = round(PB0 + PW + EAR_ROOM, 2)   # D2B: hip roll axis above the hip-pitch axis (v3 20.0)
ANKLE_ROLL_DOWN = HIP_ROLL_UP          # D2B: ankle roll axis below the ankle-pitch axis
L1 = 105.0 if V4 else 48.0             # H -> knee (v3 48, chosen by check_mujoco: see docs/mechanical)
L2 = L1                                # knee -> A
# roll servos: v3 hangs them inboard (-y); the DS3218 is too long for that (the two hip blocks would meet on the
# centre line), so v4 turns both roll servos outboard
ROLL_B = Y if V4 else NY
HIP_ROLL_MAX = 20.0      # deg, mechanical range used for clearances (gait uses ±17)
ANKLE_ROLL_MAX = 20.0
SOLE_PLATE = 3.0
TPU_SOLE = 3.0
# foot outline under the ankle axes: v3 76 x 38 centred; v4 covers the outboard ankle-roll servo and keeps the
# inner edge 10 mm inside the pocket face
SOLE = (dict(length=96.0, width=PB0 + PB1 + 12.0, x=0.0, y=(PB1 - PB0 - 8.0) / 2) if V4
        else dict(length=76.0, width=38.0, x=0.0, y=0.0))
# FSR 402 (Interlink datasheet, ASSUMED): disc Ø18.28, 0.46-0.55 thick, tail 56 long; toe and heel under the plate,
# tails out along +y (outboard) to the leg harness; a TPU puck Ø10 x 0.3 on the sole presses the active area
FSR_X = (SOLE["length"] / 2 - 16.0, -(SOLE["length"] / 2 - 16.0))
FSR_POCKET = (18.8, 0.6)       # diameter, depth in the plate underside
FSR_TAIL = (8.4, 0.6)          # channel width, depth
FSR_PUCK = (10.0, 0.3)
SPINE = 3.6              # structural spine thickness
HIP_Y = round(LP.HALF + 5.6, 2)   # leg centre lines at y = ±HIP_Y on the pelvis (v3 28.0): legs 11 mm apart
YAW_MAX = 45.0
RING_R = (13.0, 21.0)    # yaw thrust ring: carries the robot's weight past the yaw servo
# idler gap wide enough to take the driven link off the spline; the servo side moves out, the idler stays
WIDE = dict(gap=J.LEG_GAP, keep_idler=True, servo=LEG_SERVO)
ARM = 4.0                # side-plate arm thickness
# closed shin (default): the side outline below, printed as 4 mm webs on the outer faces.
# hybrid shin (frame B, only if the printed one fails test T4): two cut metal side plates on the outer faces, a
# printed PETG core (the four joint discs,
# 4 mm webs behind the plates, the front spine). The plates wrap the discs (bearing on the disc rims) and are held
# by countersunk M2 screws into inserts in the webs. Outline stays inside the old silhouette above the knee axis
# (thigh arms there) and above the ankle axis on the idler side (ankle roll parts below).
SHIN_WEB = 2.4           # closed shin side web: 6 perimeters (solid); FEA peak 8.2 MPa of 12, +2 g over the open shin
PLATE_T = 1.5            # 5052-H32 aluminium or 3K carbon, same DXF (FEA: 2.0 buys nothing)
PLATE_RING = {"h": LP.HORN_R + 3.0, "i": LP.IDLER_R + 3.0}   # plate outer radius round the horn / idler discs
PLATE_HOLE = {"h": LP.HORN_R + 0.15, "i": LP.IDLER_R + 0.15}
PLATE_SCREWS = [(3.5, -L1 - PB0 - 9.9), (10.0, -L1 - PB0 - 9.9), (-4.0, -L1 - L2 + PB0 + 8.9),
                (6.0, -L1 - L2 + PB0 + 8.9)]  # (x, z)
SHIN_FRONT = round(LP.HORN_R + 1.8, 2)
PLATE_FILLET = 3.0
PLATE_BACK = round(PB1 + 10.9, 2)  # shin plate back edge starts this far below the knee (the thigh folds in above)
WEB_Y = round(3.6 + hw.CLEAR_SERVO + LEG_SERVO.tab_low, 2)   # block webs: idler wall face up to the ear underside

K = -L1
A = -(L1 + L2)
AR = A - ANKLE_ROLL_DOWN


def bx(x, y, z):
    sz = [hi - lo for lo, hi in (x, y, z)]
    return cq.Solid.makeBox(*sz, cq.Vector(x[0], y[0], z[0]))


def rotated(shape, axis_pt, axis, deg):
    return shape.rotate(cq.Vector(*axis_pt), cq.Vector(*np.add(axis_pt, axis)), deg)


def fuse(*shapes):
    s = shapes[0]
    for t in shapes[1:]:
        s = s.fuse(t)
    return s.clean()


class Leg:
    def __init__(self, shin="closed"):
        self.shin_frame = shin
        self.joints = {
            # gap = LEG_GAP: the driven link comes off the spline along the shaft (N3 review G-2)
            "hip_roll": J.Joint("hip_roll", "hip_block", "yaw_bracket", (0, 0, HIP_ROLL_UP), X, ROLL_B, **WIDE),
            "hip_pitch": J.Joint("hip_pitch", "hip_block", "thigh", (0, 0, 0), Y, NZ, **WIDE),
            "knee": J.Joint("knee", "thigh", "shin", (0, 0, K), Y, Z, **WIDE),
            "ankle_pitch": J.Joint("ankle_pitch", "ankle_block", "shin", (0, 0, A), Y, Z, **WIDE),
            "ankle_roll": J.Joint("ankle_roll", "ankle_block", "foot", (0, 0, AR), X, ROLL_B, **WIDE),
        }
        self.parts = {}
        self.extra_holes = []
        hb = self._hip_block_body()
        self.yaw_bottom = self._swept_top(hb, (0, 0, HIP_ROLL_UP), X, HIP_ROLL_MAX) + hw.GAP_MOVING
        # yaw: the bracket's top plate is the yaw horn plate, its outer (cap) face looking down at the hip block
        oz = self.yaw_bottom + J.HP[1] - J.CENTER
        self.joints["hip_yaw"] = J.Joint("hip_yaw", "hip_mount", "yaw_bracket", (0, 0, oz), NZ, NX, idler=False)
        ab = self._ankle_block_body()
        self.sole_top = self._swept_bottom(ab, (0, 0, AR), X, ANKLE_ROLL_MAX) - hw.GAP_MOVING
        self.floor = self.sole_top - SOLE_PLATE - TPU_SOLE
        # NTC bead on the hip-roll servo case (obs[57..58]): hole through the upper pocket wall
        hb = hb.cut(self.joints["hip_roll"].ntc_cut("hip_block", side=-1))
        self.parts["hip_block"] = self._finish("hip_block", hb)
        self.parts["ankle_block"] = self._finish("ankle_block", ab)
        self.parts["yaw_bracket"] = self._finish("yaw_bracket", self._yaw_bracket_body())
        self.parts["thigh"] = self._finish("thigh", self._thigh_body())
        if shin == "hybrid":
            core, plates = self._shin_hybrid()
            self.parts["shin"] = self._finish("shin", core)
            self.parts.update(plates)
        elif shin == "closed":
            self.parts["shin"] = self._finish("shin", self._shin_closed())
        else:                                   # "open": the first shin, kept for comparison (fea.py)
            self.parts["shin"] = self._finish("shin", self._shin_body())
        self.parts["foot"] = self._finish("foot", self._foot_body())
        self.parts["sole"] = self._sole()
        self.parts["hip_mount"] = self._finish("hip_mount", self._hip_mount_body())
        self.parts["thrust_ring"] = self._thrust_ring()
        self._tie_slots()

    # ------------------------------------------------------------ sweeps
    @staticmethod
    def _swept_top(shape, pt, axis, deg):
        return max(rotated(shape, pt, axis, d).BoundingBox().zmax for d in np.linspace(-deg, deg, 9))

    @staticmethod
    def _swept_bottom(shape, pt, axis, deg):
        return min(rotated(shape, pt, axis, d).BoundingBox().zmin for d in np.linspace(-deg, deg, 9))

    # ------------------------------------------------------------ finishing: cut every joint feature of a part
    def _finish(self, name, body):
        for j in self.joints.values():
            if j.holder == name:
                body = body.cut(j.holder_cut(name))
            if j.driven == name:
                body = body.cut(j.driven_cut(name))
        return body.clean()

    def holes(self):
        out = [h for j in self.joints.values() for h in j.holes] + self.extra_holes
        return sorted(out, key=lambda h: (h.part, h.id))

    # ------------------------------------------------------------ hip block: hip roll pocket over hip pitch pocket
    def _hip_block_body(self):
        jr, jp = self.joints["hip_roll"], self.joints["hip_pitch"]
        iw_y = min(p[1] for p in jp.a_range(LP.IW))    # web follows the hip pitch pocket
        web = bx((-PW, round(PW * 0.53, 1)), (iw_y, iw_y + WEB_Y), (PB0 - 0.1, HIP_ROLL_UP - PW + 0.1))
        return fuse(jr.holder_add(), jp.holder_add(), web)

    # ------------------------------------------------------------ ankle block: ankle pitch over ankle roll pocket
    def _ankle_block_body(self):
        jp, jr = self.joints["ankle_pitch"], self.joints["ankle_roll"]
        iw_y = min(p[1] for p in jp.a_range(LP.IW))    # web follows the ankle pitch pocket
        web = bx((-PW, round(PW * 0.53, 1)), (iw_y, iw_y + WEB_Y), (AR + PW - 0.1, A - PB0 + 0.1))
        return fuse(jp.holder_add(), jr.holder_add(), web)

    # ------------------------------------------------------------ yaw bracket: U around the hip block
    def _yaw_bracket_body(self):
        jy, jr = self.joints["hip_yaw"], self.joints["hip_roll"]
        zb = self.yaw_bottom
        hp_x = sorted(p[0] for p in jr.a_range(LP.HP))          # horn plate (front) x range
        ip_x = sorted(p[0] for p in jr.a_range(jr.IP))           # idler plate (back) x range
        top_t = J.HP[1] - J.HP[0]
        top = fuse(bx((ip_x[0], hp_x[1]), (-9.0, 9.0), (zb, zb + top_t)),
                   cq.Solid.makeCylinder(RING_R[1], top_t, cq.Vector(0, 0, zb)))     # thrust face for the ring
        front = bx((hp_x[1] - ARM, hp_x[1]), (-LP.HORN_R, LP.HORN_R), (HIP_ROLL_UP, zb + top_t))
        back = bx((ip_x[0], ip_x[0] + ARM), (-LP.IDLER_R, LP.IDLER_R), (HIP_ROLL_UP, zb + top_t))
        return fuse(top, front, back, jy.horn_disc(), jr.horn_disc(), jr.idler_disc())

    # ------------------------------------------------------------ thigh: hip-pitch plates, back spine, knee pocket
    def _thigh_body(self):
        jp, jk = self.joints["hip_pitch"], self.joints["knee"]
        hp_y = sorted(p[1] for p in jp.a_range(LP.HP))
        ip_y = sorted(p[1] for p in jp.a_range(jp.IP))
        spine_x = (-LP.HORN_R - SPINE / 2, -LP.HORN_R + SPINE / 2)
        z_top = -round(np.hypot(PB1, PW) + 3.7, 1)            # spine stays outside the hip block's reach from H
        # the knee servo slides in along -y: nothing on the +y side over the pocket (x -8.3..8.3, z up to K+23)
        horn_arm = fuse(bx((spine_x[0], 0.0), (hp_y[1] - ARM, hp_y[1]), (-LP.HORN_R, 0.0)),
                        bx((spine_x[0], spine_x[1]), (hp_y[1] - ARM, hp_y[1]), (K + LP.HORN_R + 2, 0.0)))
        ir = LP.IDLER_R
        idler_arm = fuse(bx((-ir - 1.5, ir + 1.5), (ip_y[0], ip_y[0] + ARM), (K + ir + 2.5, 0.0)),
                         bx((spine_x[0], 0), (ip_y[0], ip_y[0] + ARM), (K + LP.IDLER_R + 2.5, 0.0)))
        spine = bx(spine_x, (ip_y[0], hp_y[1]), (K + LP.HORN_R + 2, z_top))
        iw_y = min(p[1] for p in jk.a_range(LP.IW))    # knee pocket wall: the link follows the pocket
        pocket_link = bx((spine_x[0], -PW + 2.3), (iw_y, iw_y + WEB_Y), (K - PB0, K + PB1))
        return fuse(jp.horn_disc(), jp.idler_disc(), horn_arm, idler_arm, spine, jk.holder_add(), pocket_link)

    # ------------------------------------------------------------ shin: knee and ankle plates, front spine (shin bone)
    def _shin_body(self):
        jk, ja = self.joints["knee"], self.joints["ankle_pitch"]
        hp_y = sorted(p[1] for p in jk.a_range(LP.HP))
        ip_y = sorted(p[1] for p in jk.a_range(jk.IP))
        spine_x = (LP.HORN_R - SPINE / 2, LP.HORN_R + SPINE / 2)
        z0, z1 = A + PB1 + 5.9, K - PB0 - 6.9           # clear of the ankle block's pitch pocket and the knee pocket
        plates = []
        for zc, lo in ((K, 1), (A, 0)):          # at A only the upper half: the ankle roll servo sits below
            plates += [bx((0.0, spine_x[1]), (hp_y[1] - ARM, hp_y[1]), (zc - lo * LP.HORN_R, zc + LP.HORN_R)),
                       bx((0.0, spine_x[1]), (ip_y[0], ip_y[0] + ARM), (zc - lo * LP.IDLER_R, zc + LP.IDLER_R))]
        side_h = bx(spine_x, (hp_y[1] - ARM, hp_y[1]), (A, K))
        side_i = bx(spine_x, (ip_y[0], ip_y[0] + ARM), (A, K))
        spine = bx(spine_x, (ip_y[0], hp_y[1]), (z0, z1))
        return fuse(jk.horn_disc(), jk.idler_disc(), ja.horn_disc(), ja.idler_disc(), *plates, side_h, side_i, spine)

    # ------------------------------------------------------------ hybrid shin: metal side plates + PETG core
    def _shin_side(self, y0, y1, side):
        """Outline of one side of the hybrid shin between y0 and y1 (leg frame), without the disc holes."""
        R = PLATE_RING[side]
        top = LP.HORN_R if side == "h" else LP.IDLER_R         # stay inside the old silhouette above the knee

        def cyl_y(z, r):
            return cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(0, y0, z), cq.Vector(0, 1, 0))

        # round the knee: the old disc radius behind the axis, the plate radius in front; behind and below the knee
        # the thigh folds in at full bend, so the back edge stays at x = 0 down to PLATE_BACK below the knee
        knee = fuse(cyl_y(K, top).intersect(bx((-50, 0), (y0, y1), (K - 50, K))),
                    cyl_y(K, R).intersect(bx((0, SHIN_FRONT), (y0, y1), (K - 50, K + top))))
        ankle = cyl_y(A, R) if side == "h" else cyl_y(A, R).intersect(bx((-50, 50), (y0, y1), (A, A + 50)))
        pts = [(SHIN_FRONT, K), (SHIN_FRONT, A), (-R, A), (-R, K - PLATE_BACK), (0.0, K - PLATE_BACK + 5.0),
               (0.0, K)]
        band = cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0).val()
        # concave fillet where the back edge leaves the knee disc (no sharp inside corner)
        rf = PLATE_FILLET
        zc = K - np.sqrt((top + rf) ** 2 - rf ** 2)
        fillet = bx((-rf, 0.0), (y0, y1), (zc, K - top + 1.0))
        fillet = fillet.cut(cyl_y(0, rf).moved(cq.Location(cq.Vector(-rf, 0, zc))))
        return fuse(knee, band, ankle, fillet).intersect(bx((-50, SHIN_FRONT), (y0, y1), (-300, 50)))

    def _shin_closed(self):
        """Printed shin with closed sides: the hybrid outline as 4 mm side webs on the outer faces, no metal.
        FEA (fea.py): deflection 7.8 -> 0.4 mm, peak 54 -> 8 MPa (PLA limit 12); metal only if test T4 fails."""
        jk, ja = self.joints["knee"], self.joints["ankle_pitch"]
        hp_out = max(p[1] for p in jk.a_range(LP.HP))
        ip_out = min(p[1] for p in jk.a_range(jk.IP))
        spine_x = (LP.HORN_R - SPINE / 2, LP.HORN_R + SPINE / 2)
        spine = bx(spine_x, (ip_out, hp_out), (A + PB1 + 5.9, K - PB0 - 6.9))
        return fuse(jk.horn_disc(), jk.idler_disc(), ja.horn_disc(), ja.idler_disc(), spine,
                    self._shin_side(hp_out - SHIN_WEB, hp_out, "h"), self._shin_side(ip_out, ip_out + SHIN_WEB, "i"))

    def _shin_hybrid(self):
        jk, ja = self.joints["knee"], self.joints["ankle_pitch"]
        hp_out = max(p[1] for p in jk.a_range(LP.HP))                    # horn-side outer face
        ip_out = min(p[1] for p in jk.a_range(jk.IP))                    # idler-side outer face
        layers = {"h": ((hp_out - PLATE_T, hp_out), (hp_out - PLATE_T - ARM, hp_out - PLATE_T)),
                  "i": ((ip_out, ip_out + PLATE_T), (ip_out + PLATE_T, ip_out + PLATE_T + ARM))}
        spine_x = (LP.HORN_R - SPINE / 2, LP.HORN_R + SPINE / 2)
        z0, z1 = A + PB1 + 5.9, K - PB0 - 6.9
        spine = bx(spine_x, (layers["i"][1][0], layers["h"][1][1]), (z0, z1))
        core = [jk.horn_disc(), jk.idler_disc(), ja.horn_disc(), ja.idler_disc(), spine]
        plates = {}
        ins = hw.HOLES["ins_m2"]
        for side, ((py0, py1), (wy0, wy1)) in layers.items():
            hole_r = PLATE_HOLE[side]
            holes = [cq.Solid.makeCylinder(hole_r, 60, cq.Vector(0, -30, z), cq.Vector(0, 1, 0)) for z in (K, A)]
            plate = self._shin_side(py0, py1, side)
            web = self._shin_side(wy0, wy1, side)
            for h in holes:
                plate = plate.cut(h)
            # behind the knee the outline is the disc itself: no plate sliver round the back of the hole
            plate = plate.cut(cq.Solid.makeCylinder(hole_r + 1.5, 60, cq.Vector(0, -30, K), cq.Vector(0, 1, 0))
                              .intersect(bx((-50, 0), (-30, 30), (K - 50, K + 50))))
            inward = 1 if side == "h" else -1                             # from the plate face into the web
            face = py0 if side == "h" else py1
            for k, (x, z) in enumerate(PLATE_SCREWS):
                # countersunk M2 through the plate, insert in the web (3.4 deep, 0.6 skin behind)
                plate = plate.cut(cq.Solid.makeCylinder(1.15, 60, cq.Vector(x, -30, z), cq.Vector(0, 1, 0)))
                cs_top = py1 if side == "h" else py0
                cone = cq.Solid.makeCone(2.0, 0.9, 1.1, cq.Vector(x, cs_top, z), cq.Vector(0, -inward, 0))
                plate = plate.cut(cone)
                web = web.cut(cq.Solid.makeCylinder(ins.d / 2, 3.4, cq.Vector(x, face, z), cq.Vector(0, -inward, 0)))
                self.extra_holes.append(J.Hole("shin", f"plate_{side}{k + 1}", "ins_m2",
                                               "INS-M2 + M2x5 countersunk ISO 10642 (side plate)",
                                               (x, face, z), (0, -inward, 0), ins.d, 3.4, "knee"))
            core.append(web)
            plates[f"shin_plate_{side}"] = plate.clean()
        return fuse(*core), plates

    # ------------------------------------------------------------ foot: ankle-roll uprights on the sole plate
    def _foot_body(self):
        jr = self.joints["ankle_roll"]
        hp_x = sorted(p[0] for p in jr.a_range(LP.HP))
        ip_x = sorted(p[0] for p in jr.a_range(jr.IP))
        st = self.sole_top
        sx0, sx1 = SOLE["x"] - SOLE["length"] / 2, SOLE["x"] + SOLE["length"] / 2
        sy0, sy1 = SOLE["y"] - SOLE["width"] / 2, SOLE["y"] + SOLE["width"] / 2
        plate = bx((sx0, sx1), (sy0, sy1), (st - SOLE_PLATE, st))
        front = bx((hp_x[1] - ARM, hp_x[1]), (-LP.HORN_R, LP.HORN_R), (st - 0.01, AR))
        back = bx((ip_x[0], ip_x[0] + ARM), (-LP.IDLER_R, LP.IDLER_R), (st - 0.01, AR))
        body = fuse(plate, front, back, jr.horn_disc(), jr.idler_disc())
        # sole: 4 × M2 inserts from below, at the corners
        spec = hw.HOLES["ins_m2"]
        for i, (px, py) in enumerate([(sx0 + 7, sy0 + 7), (sx1 - 7, sy0 + 7), (sx0 + 7, sy1 - 7), (sx1 - 7, sy1 - 7)]):
            z0 = st - SOLE_PLATE
            body = body.cut(cq.Solid.makeCylinder(spec.d / 2, spec.depth - 1.0, cq.Vector(px, py, z0)))
            self.extra_holes.append(J.Hole("foot", f"sole.ins{i + 1}", "ins_m2", "INS-M2 + M2x4 (TPU sole)",
                                           (px, py, round(z0, 3)), (0, 0, 1), spec.d, spec.depth - 1.0, "sole"))
        # FSR pockets and tail channels in the plate underside
        z0 = st - SOLE_PLATE
        for k, x in enumerate(FSR_X):
            body = body.cut(cq.Solid.makeCylinder(FSR_POCKET[0] / 2, FSR_POCKET[1], cq.Vector(x, 0, z0 - 0.01)))
            body = body.cut(bx((x - FSR_TAIL[0] / 2, x + FSR_TAIL[0] / 2), (0.0, sy1 + 1.0),
                               (z0 - 0.01, z0 + FSR_TAIL[1])))
            self.extra_holes.append(J.Hole("foot", f"fsr{k + 1}", "fsr",
                                           "FSR 402 pocket, tail out to +y (obs[50..53])", (x, 0.0, round(z0, 3)),
                                           (0, 0, 1), FSR_POCKET[0], FSR_POCKET[1], "sole"))
        return body

    def _sole(self):
        sx0, sx1 = SOLE["x"] - SOLE["length"] / 2, SOLE["x"] + SOLE["length"] / 2
        sy0, sy1 = SOLE["y"] - SOLE["width"] / 2, SOLE["y"] + SOLE["width"] / 2
        z0 = self.sole_top - SOLE_PLATE - TPU_SOLE
        s = bx((sx0, sx1), (sy0, sy1), (z0, z0 + TPU_SOLE))
        for i, (px, py) in enumerate([(sx0 + 7, sy0 + 7), (sx1 - 7, sy0 + 7), (sx0 + 7, sy1 - 7), (sx1 - 7, sy1 - 7)]):
            s = s.cut(cq.Solid.makeCylinder(1.15, TPU_SOLE + 1, cq.Vector(px, py, z0 - 0.5)))
            s = s.cut(cq.Solid.makeCylinder(2.1, 2.0, cq.Vector(px, py, z0 - 0.01)))
            self.extra_holes.append(J.Hole("sole", f"sole.screw{i + 1}", "clr_m2", "M2x4 from below, head sunk 2",
                                           (px, py, round(z0, 3)), (0, 0, 1), 2.3, 0, "sole"))
        for x in FSR_X:                                  # actuator pucks under the FSR active areas
            s = s.fuse(cq.Solid.makeCylinder(FSR_PUCK[0] / 2, FSR_PUCK[1], cq.Vector(x, 0, z0 + TPU_SOLE)))
        return s

    # ------------------------------------------------------------ hip mount: yaw servo pocket, bolts to the pelvis
    def yaw_case_top(self):
        return float(self.joints["hip_yaw"].p[2])

    def _hip_mount_body(self):
        jy = self.joints["hip_yaw"]
        ct = self.yaw_case_top()
        top = ct - J.IW[0]                              # idler wall of the yaw pocket is the top face
        flange = bx((-24.0, 14.0), (-13.0, 13.0), (top - 3.0, top))
        skirts = [bx((-9.0, 1.0), (sy * 9.0, sy * 18.0) if sy > 0 else (-18.0, -9.0), (ct - 0.1, top))
                  for sy in (1, -1)]
        body = fuse(jy.holder_add(), flange, *skirts)
        spec = hw.HOLES["ins_m2"]
        for i, (px, py) in enumerate([(-20.0, -10.0), (-20.0, 10.0), (10.0, -10.0), (10.0, 10.0)]):
            body = body.cut(cq.Solid.makeCylinder(spec.d / 2, spec.depth, cq.Vector(px, py, top - spec.depth)))
            self.extra_holes.append(J.Hole("hip_mount", f"pelvis.ins{i + 1}", "ins_m2",
                                           "INS-M2 + M2x6 from the pelvis floor (leg module)", (px, py, round(top, 3)),
                                           (0, 0, -1), spec.d, spec.depth, "pelvis"))
        for i, sy in enumerate((-1, 1)):
            px, py = -4.0, sy * 14.5
            body = body.cut(cq.Solid.makeCylinder(spec.d / 2, spec.depth, cq.Vector(px, py, ct - 0.1)))
            self.extra_holes.append(J.Hole("hip_mount", f"ring.ins{i + 1}", "ins_m2", "INS-M2 + M2x6 (thrust ring)",
                                           (px, py, round(ct - 0.1, 3)), (0, 0, 1), spec.d, spec.depth, "thrust_ring"))
        # cable window at the back for the yaw lead, kept below the far ear's insert (J.TAB_A, 1.6 of wall under it)
        body = body.cut(bx((-25.0, -19.5), (-6.0, 6.0), (ct - J.TAB_A[0] + hw.HOLES["ins_m2"].depth + 1.6, top - 3.5)))
        return body

    def _thrust_ring(self):
        ct = self.yaw_case_top()
        z0 = self.yaw_bottom + (J.HP[1] - J.HP[0]) + 0.2       # 0.2 above the bracket's top face
        r = cq.Solid.makeCylinder(RING_R[1], ct - 0.1 - z0, cq.Vector(0, 0, z0))
        r = r.cut(cq.Solid.makeCylinder(RING_R[0], 40, cq.Vector(0, 0, z0 - 1)))
        for sy in (-1, 1):
            r = r.fuse(bx((-9.0, 1.0), (9.0, 18.0) if sy > 0 else (-18.0, -9.0), (z0, ct - 0.1)))
        for i, sy in enumerate((-1, 1)):
            px, py = -4.0, sy * 14.5
            r = r.cut(cq.Solid.makeCylinder(1.15, 30, cq.Vector(px, py, z0 - 1)))
            r = r.cut(cq.Solid.makeCylinder(2.1, 2.2 + 1, cq.Vector(px, py, z0 - 1)))
            self.extra_holes.append(J.Hole("thrust_ring", f"ring.screw{i + 1}", "clr_m2",
                                           "M2x6 from below, head sunk 2.2 (into hip_mount)", (px, py, round(z0, 3)),
                                           (0, 0, 1), 2.3, 0, "thrust_ring"))
        return r.clean()

    # ------------------------------------------------------------ cable tie slots (2.5 mm ties) along the cable route
    TIES = {  # part: [(centre, slot axis)] slots are 3.0 × 1.6 through the spine or wall
        "thigh": [((-LP.HORN_R, 0.0, -L1 * 0.625), "y")],
        "shin": [((LP.HORN_R, 0.0, -L1 - L2 * 0.25), "y"), ((LP.HORN_R, 0.0, -L1 - L2 * 0.625), "y")],
        "yaw_bracket": [((-20.4, 0.0, 30.0), "x")],
    }

    def _tie_slots(self):
        for part, slots in self.TIES.items():
            p = self.parts[part]
            for c, ax in slots:
                for off in (-4.0, 4.0):
                    if ax == "y":
                        cut = bx((c[0] - 3, c[0] + 3), (c[1] + off - 0.8, c[1] + off + 0.8), (c[2] - 1.5, c[2] + 1.5))
                    else:
                        cut = bx((c[0] - 3, c[0] + 3), (c[1] + off - 0.8, c[1] + off + 0.8), (c[2] - 1.5, c[2] + 1.5))
                    p = p.cut(cut)
            self.parts[part] = p.clean()

    # ------------------------------------------------------------ tool access: every screw, as assembled
    def tool_paths(self):
        out = []
        for j in self.joints.values():
            for label, solid, off in j.tool_paths():
                if j.name == "hip_yaw":
                    if "ear" in label:
                        off = "yaw_bracket+thrust_ring"
                    elif "horn" in label:
                        off = "hip_block"          # the yaw horn screw is reached once the leg below is off
                out.append((label, solid, off))
        for h in self.extra_holes:
            if "screw" in h.id:                    # from below: sole and thrust ring screws
                start = np.asarray(h.pos) + 2.0 * np.asarray(h.dir)
                off = "yaw_bracket" if h.part == "thrust_ring" else None
                out.append((f"{h.part}: {h.id} (1.5 mm key)",
                            cq.Solid.makeCylinder(1.1, 35, cq.Vector(*start), cq.Vector(*(-np.asarray(h.dir)))), off))
        return out

    # ------------------------------------------------------------ cable route: every lead up the back to the hip mount
    def cable_routes(self, loop=30.0, to_board=80.0):
        """Lead length per servo: from its exit at the far end of the case, through crossing points just behind
        each joint it passes (where a joint's motion changes the length least), to the cable window of the hip
        mount, plus a slack loop per joint crossed and an allowance from the pelvis to the PCA9685."""
        window = self.window()
        out = {}
        for jn in ("ankle_roll", "ankle_pitch", "knee", "hip_pitch", "hip_roll"):
            j = self.joints[jn]
            pts = [np.array(j.world((j.P.CB, 0, j.P.CASE[0] + 3.5)))]
            # a lead crosses every joint between the link that holds its servo and the pelvis
            crossed = [c for c in ("ankle_pitch", "knee", "hip_pitch", "hip_roll", "hip_yaw")
                       if self._below(c, j.holder)]
            pts += [np.array(self.CROSS[c]) for c in crossed]
            pts.append(window)
            run = sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:]))
            out[jn] = dict(run=round(run), loops=len(crossed), crossed=crossed,
                           total=round(run + loop * len(crossed) + to_board))
        return out

    LINKS = ["yaw_bracket", "hip_block", "thigh", "shin", "ankle_block", "foot"]     # pelvis outward
    CHILD = {"hip_yaw": "yaw_bracket", "hip_roll": "hip_block", "hip_pitch": "thigh", "knee": "shin",
             "ankle_pitch": "ankle_block", "ankle_roll": "foot"}

    def _below(self, joint, link):
        """True if `link` hangs below `joint` (the joint is between it and the pelvis)."""
        return self.LINKS.index(self.CHILD[joint]) <= self.LINKS.index(link)

    @property
    def CROSS(self):
        ct = self.yaw_case_top()
        x = -LP.HORN_R - 2.0
        return {"ankle_pitch": (x, 0.0, A), "knee": (x, 0.0, K), "hip_pitch": (x, 0.0, 0.0),
                "hip_roll": (-24.0, 0.0, HIP_ROLL_UP), "hip_yaw": (-22.0, 0.0, ct)}

    def window(self):
        return np.array([-22.0, 0.0, self.yaw_case_top() + 10.0])

    # ------------------------------------------------------------ bought parts placed for display and checks
    def servos(self, cots=False):
        return {f"servo_{n}": j.servo_solid(cots=cots) for n, j in self.joints.items()}

    def horns(self):
        return {f"horn_{n}": j.horn_solid() for n, j in self.joints.items()}

    def caps(self):
        out = {}
        for n, j in self.joints.items():
            for i, c in enumerate(j.caps()):
                out[f"cap_{n}_{'horn' if i == 0 else 'idler'}"] = c
            if j.idler:
                out[f"spacer_{n}"] = j.spacer_solid()
        return out

    def sensors(self, side="l"):
        import sensors as sens
        return sens.get_leg_sensors(self, side=side)

    def summary(self):
        return dict(hip_roll_up=HIP_ROLL_UP, ankle_roll_down=ANKLE_ROLL_DOWN, L1=L1, L2=L2, hip_y=HIP_Y,
                    yaw_case_top=round(self.yaw_case_top(), 2),
                    pelvis_floor=round(self.yaw_case_top() - J.IW[0], 2),
                    yaw_plate_bottom=round(self.yaw_bottom, 2), sole_top=round(self.sole_top, 2),
                    floor=round(self.floor, 2), H_above_floor=round(-self.floor, 2),
                    A_above_floor=round(A - self.floor, 2), stack_width=round(2 * LP.HALF, 2))


if __name__ == "__main__":
    leg = Leg()
    print(leg.summary())
    for n, p in leg.parts.items():
        bb = p.BoundingBox()
        print(f"{n:12s} vol {p.Volume() / 1000:6.2f} cm3  x {bb.xmin:6.1f}..{bb.xmax:6.1f}"
              f"  y {bb.ymin:6.1f}..{bb.ymax:6.1f}  z {bb.zmin:6.1f}..{bb.zmax:6.1f}  valid {p.isValid()}")
    print(len(leg.holes()), "holes")
