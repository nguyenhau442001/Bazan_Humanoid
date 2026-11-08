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
    shin         driven by the knee and by ankle pitch: a frame with no servo
    ankle_block  carries ankle pitch and ankle roll
    foot         driven by ankle roll
"""
import cadquery as cq
import numpy as np

import hardware as hw
import joint as J

HIP_ROLL_UP = 20.0       # D2B: hip roll axis above the hip-pitch axis
ANKLE_ROLL_DOWN = 20.0   # D2B: ankle roll axis below the ankle-pitch axis
L1 = 48.0                # H -> knee (was 45)
L2 = 48.0                # knee -> A (was 45)
HIP_ROLL_MAX = 20.0      # deg, mechanical range used for clearances (gait uses ±17)
ANKLE_ROLL_MAX = 20.0
SOLE_PLATE = 3.0
TPU_SOLE = 3.0
SOLE = dict(length=76.0, width=38.0, x=0.0, y=0.0)     # foot outline, centred under the ankle axes
SPINE = 3.6              # structural spine thickness
HIP_Y = 28.0             # leg centre lines at y = ±HIP_Y on the pelvis (was ±22): 44.8 wide legs, 11 mm apart
YAW_MAX = 45.0
RING_R = (13.0, 21.0)    # yaw thrust ring: carries the robot's weight past the yaw servo
ARM = 4.0                # side-plate arm thickness

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
NX, NY, NZ = (-1, 0, 0), (0, -1, 0), (0, 0, -1)
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
    def __init__(self):
        self.joints = {
            "hip_roll": J.Joint("hip_roll", "hip_block", "yaw_bracket", (0, 0, HIP_ROLL_UP), X, NY),
            "hip_pitch": J.Joint("hip_pitch", "hip_block", "thigh", (0, 0, 0), Y, NZ),
            "knee": J.Joint("knee", "thigh", "shin", (0, 0, K), Y, Z),
            "ankle_pitch": J.Joint("ankle_pitch", "ankle_block", "shin", (0, 0, A), Y, Z),
            "ankle_roll": J.Joint("ankle_roll", "ankle_block", "foot", (0, 0, AR), X, NY),
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
        self.parts["hip_block"] = self._finish("hip_block", hb)
        self.parts["ankle_block"] = self._finish("ankle_block", ab)
        self.parts["yaw_bracket"] = self._finish("yaw_bracket", self._yaw_bracket_body())
        self.parts["thigh"] = self._finish("thigh", self._thigh_body())
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
        web = bx((-8.3, 4.4), (-15.3, 4.4), (8.0, HIP_ROLL_UP - 8.2))
        return fuse(jr.holder_add(), jp.holder_add(), web)

    # ------------------------------------------------------------ ankle block: ankle pitch over ankle roll pocket
    def _ankle_block_body(self):
        jp, jr = self.joints["ankle_pitch"], self.joints["ankle_roll"]
        web = bx((-8.3, 4.4), (-15.3, 4.4), (AR + 8.2, A - 8.0))
        return fuse(jp.holder_add(), jr.holder_add(), web)

    # ------------------------------------------------------------ yaw bracket: U around the hip block
    def _yaw_bracket_body(self):
        jy, jr = self.joints["hip_yaw"], self.joints["hip_roll"]
        zb = self.yaw_bottom
        hp_x = sorted(p[0] for p in jr.a_range(J.HP))           # horn plate (front) x range
        ip_x = sorted(p[0] for p in jr.a_range(J.IP))           # idler plate (back) x range
        top_t = J.HP[1] - J.HP[0]
        top = fuse(bx((ip_x[0], hp_x[1]), (-9.0, 9.0), (zb, zb + top_t)),
                   cq.Solid.makeCylinder(RING_R[1], top_t, cq.Vector(0, 0, zb)))     # thrust face for the ring
        front = bx((hp_x[1] - ARM, hp_x[1]), (-J.HORN_R, J.HORN_R), (HIP_ROLL_UP, zb + top_t))
        back = bx((ip_x[0], ip_x[0] + ARM), (-J.IDLER_R, J.IDLER_R), (HIP_ROLL_UP, zb + top_t))
        return fuse(top, front, back, jy.horn_disc(), jr.horn_disc(), jr.idler_disc())

    # ------------------------------------------------------------ thigh: hip-pitch plates, back spine, knee pocket
    def _thigh_body(self):
        jp, jk = self.joints["hip_pitch"], self.joints["knee"]
        hp_y = sorted(p[1] for p in jp.a_range(J.HP))
        ip_y = sorted(p[1] for p in jp.a_range(J.IP))
        spine_x = (-12.0 - SPINE / 2, -12.0 + SPINE / 2)
        z_top = -24.5                                   # spine stays outside the hip block's reach from H
        # the knee servo slides in along -y: nothing on the +y side over the pocket (x -8.3..8.3, z up to K+23)
        horn_arm = fuse(bx((spine_x[0], 0.0), (hp_y[1] - ARM, hp_y[1]), (-J.HORN_R, 0.0)),
                        bx((spine_x[0], spine_x[1]), (hp_y[1] - ARM, hp_y[1]), (K + J.HORN_R + 2, 0.0)))
        idler_arm = fuse(bx((-9.0, 9.0), (ip_y[0], ip_y[0] + ARM), (K + 10, 0.0)),
                         bx((spine_x[0], 0), (ip_y[0], ip_y[0] + ARM), (K + 10, 0.0)))
        spine = bx(spine_x, (ip_y[0], hp_y[1]), (K + J.HORN_R + 2, z_top))
        pocket_link = bx((spine_x[0], -6.0), (-15.3, 4.4), (K - 8.1, K + 19.1))
        return fuse(jp.horn_disc(), jp.idler_disc(), horn_arm, idler_arm, spine, jk.holder_add(), pocket_link)

    # ------------------------------------------------------------ shin: knee and ankle plates, front spine (shin bone)
    def _shin_body(self):
        jk, ja = self.joints["knee"], self.joints["ankle_pitch"]
        hp_y = sorted(p[1] for p in jk.a_range(J.HP))
        ip_y = sorted(p[1] for p in jk.a_range(J.IP))
        spine_x = (12.0 - SPINE / 2, 12.0 + SPINE / 2)
        z0, z1 = A + 25.0, K - 15.0                     # clear of the ankle block's pitch pocket and the knee pocket
        plates = []
        for zc, lo in ((K, 1), (A, 0)):          # at A only the upper half: the ankle roll servo sits below
            plates += [bx((0.0, spine_x[1]), (hp_y[1] - ARM, hp_y[1]), (zc - lo * J.HORN_R, zc + J.HORN_R)),
                       bx((0.0, spine_x[1]), (ip_y[0], ip_y[0] + ARM), (zc - lo * J.IDLER_R, zc + J.IDLER_R))]
        side_h = bx(spine_x, (hp_y[1] - ARM, hp_y[1]), (A, K))
        side_i = bx(spine_x, (ip_y[0], ip_y[0] + ARM), (A, K))
        spine = bx(spine_x, (ip_y[0], hp_y[1]), (z0, z1))
        return fuse(jk.horn_disc(), jk.idler_disc(), ja.horn_disc(), ja.idler_disc(), *plates, side_h, side_i, spine)

    # ------------------------------------------------------------ foot: ankle-roll uprights on the sole plate
    def _foot_body(self):
        jr = self.joints["ankle_roll"]
        hp_x = sorted(p[0] for p in jr.a_range(J.HP))
        ip_x = sorted(p[0] for p in jr.a_range(J.IP))
        st = self.sole_top
        sx0, sx1 = SOLE["x"] - SOLE["length"] / 2, SOLE["x"] + SOLE["length"] / 2
        sy0, sy1 = SOLE["y"] - SOLE["width"] / 2, SOLE["y"] + SOLE["width"] / 2
        plate = bx((sx0, sx1), (sy0, sy1), (st - SOLE_PLATE, st))
        front = bx((hp_x[1] - ARM, hp_x[1]), (-J.HORN_R, J.HORN_R), (st - 0.01, AR))
        back = bx((ip_x[0], ip_x[0] + ARM), (-J.IDLER_R, J.IDLER_R), (st - 0.01, AR))
        body = fuse(plate, front, back, jr.horn_disc(), jr.idler_disc())
        # sole: 4 × M2 inserts from below, at the corners
        spec = hw.HOLES["ins_m2"]
        for i, (px, py) in enumerate([(sx0 + 7, sy0 + 7), (sx1 - 7, sy0 + 7), (sx0 + 7, sy1 - 7), (sx1 - 7, sy1 - 7)]):
            z0 = st - SOLE_PLATE
            body = body.cut(cq.Solid.makeCylinder(spec.d / 2, spec.depth - 1.0, cq.Vector(px, py, z0)))
            self.extra_holes.append(J.Hole("foot", f"sole.ins{i + 1}", "ins_m2", "INS-M2 + M2x6 (TPU sole)",
                                           (px, py, round(z0, 3)), (0, 0, 1), spec.d, spec.depth - 1.0, "sole"))
        return body

    def _sole(self):
        sx0, sx1 = SOLE["x"] - SOLE["length"] / 2, SOLE["x"] + SOLE["length"] / 2
        sy0, sy1 = SOLE["y"] - SOLE["width"] / 2, SOLE["y"] + SOLE["width"] / 2
        z0 = self.sole_top - SOLE_PLATE - TPU_SOLE
        s = bx((sx0, sx1), (sy0, sy1), (z0, z0 + TPU_SOLE))
        for i, (px, py) in enumerate([(sx0 + 7, sy0 + 7), (sx1 - 7, sy0 + 7), (sx0 + 7, sy1 - 7), (sx1 - 7, sy1 - 7)]):
            s = s.cut(cq.Solid.makeCylinder(1.15, TPU_SOLE + 1, cq.Vector(px, py, z0 - 0.5)))
            s = s.cut(cq.Solid.makeCylinder(2.1, 2.0, cq.Vector(px, py, z0 - 0.01)))
            self.extra_holes.append(J.Hole("sole", f"sole.screw{i + 1}", "clr_m2", "M2x6 from below, head sunk 2",
                                           (px, py, round(z0, 3)), (0, 0, 1), 2.3, 0, "sole"))
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
                                           "INS-M2 + M2x8 from the pelvis floor (leg module)", (px, py, round(top, 3)),
                                           (0, 0, -1), spec.d, spec.depth, "pelvis"))
        for i, sy in enumerate((-1, 1)):
            px, py = -4.0, sy * 14.5
            body = body.cut(cq.Solid.makeCylinder(spec.d / 2, spec.depth, cq.Vector(px, py, ct - 0.1)))
            self.extra_holes.append(J.Hole("hip_mount", f"ring.ins{i + 1}", "ins_m2", "INS-M2 + M2x6 (thrust ring)",
                                           (px, py, round(ct - 0.1, 3)), (0, 0, 1), spec.d, spec.depth, "thrust_ring"))
        # cable window at the back for the five leg servo leads
        body = body.cut(bx((-25.0, -19.5), (-6.0, 6.0), (ct + 1.0, top - 3.5)))
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
        "thigh": [((-12.0, 0.0, -30.0), "y")],
        "shin": [((12.0, 0.0, -60.0), "y"), ((12.0, 0.0, -78.0), "y")],
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
        ct = self.yaw_case_top()
        window = np.array([-22.0, 0.0, ct + 10.0])
        cross = {"ankle_pitch": (-14.0, 0.0, A), "knee": (-14.0, 0.0, K), "hip_pitch": (-14.0, 0.0, 0.0),
                 "hip_roll": (-24.0, 0.0, HIP_ROLL_UP), "hip_yaw": (-22.0, 0.0, ct)}
        order = ["ankle_roll", "ankle_pitch", "knee", "hip_pitch", "hip_roll", "hip_yaw"]
        out = {}
        for i, jn in enumerate(order[:-1]):
            j = self.joints[jn]
            pts = [np.array(j.world((J.CB, 0, J.CASE[0] + 3.5)))]
            crossed = [c for c in order[i + 1:]]
            pts += [np.array(cross[c]) for c in crossed if c in cross]
            pts.append(window)
            run = sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:]))
            out[jn] = dict(run=round(run), loops=len(crossed), total=round(run + loop * len(crossed) + to_board))
        return out

    # ------------------------------------------------------------ bought parts placed for display and checks
    def servos(self):
        return {f"servo_{n}": j.servo_solid() for n, j in self.joints.items()}

    def horns(self):
        return {f"horn_{n}": j.horn_solid() for n, j in self.joints.items()}

    def caps(self):
        out = {}
        for n, j in self.joints.items():
            for i, c in enumerate(j.caps()):
                out[f"cap_{n}_{'horn' if i == 0 else 'idler'}"] = c
        return out

    def summary(self):
        return dict(hip_roll_up=HIP_ROLL_UP, ankle_roll_down=ANKLE_ROLL_DOWN, L1=L1, L2=L2, hip_y=HIP_Y,
                    yaw_case_top=round(self.yaw_case_top(), 2),
                    pelvis_floor=round(self.yaw_case_top() - J.IW[0], 2),
                    yaw_plate_bottom=round(self.yaw_bottom, 2), sole_top=round(self.sole_top, 2),
                    floor=round(self.floor, 2), H_above_floor=round(-self.floor, 2),
                    A_above_floor=round(A - self.floor, 2), stack_width=round(2 * J.HALF, 2))


if __name__ == "__main__":
    leg = Leg()
    print(leg.summary())
    for n, p in leg.parts.items():
        bb = p.BoundingBox()
        print(f"{n:12s} vol {p.Volume() / 1000:6.2f} cm3  x {bb.xmin:6.1f}..{bb.xmax:6.1f}"
              f"  y {bb.ymin:6.1f}..{bb.ymax:6.1f}  z {bb.zmin:6.1f}..{bb.zmax:6.1f}  valid {p.isValid()}")
    print(len(leg.holes()), "holes")
