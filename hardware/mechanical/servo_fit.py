"""Does each servo type fit the pocket of its size class? Numbers only, no CAD: fast enough to run after every edit.

joint.py builds one pocket per size class (hardware.POCKET_SERVO): the 9 g pocket around the MG92B, shared by the
MG90S and the SG90, and the standard pocket around the DS3218. A servo whose sizes differ from its pocket's servo
can wobble, sit too high or too low (the ears rest on the bosses, so the ear height sets where the shaft ends up
along the joint axis), or not take its ear screws.

    python servo_fit.py            # table for every servo in hardware.SERVOS, exits 1 if one does not fit

Run it after you measure your servos and update hardware.py. A FAIL means: change the pocket, or shim / drill the
servo as the hint says, before you print the parts for that joint.
"""
import sys
from dataclasses import dataclass

import hardware as hw
import joint as J

SCREW_D = {"M2": 2.0, "M3": 3.0}     # ear screw major diameter


@dataclass
class Check:
    what: str
    value: float        # mm, the margin (> 0 = room left) or the error
    lo: float           # pass if lo <= value <= hi
    hi: float
    hint: str

    @property
    def ok(self):
        return self.lo - 1e-6 <= self.value <= self.hi + 1e-6


def checks(t: hw.Servo) -> list:
    """Margins of servo t seated in the pocket of its size class: ears on the bosses, shaft on the joint axis."""
    P = J.pocket(t)
    p, c = P.S, hw.CLEAR_SERVO
    screw = SCREW_D[t.ear_screw[:2]]
    # along the shaft (a), case top of the pocket's servo = 0. t's ears rest on the same bosses (TAB_A[0]).
    top = P.TAB_A[0] + (t.case_h - t.tab_low)            # where t's case top ends up
    bottom = top - t.case_h
    cab = (bottom + t.cable_z[0], bottom + t.cable_z[1])
    win = (P.CASE[0] + p.cable_z[0] - 0.5, P.CASE[0] + p.cable_z[1] + 0.5)
    # along the case (b), shaft = 0
    mid = t.length / 2 - t.shaft_off
    holes = (mid - t.hole_pitch / 2, mid + t.hole_pitch / 2)
    hole_err = max(abs(h - hp) for h, hp in zip(holes, P.HOLE_B))
    return [
        Check("width: gap per side", (p.width + 2 * c - t.width) / 2, 0.05, 0.35,
              "under 0.05: sand the pocket; over 0.35: one layer of paper tape (0.1) on both sides of the case"),
        Check("length: near end of case to pocket", -t.shaft_off - (P.CB0 - c), 0.0, 1.0,
              "negative: the case hits the pocket end"),
        Check("length: far end of case to pocket", (P.CB + c) - (t.length - t.shaft_off), 0.0, 1.0,
              "negative: the case hits the pocket end"),
        Check("ear span: room at the ear slot ends", (p.tab_span + 2 * c - t.tab_span) / 2 - abs(mid - P.CMID),
              0.0, 1.0, "negative: the ears do not drop into the slot; file the ear ends"),
        Check(f"ear holes: {t.ear_screw[:2]} passes (hole - screw)", t.hole_d - screw, 0.2, 1.0,
              "drill both ear holes to Ø2.3 with a hand drill, then set hole_d = 2.3 in hardware.py"),
        Check("ear holes: offset to the inserts", hole_err, 0.0, max((t.hole_d - screw) / 2, 0.0),
              "the M2 screw cannot reach the insert: drill the ear holes larger (Ø2.4 allows 0.2)"),
        Check("shaft height: case top vs design", top, -0.3, 0.3,
              "the horn and the idler stack shift by this much along the axis: change TAB_A or shim the ears"),
        Check("case bottom to idler wall", bottom - P.IW[1], 0.0, 1.0,
              "negative: the case hits the idler wall before the ears seat"),
        Check("gear tower to horn plate", P.HP[0] - (top + t.tower_h), 0.3, 2.0,
              "under 0.3: the turning horn plate rubs on the tower"),
        Check("spline top vs design (horn seating)", top + t.spline_top - p.spline_top, -0.5, 0.5,
              "the horn sits higher/lower than its recess in the horn plate"),
        Check("cable exit inside the window", min(cab[0] - win[0], win[1] - cab[1]), 0.0, 5.0,
              "negative: the lead is pinched at the end of the pocket"),
    ]


def main():
    bad = 0
    print(f"Clearance {hw.CLEAR_SERVO} per side. Values in mm.\n")
    for t in hw.SERVOS.values():
        print(f"{t.name} in the {J.pocket(t).S.name} pocket"
              f"{'  (MEASURE: datasheet values)' if t.name in hw.UNMEASURED else ''}")
        for ch in checks(t):
            mark = "ok  " if ch.ok else "FAIL"
            bad += not ch.ok
            print(f"  {mark} {ch.what:40s} {ch.value:+6.2f}   pass {ch.lo:+.2f}..{ch.hi:+.2f}"
                  + ("" if ch.ok else f"\n         -> {ch.hint}"))
        print()
    print("all servo types fit" if not bad else f"{bad} check(s) failed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
