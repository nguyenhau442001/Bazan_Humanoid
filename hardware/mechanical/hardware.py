"""Bought parts and print rules for Mico's mechanics: servos, screws, inserts, bearings.

Every size is in millimetres. Values marked MEASURE come from datasheets and clone-to-clone they vary by a few
tenths: measure the servos you bought with calipers and update this file before printing. Everything downstream
(pockets, holes, drawings, the assembly guide) is generated from here.
"""
from dataclasses import dataclass


# ---------------------------------------------------------------- servos ("9 g" case family)
@dataclass(frozen=True)
class Servo:
    name: str
    length: float        # case along its long side (b axis)
    width: float         # case across (w axis)
    case_h: float        # case bottom to case top (a axis)
    shaft_off: float     # output shaft centre to the near end of the case
    tab_span: float      # mounting ears, end to end
    tab_t: float         # ear thickness
    tab_low: float       # case bottom to the underside of the ears
    hole_pitch: float    # ear hole centre to centre
    hole_d: float        # ear hole diameter
    tower_d: float       # gear tower diameter, centred on the shaft
    tower_h: float       # above the case top
    spline_d: float
    spline_top: float    # case top to the top of the spline
    cable_z: tuple       # cable exit window on the far end face, from the case bottom (min, max)
    cable_w: float       # window width
    mass: float          # g
    stall_kgcm: float = 0.0   # catalogue stall torque at 6 V (MEASURE on the P0 bench)
    speed_s60: float = 0.0    # catalogue no-load speed, s per 60 deg


MG92B = Servo("MG92B", length=22.8, width=12.2, case_h=22.7, shaft_off=5.9, tab_span=32.4, tab_t=2.5,
              tab_low=15.9, hole_pitch=27.8, hole_d=2.0, tower_d=11.8, tower_h=4.0, spline_d=4.8, spline_top=8.3,
              cable_z=(1.5, 5.5), cable_w=4.5, mass=14.0, stall_kgcm=3.5, speed_s60=0.08)       # MEASURE all
MG90S = Servo("MG90S", 22.8, 12.2, 22.7, 5.9, 32.4, 2.5, 15.9, 27.8, 2.0, 11.8, 4.0, 4.8, 8.3, (1.5, 5.5), 4.5,
              13.4, stall_kgcm=2.2, speed_s60=0.08)                                             # MEASURE all
# SG90 (shoulder roll, elbow, neck yaw, head pitch). Sizes from the web, not measured yet (2026-11-18):
#   TowerPro sells the SG90 as 23.0 × 12.2 × 29 mm and the MG90S as "identical mounting dimensions to the SG90";
#   the datasheet table gives F = 16 (case bottom to ear underside) like the MG90S column. The old numbers here
#   (22.2 × 11.8) come from clone datasheets and put the shaft 0.3 mm off the idler axis in the shared pocket.
#   Spline 21 teeth, Ø4.8 (same as MG90S). The vendor MG90S STEP in cots/ disagrees on tab_low (18.4) and the
#   tower (5.75): see docs/mechanical/test_print.md; measure one servo before the final print.
SG90 = Servo("SG90", length=22.8, width=12.2, case_h=22.7, shaft_off=5.9, tab_span=32.4, tab_t=2.5,
             tab_low=15.9, hole_pitch=27.8, hole_d=2.0, tower_d=11.8, tower_h=4.0, spline_d=4.8, spline_top=8.3,
             cable_z=(1.5, 5.5), cable_w=4.5, mass=9.0, stall_kgcm=1.8, speed_s60=0.10)         # MEASURE all
SERVOS = {s.name: s for s in (MG92B, MG90S, SG90)}
JOINT_SERVO = {  # which servo type sits in which joint (both sides)
    "hip_yaw": "MG90S", "hip_roll": "MG92B", "hip_pitch": "MG92B", "knee": "MG92B", "ankle_pitch": "MG92B",
    "ankle_roll": "MG92B", "shoulder_pitch": "MG90S", "shoulder_roll": "SG90", "elbow": "SG90",
    "neck_yaw": "SG90", "head_pitch": "SG90",
}
# Servo types whose sizes above still come from datasheets. Remove a name once you have measured that servo
# with calipers and typed the numbers in (see docs/mechanical/test_print.md).
UNMEASURED = {"MG92B", "MG90S", "SG90"}


# ---------------------------------------------------------------- horn (the round disc horn sold with the servo)
@dataclass(frozen=True)
class Horn:
    disc_d: float = 20.0     # MEASURE
    disc_t: float = 1.5      # MEASURE
    disc_low: float = 6.9    # case top to the underside of the disc, horn pressed fully on (MEASURE)
    hub_d: float = 7.0
    screw_r: float = 7.0     # radius of the two holes used to bolt the horn to the plate (drill if missing)
    screw_head_d: float = 4.5  # the horn's own centre screw
    measured: bool = False   # True once disc_t and disc_low come from calipers (step M-2), not the datasheet


def horn_screw(horn=None):
    """Horn-to-plate screw (decided 2026-11-19): ST2x3 until the real horn is measured (M-2); ST2x4 only if the
    disc is 1.8 or thicker, so the longer tip still stops short of the outer skin. No CAD change either way."""
    horn = horn or HORN
    return "ST2x4" if horn.measured and horn.disc_t >= 1.8 else "ST2x3"


HORN = Horn()

# 3-pin servo plug (JR / Dupont housing, 3 x 2.54 pitch): it has to pass every opening the lead goes through
PLUG = (7.7, 2.6)        # width, thickness (MEASURE)


# ---------------------------------------------------------------- fasteners (ISO metric, sold in HCMC hardware shops)
@dataclass(frozen=True)
class Fastener:
    code: str
    desc: str
    shop: str


FASTENERS = {
    "M2x6": Fastener("M2x6", "M2×6 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×6 inox"),
    # M2x4: TPU sole (1.0 under the sunk head + 3.0 insert); M2x5: board sleds (1.8 + 3.2 in the deck insert)
    "M2x4": Fastener("M2x4", "M2×4 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×4 inox"),
    "M2x5": Fastener("M2x5", "M2×5 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×5 inox"),
    "M2x8": Fastener("M2x8", "M2×8 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×8 inox"),
    "ST2x4": Fastener("ST2x4", "2×4 self-tapping pan head (horn to plate)", "vít tự ren đầu tròn 2×4"),
    # horn to plate: 3 long so the tip stops 0.9 short of the 0.4 skin over the blind pilot (N3 review E-1)
    "ST2x3": Fastener("ST2x3", "2×3 self-tapping pan head (horn to plate)", "vít tự ren đầu tròn PA 2×3"),
    "M3x6B": Fastener("M3x6B", "M3×6 button head, ISO 7380 (idler pivot)", "ốc đầu dù M3×6 inox"),
    # inserts: CNC Kitchen sizes (datasheet): M2 standard L3.0, knurl Ø3.6, hole Ø3.2, wall ≥ 1.3;
    # M3 short L3.0, knurl Ø4.6, hole Ø4.0, wall ≥ 1.6. Any insert with these sizes works; others need a coupon.
    "INS-M2": Fastener("INS-M2", "M2 brass heat-set insert, L3.0, knurl Ø3.6, for a Ø3.2 hole (CNC Kitchen "
                       "TC-M2x3.0 or equal)", "tán cấy nhiệt M2 dài 3, gai Ø3.6, lỗ Ø3.2"),
    "INS-M3": Fastener("INS-M3", "M3 brass heat-set insert, L3.0, knurl Ø4.6, for a Ø4.0 hole (CNC Kitchen "
                       "M3 short or equal)", "tán cấy nhiệt M3 dài 3, gai Ø4.6, lỗ Ø4.0"),
    # MR63ZZ: OD 6 (0/−0.008), width 2.5 (0/−0.12). In every joint it is the floating bearing: the outer ring
    # slides in its seat (slip fit), the joint is located along its axis by the horn on the spline.
    "MR63ZZ": Fastener("MR63ZZ", "Ball bearing 3×6×2.5, shielded", "bạc đạn MR63ZZ 3×6×2.5"),
    "W3": Fastener("W3", "M3 steel shim washer 3.2×5×0.5 under the idler screw head (flat: bridges the recessed "
                   "shield)", "long đền thép mỏng M3 3.2×5×0.5"),
    "M3x10B": Fastener("M3x10B", "M3×10 button head, ISO 7380 (leg idler pivot)", "ốc đầu dù M3×10 inox"),
    # idler spacer: clamps the bearing inner race (land ~Ø3.9) against the wall insert; Ø4.0 stays inside the
    # shield's inner edge, a printed boss (Ø4.2) did not, and the M3 insert's Ø4.6 knurl never fitted in that boss
    "TUBE4": Fastener("TUBE4", "Brass tube Ø4.0 × 0.45 wall (ID 3.1), cut square: 1.3 mm (head), 5.1 mm (legs)",
                      "ống đồng Ø4 dày 0.45 (cắt đoạn 1.3 / 5.1 mm)"),
    # body v4 (torso stack, sleds, arms, head)
    "M2x10": Fastener("M2x10", "M2×10 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×10 inox"),
    "M3x6": Fastener("M3x6", "M3×6 socket head cap screw, ISO 4762, stainless", "ốc lục giác M3×6 inox"),
    "SO40": Fastener("SO40", "M3 hex standoff male-female, nylon, 40 + 6 thread, 5.5 AF", "trụ nhựa đực-cái M3 40+6"),
    "SO35": Fastener("SO35", "M3 hex standoff male-female, nylon, 35 + 6 thread, 5.5 AF", "trụ nhựa đực-cái M3 35+6"),
    "TIE": Fastener("TIE", "cable tie 2.5 mm wide, 100 mm", "dây rút nhựa 2.5×100"),
    "STRAP": Fastener("STRAP", "velcro battery strap 20 mm × 200 mm", "dây dán pin velcro 20 mm"),
    "FOAM": Fastener("FOAM", "double-sided foam tape 1 mm", "băng keo 2 mặt xốp 1 mm"),
}


# ---------------------------------------------------------------- holes as printed (FDM, 0.4 nozzle, PLA)
@dataclass(frozen=True)
class HoleSpec:
    d: float           # printed diameter
    depth: float       # 0 = through
    cb_d: float = 0.0  # counterbore diameter (0 = none)
    cb_h: float = 0.0
    note: str = ""


HOLES = {
    "ins_m2": HoleSpec(3.2, 4.0, note="heat-set insert M2: press with a soldering iron at 220 °C"),
    "ins_m3": HoleSpec(4.0, 3.6, note="heat-set insert M3: press with a soldering iron at 220 °C"),
    "clr_m2": HoleSpec(2.3, 0, note="M2 clearance"),
    "clr_m3": HoleSpec(3.3, 0, note="M3 clearance"),
    "pilot_st2": HoleSpec(1.6, 0, note="pilot for 2 mm self-tapping screw through the horn"),
    "bearing": HoleSpec(6.05, 2.5, note="MR63ZZ seat, SLIP fit (floating bearing): Ø6.00 pin goes in by hand, "
                        "Ø6.10 does not; set XY hole compensation from T0, never ream free-hand"),
    "access": HoleSpec(5.2, 0, note="hex key and screwdriver access to the horn screw"),
    "ins_m2_thru": HoleSpec(3.2, 0, note="heat-set insert M2 through a 3 mm plate: press flush with the marked face"),
    "ins_m3_thru": HoleSpec(4.0, 0, note="heat-set insert M3 through the plate: press flush with the top face"),
    "cb_m2": HoleSpec(2.3, 0, cb_d=4.4, cb_h=2.2, note="M2 clearance, counterbore for the socket head (flush)"),
    "clr_so": HoleSpec(3.3, 0, note="M3 clearance for the standoff's male thread"),
    "boss_m2": HoleSpec(3.2, 5.0, note="board boss: M2 insert pressed flush with the boss top, hole 5 deep"),
    "tie": HoleSpec(1.6, 0, note="cable-tie slot 3.0 × 1.6, through"),
    "cable": HoleSpec(0.0, 0, note="cable window, rectangular (size in the feature)"),
    "ntc": HoleSpec(3.6, 0, note="NTC 10k epoxy bead (Ø3, ASSUMED) through the pocket wall onto the servo case"),
    "fsr": HoleSpec(18.8, 0.6, note="FSR 402 pocket (Ø18.28 x 0.46-0.55, Interlink datasheet, ASSUMED) + tail channel"),
}

# ---------------------------------------------------------------- print and fit rules
CLEAR_SERVO = 0.2     # case to pocket wall, each side
WALL = 2.0            # minimum structural wall
IDLER_WALL = 4.0      # pocket wall that carries the M3 insert for the idler
PLATE = 3.5           # horn plate
IDLER_PLATE = 3.0     # idler plate with the bearing
GAP_MOVING = 1.0      # minimum gap between parts that move against each other
