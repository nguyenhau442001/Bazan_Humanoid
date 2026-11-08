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


MG92B = Servo("MG92B", length=22.8, width=12.2, case_h=22.7, shaft_off=5.9, tab_span=32.4, tab_t=2.5,
              tab_low=15.9, hole_pitch=27.8, hole_d=2.0, tower_d=11.8, tower_h=4.0, spline_d=4.8, spline_top=8.3,
              cable_z=(1.5, 5.5), cable_w=4.5, mass=14.0)                                       # MEASURE all
MG90S = Servo("MG90S", 22.8, 12.2, 22.7, 5.9, 32.4, 2.5, 15.9, 27.8, 2.0, 11.8, 4.0, 4.8, 8.3, (1.5, 5.5), 4.5,
              13.4)                                                                             # MEASURE all


# ---------------------------------------------------------------- horn (the round disc horn sold with the servo)
@dataclass(frozen=True)
class Horn:
    disc_d: float = 20.0     # MEASURE
    disc_t: float = 1.5      # MEASURE
    disc_low: float = 6.9    # case top to the underside of the disc, horn pressed fully on (MEASURE)
    hub_d: float = 7.0
    screw_r: float = 7.0     # radius of the two holes used to bolt the horn to the plate (drill if missing)
    screw_head_d: float = 4.5  # the horn's own centre screw


HORN = Horn()


# ---------------------------------------------------------------- fasteners (ISO metric, sold in HCMC hardware shops)
@dataclass(frozen=True)
class Fastener:
    code: str
    desc: str
    shop: str


FASTENERS = {
    "M2x6": Fastener("M2x6", "M2×6 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×6 inox"),
    "M2x8": Fastener("M2x8", "M2×8 socket head cap screw, ISO 4762, stainless", "ốc lục giác M2×8 inox"),
    "ST2x4": Fastener("ST2x4", "2×4 self-tapping pan head (horn to plate)", "vít tự ren đầu tròn 2×4"),
    "M3x6B": Fastener("M3x6B", "M3×6 button head, ISO 7380 (idler pivot)", "ốc đầu dù M3×6 inox"),
    "INS-M2": Fastener("INS-M2", "M2 brass heat-set insert, 3 long, 3.2 OD", "tán cấy nhiệt M2 L3 OD3.2"),
    "INS-M3": Fastener("INS-M3", "M3 brass heat-set insert, 3 long, 4.2 OD", "tán cấy nhiệt M3 L3 OD4.2"),
    "MR63ZZ": Fastener("MR63ZZ", "Ball bearing 3×6×2.5, shielded", "bạc đạn MR63ZZ 3×6×2.5"),
    "W3": Fastener("W3", "M3 shim washer 3.2×5×0.5 (inner race only)", "long đền mỏng M3 3.2×5×0.5"),
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
    "bearing": HoleSpec(6.05, 2.5, note="MR63ZZ seat, press fit; ream lightly if tight"),
    "access": HoleSpec(5.2, 0, note="hex key and screwdriver access to the horn screw"),
    "ins_m2_thru": HoleSpec(3.2, 0, note="heat-set insert M2 through a 3 mm plate: press flush with the marked face"),
    "ins_m3_thru": HoleSpec(4.0, 0, note="heat-set insert M3 through the plate: press flush with the top face"),
    "cb_m2": HoleSpec(2.3, 0, cb_d=4.4, cb_h=2.2, note="M2 clearance, counterbore for the socket head (flush)"),
    "clr_so": HoleSpec(3.3, 0, note="M3 clearance for the standoff's male thread"),
    "boss_m2": HoleSpec(3.2, 5.0, note="board boss: M2 insert pressed flush with the boss top, hole 5 deep"),
    "tie": HoleSpec(1.6, 0, note="cable-tie slot 3.0 × 1.6, through"),
}

# ---------------------------------------------------------------- print and fit rules
CLEAR_SERVO = 0.2     # case to pocket wall, each side
WALL = 2.0            # minimum structural wall
IDLER_WALL = 4.0      # pocket wall that carries the M3 insert for the idler
PLATE = 3.5           # horn plate
IDLER_PLATE = 3.0     # idler plate with the bearing
GAP_MOVING = 1.0      # minimum gap between parts that move against each other
