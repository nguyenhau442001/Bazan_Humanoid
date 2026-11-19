"""Draw the BZN-MB rev A diagram set as SVG: sheet 1 board block diagram, sheet 2 power tree.

Every block, connector and wire is placed by hand on a 10 px grid, wires are orthogonal, crossings carry no dot
(a dot marks a junction). Run: python3 gen_diagrams.py  (writes BZN-MB-BD-001_s1.svg and _s2.svg here).
"""
from datetime import date
from pathlib import Path
from xml.sax.saxutils import escape

HERE = Path(__file__).resolve().parent
W, H = 1700, 1100
DOC, REV, STATUS = "BZN-MB-BD-001", "A0", "DRAFT, FOR REVIEW"
INK, PWR, SIG, ANA, BUS, OFF = "#1d2333", "#c4622f", "#2f3d63", "#2c7a5a", "#6a3d9a", "#8d8778"
FILL = {"pwr": "#fde8dc", "logic": "#e6eaf4", "ana": "#e1f1ea", "off": "#ffffff", "prot": "#fbd3d3", "mcu": "#d6def0"}
STROKE = {"pwr": PWR, "logic": SIG, "ana": ANA, "off": OFF, "prot": "#b03030", "mcu": SIG}


class Sheet:
    def __init__(self, title, sheet_no, sheets):
        self.el, self.title, self.no, self.n = [], title, sheet_no, sheets
        self.el.append(f'<rect x="10" y="10" width="{W - 20}" height="{H - 20}" fill="white" stroke="{INK}" '
                       f'stroke-width="2"/>')

    def text(self, x, y, s, size=12, anchor="start", weight="normal", color=INK, italic=False, mono=False):
        st = ' font-style="italic"' if italic else ""
        st += ' font-family="DejaVu Sans Mono, monospace"' if mono else ""
        for i, line in enumerate(s.split("\n")):
            self.el.append(f'<text x="{x}" y="{y + i * (size + 3)}" font-size="{size}" text-anchor="{anchor}" '
                           f'font-weight="{weight}" fill="{color}"{st}>{escape(line)}</text>')

    def block(self, x, y, w, h, ref, body, kind="logic", dashed=False, mono=False):
        dash = ' stroke-dasharray="6 4"' if dashed else ""
        self.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="3" fill="{FILL[kind]}" '
                       f'stroke="{STROKE[kind]}" stroke-width="1.4"{dash}/>')
        if ref:
            self.text(x + 6, y + 15, ref, 12, weight="bold", color=STROKE[kind])
            self.text(x + 6, y + 31, body, 11, mono=mono)
        else:
            self.text(x + 6, y + 16, body, 11)

    def conn(self, x, y, ref, side):
        """Connector symbol straddling the board edge at (x, y)."""
        self.el.append(f'<rect x="{x - 24}" y="{y - 11}" width="48" height="22" fill="#fff6c8" stroke="{INK}" '
                       f'stroke-width="1.2"/>')
        self.text(x, y + 4, ref, 10, "middle", "bold")

    def wire(self, pts, kind="sig", label=None, at=0, dy=-5, arrow=True, dash=False, width=None):
        c = {"pwr": PWR, "sig": SIG, "ana": ANA, "bus": BUS, "off": OFF}[kind]
        w = width or {"pwr": 3.0, "bus": 3.0}.get(kind, 1.4)
        d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(pts))
        da = ' stroke-dasharray="5 4"' if dash else ""
        mk = f' marker-end="url(#a_{kind})"' if arrow else ""
        self.el.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="{w}"{da}{mk}/>')
        if label:
            (x0, y0), (x1, y1) = pts[at], pts[at + 1]
            if y0 == y1:
                self.text((x0 + x1) / 2, y0 + dy, label, 10, "middle", color=c)
            else:
                self.text(x0 + 6, (y0 + y1) / 2, label, 10, color=c)

    def dot(self, x, y, kind="sig"):
        c = {"pwr": PWR, "sig": SIG, "ana": ANA, "bus": BUS}[kind]
        self.el.append(f'<circle cx="{x}" cy="{y}" r="4" fill="{c}"/>')

    def title_block(self):
        x, y, w = W - 560, H - 150, 550
        rows = [("Project", "BZN-01 Bazan humanoid: mainboard BZN-MB rev A"), ("Title", self.title),
                ("Doc no.", f"{DOC}   Rev {REV}   Sheet {self.no} / {self.n}"),
                ("Date / drawn", f"{date.today().isoformat()}   H. Nguyen Ngoc"),
                ("Status", STATUS), ("Source", "hardware/electrical/mainboard/gen_diagrams.py")]
        self.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="140" fill="white" stroke="{INK}" '
                       f'stroke-width="2"/>')
        for i, (k, v) in enumerate(rows):
            yy = y + i * 23
            if i:
                self.el.append(f'<line x1="{x}" y1="{yy}" x2="{x + w}" y2="{yy}" stroke="{INK}"/>')
            self.text(x + 8, yy + 16, k, 11, color="#555")
            self.text(x + 110, yy + 16, v, 12, weight="bold" if i in (1, 2) else "normal")
        self.el.append(f'<line x1="{x + 100}" y1="{y}" x2="{x + 100}" y2="{y + 140}" stroke="{INK}"/>')

    def legend(self, x, y, items):
        self.el.append(f'<rect x="{x}" y="{y}" width="400" height="{24 + 20 * len(items)}" fill="white" '
                       f'stroke="{INK}"/>')
        self.text(x + 8, y + 16, "Legend", 11, weight="bold")
        for i, (kind, label) in enumerate(items):
            yy = y + 34 + i * 20
            if kind in ("pwr", "sig", "ana", "bus"):
                self.wire([(x + 10, yy - 4), (x + 60, yy - 4)], kind, arrow=kind != "bus")
            elif kind == "kelvin":
                self.wire([(x + 10, yy - 4), (x + 60, yy - 4)], "ana", dash=True, arrow=False)
            elif kind == "conn":
                self.conn(x + 35, yy - 4, "J", "l")
            elif kind == "dot":
                self.wire([(x + 10, yy - 4), (x + 60, yy - 4)], "sig", arrow=False)
                self.dot(x + 35, yy - 4)
            else:
                self.el.append(f'<rect x="{x + 15}" y="{yy - 12}" width="40" height="16" fill="{FILL[kind]}" '
                               f'stroke="{STROKE[kind]}"/>')
            self.text(x + 75, yy, label, 11)

    def save(self, name):
        markers = "".join(
            f'<marker id="a_{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="11" markerHeight="11" '
            f'markerUnits="userSpaceOnUse" orient="auto-start-reverse">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            for k, c in (("pwr", PWR), ("sig", SIG), ("ana", ANA), ("bus", BUS), ("off", OFF)))
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
               f'font-family="DejaVu Sans, Arial, sans-serif"><defs>{markers}</defs>' + "\n".join(self.el) + "</svg>")
        (HERE / name).write_text(svg)


def sheet1():
    s = Sheet("Board block diagram", 1, 2)
    s.text(30, 45, "BZN-MB rev A: board block diagram", 22, weight="bold")
    s.text(30, 68, "Servo power flows along the top band, logic and sensing along the bottom band. Pins, addresses "
                   "and channels match software/firmware/stm32f411 (main.h, joint_table.h).", 12, color="#555")
    # board boundary
    bx0, by0, bx1, by1 = 250, 90, 1390, 900
    s.el.append(f'<rect x="{bx0}" y="{by0}" width="{bx1 - bx0}" height="{by1 - by0}" fill="none" stroke="{INK}" '
                f'stroke-width="2.5" stroke-dasharray="14 6"/>')
    s.text(bx0 + 10, by0 + 20, "BZN-MB rev A  (80 × 60 mm, 4 layers)", 13, weight="bold")

    # ---------------- off-board, left
    s.block(30, 110, 180, 50, "BT1", "2S LiPo 1100 mAh 30C\n6.0–8.4 V", "off")
    s.block(30, 190, 180, 50, "F_MAIN + SW1", "15 A fuse, power switch\non the battery lead", "prot")
    s.block(30, 275, 180, 50, "UBEC (module)", "6.0 V, 8 A cont.\n15 A burst", "off")
    s.wire([(120, 160), (120, 190)], "pwr")
    s.wire([(120, 240), (120, 275)], "pwr")
    s.block(30, 560, 180, 60, "Leg servo pots", "12 wipers + ground sense\nL 6, R 6 (MG92B)", "off")
    s.block(30, 660, 180, 44, "Foot sensors", "4 × FSR402", "off")
    s.block(30, 740, 180, 44, "Hip-roll servos", "2 × NTC 10 kΩ on case", "off")

    # ---------------- off-board, right
    s.block(1480, 110, 190, 120, "Leg servos", "12 × MG92B\nL: J3–J8\nR: J9–J14", "off")
    s.block(1480, 390, 190, 64, "Arm + neck servos", "2 × MG90S, 6 × SG90\nJ15–J22", "off")
    s.block(1480, 500, 190, 50, "BUCK (module)", "5 V 3 A", "off")
    s.block(1480, 580, 190, 54, "Radxa Zero 3W", "application domain", "off")
    s.block(1480, 800, 190, 50, "Debug probe", "ST-Link / OpenOCD", "off")
    s.wire([(30, 215), (20, 215), (20, 80), (1575, 80), (1575, 500)], "pwr", "VBAT (after F_MAIN)", at=3)
    s.wire([(1575, 550), (1575, 580)], "pwr")

    # ---------------- connectors on the edge
    s.conn(bx0, 215, "J2", "l")
    s.conn(bx0, 300, "J1", "l")
    s.conn(bx0, 575, "J23", "l")
    s.conn(bx0, 605, "J24", "l")
    s.conn(bx0, 682, "J25", "l")
    s.conn(bx0, 762, "J26", "l")
    s.conn(bx1, 170, "J3–J14", "r")
    s.conn(bx1, 420, "J15–J22", "r")
    s.conn(bx1, 607, "J27", "r")
    s.conn(bx1, 825, "J28", "r")
    s.wire([(210, 215), (bx0 - 24, 215)], "pwr", arrow=False)
    s.wire([(210, 300), (bx0 - 24, 300)], "pwr", arrow=False)
    s.wire([(210, 575), (bx0 - 24, 575)], "ana", arrow=False)
    s.wire([(210, 605), (bx0 - 24, 605)], "ana", arrow=False)
    s.wire([(210, 682), (bx0 - 24, 682)], "ana", arrow=False)
    s.wire([(210, 762), (bx0 - 24, 762)], "ana", arrow=False)
    s.wire([(bx1 + 24, 170), (1480, 170)], "pwr", "6 V + PWM", dy=-6)
    s.wire([(bx1 + 24, 420), (1480, 420)], "pwr", "6 V + PWM", dy=-6)
    s.wire([(1480, 600), (bx1 + 24, 600)], "sig", "TX", dy=-5)
    s.wire([(bx1 + 24, 614), (1480, 614)], "sig", "RX", dy=14)
    s.wire([(1480, 825), (bx1 + 24, 825)], "sig", "SWD", dy=-6)

    # ---------------- power band
    s.block(300, 272, 150, 56, "Q1, D1", "reverse-polarity P-FET\nTVS clamp", "prot")
    s.wire([(bx0 + 24, 300), (300, 300)], "pwr")
    s.block(300, 180, 150, 70, "F4, D2, U18, U19", "PTC + diode on VBAT\nLDO +5V_LOGIC\nLDO +5VA", "pwr")
    s.wire([(bx0 + 24, 215), (300, 215)], "pwr")
    s.wire([(450, 300), (475, 300)], "pwr", arrow=False)
    s.text(470, 133, "+6V_SERVO", 11, "middle", "bold", PWR)
    s.wire([(475, 150), (475, 440)], "pwr", arrow=False)
    s.dot(475, 300, "pwr")
    rows = [(150, "F1, RS1", "fuse + 10 mΩ shunt\nleft leg", "+6V_LL", "U4  INA226 0x44"),
            (300, "F2, RS2", "fuse + 10 mΩ shunt\nright leg", "+6V_RL", "U5  INA226 0x45"),
            (440, "F3, RS3", "fuse + 10 mΩ shunt\narms + neck", "+6V_ARM", "U6  INA226 0x46")]
    for y, ref, body, net, ina in rows:
        s.wire([(475, y), (520, y)], "pwr")
        s.block(520, y - 25, 150, 50, ref, body, "prot")
        s.block(520, y + 38, 150, 30, "", ina, "logic")
        s.wire([(595, y + 25), (595, y + 38)], "ana", dash=True, arrow=False)
    s.text(605, 186, "Kelvin", 9, color=ANA)
    # branch rails to the header blocks
    s.wire([(670, 150), (1200, 150)], "pwr", "+6V_LL  (≤ 8.2 A, shunt full scale)", dy=-6)
    s.wire([(670, 300), (1150, 300), (1150, 220), (1200, 220)], "pwr")
    s.text(800, 294, "+6V_RL", 10, color=PWR)
    s.wire([(670, 440), (1200, 440)], "pwr")
    s.text(1040, 434, "+6V_ARM", 10, color=PWR)
    s.block(1200, 120, 150, 120, "J3–J14",
            "12 leg servo headers\n2.54 mm, GND / V+ / S\n220 Ω on each S\nC_BULK × 2", "pwr")
    s.block(1200, 400, 150, 76, "J15–J22", "8 arm / neck headers\n220 Ω on each S\nC_BULK", "pwr")
    s.wire([(1350, 170), (bx1 - 24, 170)], "pwr", arrow=False)
    s.wire([(1350, 420), (bx1 - 24, 420)], "pwr", arrow=False)

    # PCA9685s between the rails
    s.block(990, 330, 150, 56, "U2  PCA9685", "0x40, legs ch 0–11\nOE ← OE_LEGS", "logic")
    s.block(990, 486, 150, 56, "U3  PCA9685", "0x41, arms + neck ch 0–7\nOE ← OE_ARMS", "logic")
    s.wire([(1140, 358), (1170, 358), (1170, 230), (1200, 230)], "sig", "PWM ×12", at=1, dy=0)
    s.wire([(1140, 514), (1175, 514), (1175, 466), (1200, 466)], "sig", "PWM ×8", at=1, dy=0)

    # trip latch
    s.block(760, 330, 160, 70, "U15–U17", "trip latch, powers up tripped\nOE = MCU_OE OR TRIP", "prot")
    s.wire([(670, 203), (720, 203), (720, 493), (670, 493)], "sig", arrow=False)
    s.text(726, 250, "ALERT\n(wired-OR,\nopen drain)", 10, color=SIG)
    s.wire([(670, 353), (720, 353)], "sig", arrow=False)
    s.dot(720, 353)
    s.wire([(720, 365), (760, 365)], "sig")
    s.dot(720, 365)
    s.wire([(920, 350), (990, 350)], "sig", "OE_LEGS", dy=-5)
    s.wire([(920, 385), (950, 385), (950, 506), (990, 506)], "sig")
    s.text(880, 470, "OE_ARMS", 10, color=SIG)

    # ---------------- logic band
    s.block(760, 600, 220, 210, "U1  Black Pill (socketed)",
            "STM32F411CEU6, AP7343 3.3 V\n\nPB6/PB7    I²C1\nPB10/PB3   I²C2\nPB0/PB1    OE legs / arms\n"
            "PA0        mux SIG (ADC1_IN0)\nPA1/PA2    NTC L / R\nPA3        VBAT_SNS\nPB12–PB15  mux S0–S3\n"
            "PA9/PA10   USART1 → J27\nTRIP_CLR, TRIP_STATE: step 6", "mcu", mono=True)
    # I2C1 bus to the PCAs
    s.wire([(970, 600), (970, 370), (990, 370)], "bus", arrow=False)
    s.text(985, 585, "I²C1 400 kHz", 10, color=BUS)
    s.wire([(970, 524), (990, 524)], "bus", arrow=False)
    s.dot(970, 524, "bus")
    # I2C2 bus to the INA226s and the IMU
    s.wire([(800, 600), (800, 560), (500, 560), (500, 203), (520, 203)], "bus", "I²C2  400 kHz", at=1, dy=-6,
           arrow=False)
    s.wire([(500, 353), (520, 353)], "bus", arrow=False)
    s.wire([(500, 493), (520, 493)], "bus", arrow=False)
    s.dot(500, 353, "bus")
    s.dot(500, 493, "bus")
    s.block(1060, 640, 170, 56, "U7  MPU6050", "0x68, board centre\naxes = base_link", "logic")
    s.wire([(800, 560), (1040, 560), (1040, 668), (1060, 668)], "bus", arrow=False)
    s.dot(800, 560, "bus")
    # MCU control to the latch
    s.wire([(860, 600), (860, 400)], "sig", "OE ×2, TRIP_CLR,\nTRIP_STATE", dy=0)
    # analog chain
    s.block(300, 560, 150, 60, "U8–U13", "6 × LMV358\n12 wiper followers", "ana")
    s.block(520, 560, 150, 74, "U14", "CD74HC4067\n16:1 analog mux\nch 0–11 pots, 12–15 FSR", "ana")
    s.block(300, 660, 150, 120, "R/C network",
            "FSR dividers × 4\nNTC dividers × 2\nseries R + clamps\non every ADC input", "ana")
    s.wire([(bx0 + 24, 575), (300, 575)], "ana")
    s.wire([(bx0 + 24, 605), (300, 605)], "ana")
    s.wire([(bx0 + 24, 682), (300, 682)], "ana")
    s.wire([(bx0 + 24, 762), (300, 762)], "ana")
    s.wire([(450, 590), (520, 590)], "ana", "12", dy=-5)
    s.wire([(450, 690), (595, 690), (595, 634)], "ana", "FSR ×4", dy=-5)
    s.wire([(670, 610), (760, 610)], "ana", "SIG → PA0", dy=-5)
    s.wire([(450, 750), (760, 750)], "ana", "NTC → PA1, PA2", dy=-5)
    s.wire([(760, 628), (670, 628)], "sig", "S0–S3", dy=14)
    # UART and SWD
    s.wire([(980, 620), (1300, 620), (1300, 607), (bx1 - 24, 607)], "sig", "USART1 921 600 Bd, 3.3 V", dy=-6,
           arrow=False)
    s.block(1060, 790, 170, 56, "J28", "SWD: SWDIO, SWCLK,\nNRST, 3V3, GND", "logic")
    s.wire([(980, 818), (1060, 818)], "sig", arrow=False)
    s.wire([(1230, 818), (1300, 818), (1300, 825), (bx1 - 24, 825)], "sig", arrow=False)
    # rail tags
    s.text(300, 860, "Rails: +5V_LOGIC → U1 5 V pin through a Schottky (USB back-feed);  +3V3 from U1 → U2–U7, "
                     "U14–U17;  +5VA → U8–U13.  Sheet 2 has the full power tree.", 11, color="#555")
    s.legend(30, 930, [("pwr", "power, servo current"), ("sig", "logic signal, arrow = direction"),
                       ("ana", "analog signal"), ("bus", "I²C bus"), ("kelvin", "Kelvin sense"),
                       ("dot", "junction (crossing without dot = no connection)")])
    s.legend(450, 930, [("conn", "connector on the board edge"), ("prot", "protection"),
                        ("off", "off the board (module or cable)")])
    s.title_block()
    s.save(f"{DOC}_s1.svg")


def sheet2():
    s = Sheet("Power tree", 2, 2)
    s.text(30, 45, "BZN-MB rev A: power tree", 22, weight="bold")
    s.text(30, 68, "Current figures: worst case per the requirements. Fuse and part ratings marked TBD are sized "
                   "in step 5 against these numbers.", 12, color="#555")

    def node(x, y, ref, body, kind, w=210, h=64):
        s.block(x, y, w, h, ref, body, kind)
        return x, y, w, h

    node(40, 480, "BT1", "2S LiPo 1100 mAh 30C\n7.4 V nom, 6.0–8.4 V\n≤ 33 A by C rating", "off", 200, 76)
    node(300, 486, "F_MAIN + SW1", "fuse 15 A, switch\nbattery lead", "prot", 180, 64)
    s.wire([(240, 518), (300, 518)], "pwr", "VBAT", dy=-6)
    s.wire([(480, 518), (530, 518)], "pwr", arrow=False)
    s.wire([(530, 170), (530, 772)], "pwr", arrow=False)
    s.dot(530, 518, "pwr")
    # UBEC branch
    node(580, 140, "UBEC (module)", "VBAT → 6.0 V\n8 A cont., 15 A burst", "off", 200, 64)
    s.wire([(530, 170), (580, 170)], "pwr")
    node(840, 140, "J1, Q1, D1", "XT30, reverse-polarity\nP-FET, TVS", "prot", 180, 64)
    s.wire([(780, 170), (840, 170)], "pwr", "+6V", dy=-6)
    s.wire([(1020, 170), (1060, 170)], "pwr", arrow=False)
    s.text(1066, 128, "+6V_SERVO  ≤ 8.5 A cont., 15 A 1 s", 11, weight="bold", color=PWR)
    s.wire([(1060, 170), (1060, 370)], "pwr", arrow=False)
    s.dot(1060, 170, "pwr")
    for y, ref, load in ((150, "F1 (TBD) + RS1 10 mΩ", "+6V_LL → 6 × MG92B left leg\nstall 6 × 0.85 = 5.1 A"),
                         (250, "F2 (TBD) + RS2 10 mΩ", "+6V_RL → 6 × MG92B right leg\nstall 5.1 A"),
                         (350, "F3 (TBD) + RS3 10 mΩ", "+6V_ARM → 2 MG90S + 6 SG90\nstall TBD (P0)")):
        s.wire([(1060, y + 20), (1100, y + 20)], "pwr")
        s.block(1100, y, 190, 44, "", ref, "prot")
        s.wire([(1290, y + 20), (1330, y + 20)], "pwr")
        s.block(1330, y, 330, 44, "", load, "off")
    # buck branch
    node(580, 600, "BUCK (module)", "VBAT → 5.0 V, 3 A", "off", 200, 56)
    s.wire([(530, 628), (580, 628)], "pwr")
    s.wire([(780, 628), (840, 628)], "pwr", "+5V_RADXA", dy=-6)
    s.block(840, 600, 260, 56, "Radxa Zero 3W", "≥ 2 A recommended; never through\nthe mainboard (P-6)", "off")
    # logic branch
    node(580, 740, "J2, F4, D2", "VBAT lead, PTC (TBD)\nseries diode", "prot", 200, 64)
    s.wire([(530, 772), (580, 772)], "pwr")
    s.wire([(780, 772), (820, 772)], "pwr", arrow=False)
    s.wire([(820, 725), (820, 882)], "pwr", arrow=False)
    s.dot(820, 772, "pwr")
    s.block(860, 700, 200, 50, "U18  LDO (TBD)", "VBAT → +5V_LOGIC, ≥ 16 V in", "pwr")
    s.block(860, 780, 200, 50, "U19  LDO (TBD)", "VBAT → +5VA, ≥ 16 V in", "pwr")
    s.block(860, 860, 200, 44, "R_DIV", "VBAT → VBAT_SNS (PA3)", "ana")
    s.wire([(820, 725), (860, 725)], "pwr")
    s.wire([(820, 805), (860, 805)], "pwr")
    s.wire([(820, 882), (860, 882)], "ana")
    s.block(1100, 700, 160, 50, "D3  Schottky", "blocks USB back-feed", "prot")
    s.wire([(1060, 725), (1100, 725)], "pwr", "60 mA", dy=-6)
    s.block(1300, 690, 200, 70, "U1  Black Pill", "5 V pin → AP7343\n→ +3V3, 300 mA max", "mcu")
    s.wire([(1260, 725), (1300, 725)], "pwr")
    s.block(1300, 790, 360, 64, "+3V3 loads ≈ 55 mA",
            "U1 30, U2+U3 12, U4–U6 1, U7 4,\nU14–U17 + pull-ups 6, LEDs 2", "logic")
    s.wire([(1400, 760), (1400, 790)], "pwr")
    s.block(1100, 780, 160, 50, "U8–U13", "6 × LMV358 ≈ 3 mA", "ana")
    s.wire([(1060, 805), (1100, 805)], "pwr", "+5VA", dy=-6)
    s.text(40, 960, "Ground: battery negative is the single star point; UBEC, buck and board grounds meet there. "
                    "The VBAT lead carries no ground wire, so it never carries servo return current.", 12, color="#555")
    s.legend(40, 600, [("pwr", "power"), ("ana", "sense"), ("prot", "protection"), ("off", "module or load"),
                       ("dot", "junction")])
    s.title_block()
    s.save(f"{DOC}_s2.svg")


if __name__ == "__main__":
    sheet1()
    sheet2()
