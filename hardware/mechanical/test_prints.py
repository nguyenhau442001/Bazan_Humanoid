"""Test prints before the real build: a hole coupon and the first real parts, to check the printer, the fits and
the servos you bought before printing the whole robot.

    python test_prints.py          # writes out/test_print/*.stl and .step

    T0  hole_coupon   every hole kind of hardware.HOLES, exactly as the real parts print them (20 min)
    T1  L_hip_block   two MG92B pockets with ear inserts and the M3 idler inserts (hip roll + hip pitch)
    T2  L_foot        horn plate with its horn recess, MR63ZZ seat, the 4 sole inserts
    T3  L_sole        TPU 95A sole: counterbores, screw clamping in TPU

What to check on each print, and the measuring sheet for the servos: docs/mechanical/test_print.md, written by
this script from the same numbers (python test_prints.py --check fails if it is stale).
"""
import sys
from pathlib import Path

import cadquery as cq

import hardware as hw
import joint as J
import leg as L
import servo_fit as SF

HERE = Path(__file__).resolve().parent
OUT = HERE / "out" / "test_print"
DOC = HERE.parents[1] / "docs" / "mechanical" / "test_print.md"
FIG = DOC.with_suffix("")                      # docs/mechanical/test_print/*.png

# Coupon: a 3 mm zone that prints like the decks and sleds (through holes, inserts flush in a 3 mm plate) and a
# 6 mm zone that prints like the joint parts (blind insert holes, bearing seat). One hole per kind, label engraved.
THIN, THICK = 3.0, 6.0
PITCH = 11.0
ROW_THIN = ["clr_m2", "cb_m2", "clr_so", "ins_m2_thru", "ins_m3_thru", "tie"]
ROW_THICK = ["pilot_st2", "ins_m2", "ins_m3", "boss_m2", "bearing", "access"]
LETTERS = {k: chr(ord("A") + i) for i, k in enumerate(ROW_THIN + ROW_THICK)}   # engraved next to each hole


def _label(text, x, y, z):
    return (cq.Workplane("XY").workplane(offset=z).center(x, y)
            .text(text, 2.6, -0.4, kind="bold", halign="center", valign="center").val())


def hole_coupon():
    n = max(len(ROW_THIN), len(ROW_THICK))
    w = n * PITCH + 4.0
    thin = cq.Workplane().box(w, 14.0, THIN, centered=(False, False, False)).val()
    thick = cq.Workplane().box(w, 14.0, THICK, centered=(False, False, False)).translate((0, 14.0, 0)).val()
    s = thin.fuse(thick)
    for row, y0, h in ((ROW_THIN, 0.0, THIN), (ROW_THICK, 14.0, THICK)):
        for i, kind in enumerate(row):
            spec = hw.HOLES[kind]
            x, y = 2.0 + PITCH / 2 + i * PITCH, y0 + 5.5
            if kind == "tie":                                     # 3.0 × 1.6 slot
                s = s.cut(cq.Solid.makeBox(3.0, 1.6, h + 2, cq.Vector(x - 1.5, y - 0.8, -1)))
            else:
                depth = spec.depth if spec.depth else h + 2
                z0 = h - depth if spec.depth else -1
                s = s.cut(cq.Solid.makeCylinder(spec.d / 2, depth + (0 if spec.depth else 0) + 0.01,
                                                cq.Vector(x, y, z0 + 0.0)))
                if spec.cb_d:
                    s = s.cut(cq.Solid.makeCylinder(spec.cb_d / 2, spec.cb_h + 0.01, cq.Vector(x, y, h - spec.cb_h)))
            s = s.cut(_label(LETTERS[kind], x, y0 + 11.6, h))
    return s.clean()


def on_bed(s, rot_x=0.0):
    """The part as it goes on the print bed (orientation from leg_v3.md), lowest point at z = 0."""
    if rot_x:
        s = s.rotate((0, 0, 0), (1, 0, 0), rot_x)
    bb = s.BoundingBox()
    return s.translate((-bb.center.x, -bb.center.y, -bb.zmin))


def parts():
    leg = L.Leg()
    return {"T0_hole_coupon": on_bed(hole_coupon()),
            "T1_L_hip_block": on_bed(leg.parts["hip_block"], 90.0),     # idler wall of the hip pitch pocket down
            "T2_L_foot": on_bed(leg.parts["foot"]),                     # sole plate down, uprights vertical
            "T3_L_sole": on_bed(leg.parts["sole"])}                     # flat


# ---------------------------------------------------------------------------------------------- the sheet
COUPON_TEST = {   # kind: (what goes in, pass, if it fails)
    "clr_m2": ("ốc M2", "ốc M2 lọt qua bằng tay, không phải vặn", "chặt: tăng XY hole compensation 0.05–0.1"),
    "cb_m2": ("ốc lục giác M2×6", "đầu ốc chìm bằng hoặc thấp hơn mặt", "đầu ốc lồi: báo lại số đo Ø đầu ốc"),
    "clr_so": ("ren đực trụ nhựa M3", "ren đực lọt qua bằng tay", "chặt: tăng XY hole compensation"),
    "ins_m2_thru": ("tán cấy M2 (ép từ mặt trên)", "tán bằng mặt trên ±0.2, không nghiêng; mặt DƯỚI không phồng "
                    "quá 0.2 mm", "phồng dưới: báo lại; tầng 3 mm sẽ cần gờ 4 mm quanh lỗ"),
    "ins_m3_thru": ("tán cấy M3 (ép từ mặt trên)", "tán bằng mặt, ốc M3 vặn vào thẳng", "tán lỏng/xoay: giảm "
                    "compensation; nghiêng: ép chậm hơn"),
    "tie": ("dây rút 2.5 mm", "dây rút luồn qua được", "không lọt: mở rộng khe trong slicer"),
    "pilot_st2": ("vít tự ren ST2×4", "vít ăn chặt, nhựa không nứt", "nứt: báo lại (lỗ mồi sẽ tăng lên Ø1.7)"),
    "ins_m2": ("tán cấy M2 + ốc M2×6", "tán bằng mặt; ốc M2×6 vặn hết, giữ chặt, không chạm đáy", "chạm đáy: báo "
               "lại độ sâu"),
    "ins_m3": ("tán cấy M3 + ốc đầu dù M3×6", "tán bằng mặt; ốc vặn thẳng", "như E"),
    "boss_m2": ("tán cấy M2 (lỗ sâu 5)", "tán bằng mặt", "như H"),
    "bearing": ("bạc đạn MR63ZZ", "ép vào bằng ngón cái hoặc ê-tô, bằng mặt, lật ngược không rơi, vòng trong quay "
                "êm", "quá lỏng: giảm compensation; quá chặt: doa nhẹ bằng mũi 6.0"),
    "access": ("tua vít PH0", "tua vít lọt tự do", "chặt: tăng compensation"),
}
FIELDS = [  # field, how to measure (numbers match docs/mechanical/test_print/servo_measure.png)
    ("length", "[1] dài vỏ (không tính tai, không tính dây)"),
    ("tab_span", "[2] tai: đầu này đến đầu kia"),
    ("case_h", "[3] cao vỏ: đáy đến mặt trên vỏ (không tính tháp bánh răng)"),
    ("tab_low", "[4] đáy vỏ đến MẶT DƯỚI tai"),
    ("tab_t", "[5] dày tai"),
    ("hole_pitch", "[6] tâm lỗ tai đến tâm lỗ tai (đo mép trong + Ø lỗ)"),
    ("shaft_off", "[7] tâm trục đến đầu vỏ GẦN trục"),
    ("width", "[8] rộng vỏ"),
    ("tower_h", "[9] tháp bánh răng cao hơn mặt vỏ"),
    ("spline_top", "[10] mặt vỏ đến đỉnh trục then hoa"),
    ("hole_d", "[11] Ø lỗ tai (thử bằng chuôi mũi khoan 2.0 / 2.2 / 2.3)"),
    ("tower_d", "Ø tháp bánh răng"),
    ("cable_z", "[12] đáy vỏ đến mép dưới / mép trên chỗ dây ra (2 số)"),
    ("mass", "khối lượng (g), cân tiểu ly"),
]


def _fmt(v):
    return f"{v[0]:g}–{v[1]:g}" if isinstance(v, tuple) else f"{v:g}"


def markdown():
    leg = L.Leg()
    j = leg.joints["hip_pitch"]
    hb = leg.parts["hip_block"].BoundingBox()
    bed = hb.ymin                                   # the -y face goes on the bed
    spline_h = j.axis_point(J.S.spline_top)[1] - bed
    a = []
    w = a.append
    w("# Bazan · In thử và đo servo (bước 1)")
    w("")
    w("Sinh tự động bởi `hardware/mechanical/test_prints.py` từ `hardware.py`, `joint.py`, `leg.py` — không sửa tay. "
      "Đơn vị mm.")
    w("")
    w("Mục đích: trước khi in cả robot, kiểm tra 3 thứ bằng ít nhựa nhất: (1) máy in ra lỗ đúng cỡ, (2) servo thật "
      "có đúng số trong `hardware.py`, (3) ổ servo, tán cấy, bạc đạn, horn lắp được thật. Tổng in khoảng 2 giờ.")
    w("")
    w("## Thứ tự")
    w("")
    w("1. **Đo servo** (mục 1) — 1 con mỗi loại MG92B, MG90S, SG90 và horn tròn của từng loại. Nhập số vào "
      "`hardware.py`, xóa tên servo đó khỏi `UNMEASURED`.")
    w("2. Chạy lại (PowerShell, trong `hardware\\mechanical`): `py servo_fit.py`, `py test_prints.py`, "
      "`py check.py --quick`, `py robot.py --quick`.")
    w("3. **In T0** (phiếu thử lỗ, 20 phút). Sai > 0.1 mm thì chỉnh XY hole compensation trong slicer, KHÔNG sửa file. "
      "In lại T0 đến khi đạt.")
    w("4. **In T1, T2, T3** và thử lắp (mục 3).")
    w("5. Gửi lại kết quả theo mục 4.")
    w("")
    w("Cài đặt in: PLA, đầu 0.4, lớp 0.2, T1/T2 4 viền 40 % infill, T0 3 viền 40 %, T3 TPU 95A 3 viền 30 % gyroid. "
      "Không bật hole compensation (trừ khi T0 bảo). File: `hardware/mechanical/out/test_print/T*.stl`, "
      "đã xoay sẵn đúng hướng in, đặt thẳng lên bàn.")
    w("")
    w("## 1. Đo servo")
    w("")
    w("![đo servo](test_print/servo_measure.png)")
    w("")
    w("Thước kẹp, đo mỗi số 2 lần lấy trung bình, ghi 2 số lẻ. Cột datasheet là số đang dùng trong CAD.")
    w("")
    names = list(hw.SERVOS)
    w("| Trường | Đo gì | " + " | ".join(f"{n} datasheet | {n} đo" for n in names) + " |")
    w("| --- | --- | " + " | ".join("---: | ---:" for _ in names) + " |")
    for f, how in FIELDS:
        w(f"| `{f}` | {how} | " + " | ".join(f"{_fmt(getattr(hw.SERVOS[n], f))} |  " for n in names) + " |")
    w("")
    w(f"Horn tròn (mỗi loại servo 1 cái): Ø đĩa (`disc_d`, CAD {hw.HORN.disc_d:g}), dày đĩa (`disc_t`, "
      f"{hw.HORN.disc_t:g}), mặt vỏ đến MẶT DƯỚI đĩa khi ép hết vào trục (`disc_low`, {hw.HORN.disc_low:g}). "
      "Horn của SG90 thường khác MG92B: nếu khác, ghi riêng.")
    w("")
    w("Kết quả `servo_fit.py` với số datasheet hiện tại (ổ servo chung dựng theo MG92B):")
    w("")
    w("| Servo | Kiểm tra không đạt | Giá trị | Cần |")
    w("| --- | --- | ---: | --- |")
    nfail = 0
    for t in hw.SERVOS.values():
        for ch in SF.checks(t):
            if not ch.ok:
                nfail += 1
                w(f"| {t.name} | {ch.what} | {ch.value:+.2f} | {ch.lo:+.2f}…{ch.hi:+.2f} |")
    if not nfail:
        w("| – | tất cả đạt | | |")
    w("")
    w("Lỗ tai servo Ø2.0 không cho ốc M2 (Ø2.0) lọt: khoan lỗ tai lên Ø2.3 bằng tay cho mọi servo là bình thường. "
      "Với SG90, theo số datasheet, tâm lỗ tai lệch tán cấy tới 0.4 mm, nên đo kỹ `hole_pitch` và `shaft_off`.")
    w("")
    w("## 2. T0 · Phiếu thử lỗ")
    w("")
    w("![phiếu thử lỗ](test_print/coupon.png)")
    w("")
    w("Hàng A–F dày 3 mm (in như tầng thân và đế mạch), hàng G–L dày 6 mm (in như chi tiết khớp). Mỗi lỗ in đúng như "
      "trong chi tiết thật (`hardware.HOLES`).")
    w("")
    w("| Lỗ | Loại | Ø in | Sâu | Thử bằng | Đạt khi | Không đạt thì |")
    w("| --- | --- | ---: | --- | --- | --- | --- |")
    for k in ROW_THIN + ROW_THICK:
        sp = hw.HOLES[k]
        what, ok, fix = COUPON_TEST[k]
        d = "3.0 × 1.6" if k == "tie" else f"{sp.d:g}" + (f" (khoét Ø{sp.cb_d:g} sâu {sp.cb_h:g})" if sp.cb_d else "")
        depth = f"{sp.depth:g}" if sp.depth else "xuyên"
        w(f"| **{LETTERS[k]}** | `{k}` | {d} | {depth} | {what} | {ok} | {fix} |")
    w("")
    w("## 3. T1, T2, T3 · Thử lắp")
    w("")
    w("### T1 · `hip_block` (2 ổ servo: hip pitch và hip roll)")
    w("")
    w("1. Cấy 4 tán M2 (tai servo) và 2 tán M3 (idler), 220 °C, bằng mặt.")
    w("2. Lần lượt đẩy **MG92B, MG90S, SG90** vào ổ hip pitch từ phía trục, dây luồn qua rãnh cuối ổ. Đạt: trượt vào "
      "bằng tay, lắc ngang ≤ 0.3 mm, tai nằm phẳng trên 2 gờ (không kênh).")
    w("3. Bắt 2 × M2×6 qua tai vào tán. Đạt: ốc vào thẳng không phải ép, servo không bị kéo lệch.")
    w(f"4. **Đo vị trí trục** (servo đang bắt ốc, khối nằm mặt in xuống bàn): bàn → đỉnh trục then hoa = "
      f"**{spline_h:.1f} ± 0.3**. Lệch hơn thì ghi lại cho từng "
      "loại servo: đây là lỗi đã thấy giữa các model MG90S (độ cao tai lệch tới 2.7 mm).")
    w("5. Lặp với ổ hip roll (trục hướng ra mặt bên).")
    w("")
    w("### T2 · `foot` (đĩa horn ankle roll, ổ bạc đạn, 4 tán đế)")
    w("")
    w("1. Ép bạc đạn MR63ZZ vào ổ idler. Đạt: như lỗ K của T0.")
    w(f"2. Khoan 2 lỗ horn ở bán kính {hw.HORN.screw_r:g} lên Ø2.1, đặt horn tròn vào hốc (Ø20.4), bắt 2 × ST2×4. "
      "Đạt: horn nằm phẳng đáy hốc, không cấn.")
    w("3. Cấy 4 tán M2 ở 4 góc bàn chân.")
    w("")
    w("### T3 · `sole` (TPU 95A)")
    w("")
    w("1. Bắt đế vào T2 bằng 4 × M2×6 từ dưới. Đạt: đầu ốc chìm, đế phẳng, không vênh mép.")
    w("2. Ấn mạnh ngón cái quanh đầu ốc: TPU dưới đầu ốc chỉ còn khoảng 1 mm, xem có rách/tụt không. Nếu có: báo "
      "lại (sẽ đổi sang long đền hoặc khoét nông hơn).")
    w("")
    w("## 4. Gửi lại")
    w("")
    w("- Bảng số đo mục 1 (cả horn).")
    w("- T0: đạt/không đạt từng chữ A–L, và XY hole compensation cuối cùng đã dùng.")
    w("- T1: kết quả bước 2–4 cho từng loại servo, nhất là con số đỉnh trục.")
    w("- T2, T3: đạt/không đạt, ảnh chụp chỗ không đạt.")
    w("")
    return "\n".join(a) + "\n"


def figures():
    """The two pictures of the sheet: the coupon from above and the servo measuring diagram."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    FIG.mkdir(parents=True, exist_ok=True)
    # --- coupon top view
    fig, ax = plt.subplots(figsize=(9, 4))
    n = max(len(ROW_THIN), len(ROW_THICK))
    w = n * PITCH + 4
    ax.add_patch(Rectangle((0, 0), w, 14, fc='#f6d7b8', ec='k'))
    ax.add_patch(Rectangle((0, 14), w, 14, fc='#e9b98a', ec='k'))
    ax.text(w + 1, 7, 'dày 3 mm\n(như tầng, đế mạch)', va='center', fontsize=8)
    ax.text(w + 1, 21, 'dày 6 mm\n(như chi tiết khớp)', va='center', fontsize=8)
    for row, y0 in ((ROW_THIN, 0), (ROW_THICK, 14)):
        for i, k in enumerate(row):
            sp = hw.HOLES[k]
            x = 2 + PITCH / 2 + i * PITCH
            y = y0 + 5.5
            if k == 'tie':
                ax.add_patch(Rectangle((x - 1.5, y - 0.8), 3, 1.6, fc='w', ec='k'))
            else:
                if sp.cb_d:
                    ax.add_patch(Circle((x, y), sp.cb_d / 2, fc='#ddd', ec='k', lw=.6))
                ax.add_patch(Circle((x, y), sp.d / 2, fc='w', ec='k'))
            ax.text(x, y0 + 11.6, LETTERS[k], ha='center', va='center', fontsize=9, weight='bold')
            lab = f"Ø{sp.d:g}" if k != 'tie' else '3×1.6'
            ax.text(x, y0 + 1.4, lab, ha='center', va='center', fontsize=6.5)
    ax.set_xlim(-1, w + 18)
    ax.set_ylim(-1, 29)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('T0 · Phiếu thử lỗ (nhìn từ trên, mm)', fontsize=11)
    plt.tight_layout()
    plt.savefig(str(FIG / 'coupon.png'), dpi=140)
    plt.close()
    # --- servo measuring diagram (side + top), MG92B numbers as example
    S = hw.MG92B
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.6))
    L, W, H = S.length, S.width, S.case_h
    so = S.shaft_off
    a1.add_patch(Rectangle((0, 0), L, H, fc='#cfd8e3', ec='k'))
    el = (L - S.tab_span) / 2
    a1.add_patch(Rectangle((el, S.tab_low), S.tab_span, S.tab_t, fc='#9fb3c8', ec='k'))
    a1.add_patch(Rectangle((so - S.tower_d / 2, H), S.tower_d, S.tower_h, fc='#cfd8e3', ec='k'))
    a1.add_patch(
        Rectangle(
            (so -
             S.spline_d /
             2,
             H +
             S.tower_h),
            S.spline_d,
            S.spline_top -
            S.tower_h,
            fc='#888',
            ec='k'))
    a1.add_patch(Rectangle((L, S.cable_z[0]), 2.5, S.cable_z[1] - S.cable_z[0], fc='#e07b39', ec='k'))

    def dim(ax, x0, y0, x1, y1, t, off=(0, 0), c='C3'):
        ax.annotate('', (x0, y0), (x1, y1), arrowprops=dict(arrowstyle='<->', color=c, lw=1))
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], t, color=c, fontsize=8, ha='center', va='center',
                bbox=dict(fc='w', ec='none', pad=0.5))
    dim(a1, 0, -2, L, -2, '[1] length', (0, -1.3))
    dim(a1, el, -5.5, el + S.tab_span, -5.5, '[2] tab_span', (0, -1.3))
    dim(a1, -2, 0, -2, H, '[3] case_h', (-3.5, 0))
    dim(a1, -6, 0, -6, S.tab_low, '[4] tab_low', (-3.5, 0))
    dim(a1, L + 5, S.tab_low, L + 5, S.tab_low + S.tab_t, '[5] tab_t', (3.5, 0))
    dim(a1, 0, H + 11, so, H + 11, '[7] shaft_off', (0, 1.3))
    dim(a1, L + 9, H, L + 9, H + S.spline_top, '[10] spline_top', (4.5, 0))
    dim(a1, so + S.tower_d / 2 + 1, H, so + S.tower_d / 2 + 1, H + S.tower_h, '[9] tower_h', (4, 0))
    dim(a1, L + 3, 0, L + 3, S.cable_z[1], '[12] cable_z (max)', (5.5, -3))
    a1.axvline(so, color='k', ls='--', lw=.6)
    a1.set_xlim(-14, L + 22)
    a1.set_ylim(-9, H + 15)
    a1.set_aspect('equal')
    a1.axis('off')
    a1.set_title('Nhìn ngang (dây ra bên phải)', fontsize=10)
    # top view
    a2.add_patch(Rectangle((0, 0), L, W, fc='#cfd8e3', ec='k'))
    a2.add_patch(Rectangle((el, 0), S.tab_span, W, fc='none', ec='k', ls=':'))
    mid = L / 2
    for h in (mid - S.hole_pitch / 2, mid + S.hole_pitch / 2):
        a2.add_patch(Circle((h, W / 2), S.hole_d / 2, fc='w', ec='k'))
    a2.add_patch(Circle((so, W / 2), S.tower_d / 2, fc='#b8c6d6', ec='k'))
    a2.add_patch(Circle((so, W / 2), S.spline_d / 2, fc='#888', ec='k'))
    dim(a2, mid - S.hole_pitch / 2, -2.5, mid + S.hole_pitch / 2, -2.5, '[6] hole_pitch (tâm–tâm)', (0, -1.3))
    dim(a2, -2, 0, -2, W, '[8] width', (-3.5, 0))
    a2.text(mid + S.hole_pitch / 2, W + 2.2, '[11] hole_d', fontsize=8, color='C3', ha='center')
    a2.text(so, W + 2.2, 'tower_d', fontsize=8, color='C3', ha='center')
    a2.set_xlim(-10, L + 10)
    a2.set_ylim(-7, W + 5)
    a2.set_aspect('equal')
    a2.axis('off')
    a2.set_title('Nhìn từ trên', fontsize=10)
    plt.suptitle('Đo servo: 12 số, tên trường đúng như trong hardware.py (ví dụ vẽ theo MG92B)', fontsize=11)
    plt.tight_layout()
    plt.savefig(str(FIG / 'servo_measure.png'), dpi=140)
    plt.close(fig)


def main():
    if "--check" in sys.argv:
        if not DOC.exists() or DOC.read_text(encoding="utf-8") != markdown():
            print("docs/mechanical/test_print.md is stale: run python test_prints.py")
            return 1
        print("up to date")
        return 0
    OUT.mkdir(parents=True, exist_ok=True)
    for n, s in parts().items():
        bb = s.BoundingBox()
        cq.exporters.export(s, str(OUT / f"{n}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(s, str(OUT / f"{n}.step"))
        print(f"  {n:16s} {bb.xlen:5.1f} x {bb.ylen:5.1f} x {bb.zlen:5.1f} mm  {s.Volume() / 1000:5.2f} cm3"
              f"  valid {s.isValid()}")
    figures()
    DOC.write_text(markdown(), encoding="utf-8")
    print("wrote", OUT, "and", DOC)
    return 0


if __name__ == "__main__":
    sys.exit(main())
