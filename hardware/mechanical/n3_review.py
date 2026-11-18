"""Engineering review of N3 (head_frame) and the head-pitch joint, read-only: it changes no CAD.

Every number in docs/mechanical/n3_review/*.md comes from this script, so the review can be rerun after each
measurement or design change:

    python n3_review.py            # all checks, writes docs/mechanical/n3_review/n3_review.json
    python n3_review.py --no-fea --no-sweep --no-slice   # quick hand checks only; does not rewrite the JSON

Each finding carries a status:
    VERIFIED  follows from the CAD source or a manufacturer tolerance, no assumption
    ASSUMED   computed from values marked MEASURE in hardware.py or from an assumed material property
    UNKNOWN   needs a physical measurement or a decision before it can be computed
    FAILED    the check fails on the CAD/BOM values themselves (or on every plausible input)
"""
import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

import hardware as hw
import joint as J

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "mechanical" / "n3_review"

# Values that are not in the CAD source. Each one names where it comes from.
SRC = {
    "MR63ZZ": "MR63ZZ 3×6×2.5, OD tol 0/−0.008, width tol 0/−0.12 (vendor datasheets TFL, SKD; ISO normal class)",
    "SG90_DS": "SG90 sizes disagree between datasheets: 22.2×11.8, 23.1×12.2, 22.0×11.5 mm (TowerPro copies)",
    "SG90_TQ": "SG90 stall torque: 1.8 kgf·cm (TowerPro copy) and 1.2 kgf·cm at 4.8 V (other copies)",
    "INS": "M2 heat-set inserts sold for a Ø3.2 hole have a knurl OD of about 3.6–3.7 (e.g. TFC: L4, W3.7, "
           "hole 3.2, min wall 1.6; Accu HTFBI-M2: L3.9, hole 3.2 +0.1/−0)",
    "PLA_E": "printed PLA E = 2300 MPa, ν = 0.35 (assumed, same as the leg FEA; real value depends on infill/walls)",
    "MG90S_STEP": "vendor MG90S STEP in cots/: tower 5.75 (CAD 4.0), spline top 9.7 (CAD 8.3) — see test_print.md",
}
G = 9.81


def finding(fid, sev, status, title, evidence, numbers=None, action=""):
    return dict(id=fid, severity=sev, status=status, title=title, evidence=evidence, numbers=numbers or {},
                action=action)


# ------------------------------------------------------------------ geometry of the stack (all from joint.py)
def stack():
    h = hw.HORN
    pilot_bottom = J.HP[1] - 0.4
    return dict(
        horn_plate=J.HP, idler_plate=J.IP, idler_wall=J.IW, bearing=J.BRG, boss=J.BOSS, lip=(J.BRG[1], J.IP[1]),
        gap_inner=round(J.HP[0] - J.IP[1], 3), horn_top=J.HORN_TOP, head_top=J.HEAD_TOP,
        recess_d=h.disc_d + 0.4, recess_floor=J.HORN_TOP, plate_t_horn=round(J.HP[1] - J.HP[0], 3),
        plate_t_idler=round(J.IP[1] - J.IP[0], 3), pilot_depth=round(pilot_bottom - (J.HORN_TOP - 0.1), 3),
        pilot_bottom=round(pilot_bottom, 3), skin=0.4, seat_depth=round(J.BRG[1] - J.IP[0] - J.CAP_T, 3),
        holder_to_idler_plate=J.GAP, tower_to_horn_plate=round(J.HP[0] - J.S.tower_h, 3),
        horn_hub_above_tower=0.4)


# ------------------------------------------------------------------ A. inserts
def check_inserts():
    bom = hw.FASTENERS["INS-M2"].desc
    od = 3.2                                           # from the BOM text "3 long, 3.2 OD"
    hole = hw.HOLES["ins_m2_thru"].d
    interference = round((od - hole) / 2, 3)
    deck_t = 3.0
    # edge distance of the four shell inserts and the two cam inserts on the head deck (x, y in robot frame)
    import body as B
    hx, hy = B.Body.HEAD_X, B.Body.HEAD_Y
    pts = {"A(shell)": [(-23.0, 20.0), (-23.0, -20.0), (2.0, 20.0), (2.0, -20.0)],
           "B(cam)": [(12.0, -10.0), (12.0, 10.0)]}
    edge = {k: round(min(min(x - hx[0], hx[1] - x, y - hy[0], hy[1] - y) for x, y in v), 2) for k, v in pts.items()}
    wall_37 = {k: round(v - 3.7 / 2, 2) for k, v in edge.items()}
    return [finding(
        "A-1", "Critical", "FAILED", "Lỗ tán cấy M2 bằng đúng OD danh nghĩa của tán trong BOM",
        f"hardware.py: FASTENERS['INS-M2'] = '{bom}', HOLES['ins_m2_thru'].d = {hole}. Độ dôi hướng kính "
        f"danh nghĩa = ({od} − {hole})/2 = {interference} mm, tức là không có độ dôi. Tán M2 thương mại dùng lỗ "
        f"Ø3.2 có OD gai ≈ 3.6–3.7 [{SRC['INS']}], nên BOM và lỗ đang mô tả hai loại tán khác nhau.",
        dict(bom_od=od, hole_d=hole, radial_interference=interference, deck_t=deck_t, insert_l=3.0,
             edge_distance=edge, wall_if_od_3_7=wall_37),
        "Chọn MỘT mã tán cụ thể (có datasheet), ghi mã vào BOM, lấy Ø lỗ theo datasheet, xác nhận bằng phiếu "
        "thử (kéo tuột + mô-men xoay). Không đổi Ø lỗ trước khi có mã tán."),
        finding(
        "A-2", "Major", "UNKNOWN", "Tán L3 trong sàn 3 mm, lỗ xuyên: chưa có số liệu kéo tuột/xoay",
        f"Tán dài 3.0 = bề dày sàn 3.0, ép ngang mặt trên, lỗ xuyên nên nhựa chảy dồn xuống mặt dưới. Khoảng "
        f"cách tâm lỗ đến mép sàn: {edge} mm; với OD 3.7 còn thành {wall_37} mm (≥ 1.6 mm tối thiểu của TFC). "
        "Tải: ốc vỏ đầu M2×6 kéo tán ra theo hướng ép vào, là hướng yếu nhất của tán gai.",
        dict(edge_distance=edge, wall_if_od_3_7=wall_37),
        "Sau khi chọn mã tán: in phiếu sàn 3 mm, ép 4 tán, đo mô-men xoay tuột và lực kéo tuột (xem kế hoạch "
        "kiểm tra). Chỉ khi đó mới đánh giá được.")]


# ------------------------------------------------------------------ B/E. pivot alignment (servo shaft vs idler)
def shaft_offset(servo):
    """Where the servo shaft sits relative to the joint axis (the idler M3 insert) in the MG92B pocket.

    Returns the range of the shaft position along b and w when the ear screws hold the servo, and when they are
    loose (the case can move inside the pocket)."""
    P = J.S                                              # the pocket is built for this servo (MG92B)
    c = hw.CLEAR_SERVO
    pocket_b = (J.CB0 - c, J.CB + c)
    pocket_w = (-P.width / 2 - c, P.width / 2 + c)
    # shaft position when the case touches either end / side of the pocket
    b_lo = pocket_b[0] + servo.shaft_off
    b_hi = pocket_b[1] - (servo.length - servo.shaft_off)
    w_play = (pocket_w[1] - pocket_w[0] - servo.width) / 2
    # shaft position when the ear holes are centred on the inserts
    ear_mid = J.CMID                                     # insert pair centre along b
    servo_mid_to_shaft = servo.length / 2 - servo.shaft_off
    b_ears = ear_mid - servo_mid_to_shaft
    pitch_err = abs(P.hole_pitch - servo.hole_pitch) / 2
    return dict(b_pocket=(round(b_lo, 3), round(b_hi, 3)), w_pocket=(round(-w_play, 3), round(w_play, 3)),
                b_ears_centred=round(b_ears, 3), ear_pitch_error_each=round(pitch_err, 3))


def check_alignment(k_frame):
    sg = shaft_offset(hw.SG90)
    mg = shaft_offset(hw.MG92B)
    b_need = sg["b_ears_centred"]
    d_hi = math.hypot(abs(b_need) + sg["ear_pitch_error_each"], 0.0)
    out = [finding(
        "B-1", "Critical", "FAILED",
        "Trục ra SG90 không trùng trục idler: ổ và tán tai được dựng cho MG92B",
        "joint.py dựng ổ servo cho mọi khớp từ S = hw.MG92B (dòng `S = hw.MG92B`); head_pitch dùng SG90 "
        "(JOINT_SERVO). Với số datasheet SG90 trong hardware.py, khi tâm lỗ tai trùng tán cấy thì trục ra lệch "
        f"{b_need:+.2f} mm theo b so với trục idler (tán M3 trên neck_link). Trong ổ, trục có thể nằm ở "
        f"b ∈ {sg['b_pocket']} và w ∈ {sg['w_pocket']} mm. Với MG92B (ổ đúng loại), lệch = "
        f"{mg['b_ears_centred']:+.2f}.",
        dict(sg90=sg, mg92b=mg, misalignment_nominal=round(abs(b_need), 3),
             misalignment_worst_screwed=round(d_hi, 3)),
        "Đo SG90 thật (shaft_off, length, hole_pitch, width). Sau đó làm ổ + tán tai riêng cho SG90 trên "
        "neck_link (N2), là chi tiết khác, không phải N3. Hoặc dùng quy trình lắp tự định tâm (siết tai sau "
        "cùng), xem DFM/DFA.")]
    if k_frame:
        kx, kz = k_frame["k_b"], k_frame["k_w"]
        f_nom, f_hi = kx * abs(b_need), kx * d_hi
        mu, r_b = (0.2, 0.4), 2.5                          # plastic bushing friction, bushing radius (ASSUMED)
        t_fric = [round(m * f_hi * r_b, 1) for m in mu]
        out.append(finding(
            "B-2", "Major", "ASSUMED",
            "Lệch trục làm khung N3 bị ép cong liên tục: tải ngang lên trục ra SG90 vài N",
            f"FEA CalculiX (tet10, {k_frame['nodes']} nút; {SRC['PLA_E']}): giữ hốc horn, đặt 1 N lên ổ bạc đạn. "
            f"Độ cứng tương đối giữa hai ổ: k_b = {kx:.1f} N/mm, k_w = {kz:.1f} N/mm. Phần mềm nhất là sàn 3 mm, nơi "
            f"hai thanh đỡ cắm vào. Lệch {abs(b_need):.2f}–{d_hi:.2f} mm (B-1) ép khung với "
            f"{f_nom:.1f}–{f_hi:.1f} N tác dụng thường trực lên trục ra servo và bạc đạn (trọng lượng đầu chỉ "
            f"≈ {0.07125 * G:.2f} N). Ứng suất khung ≈ {k_frame['vm_per_N_b'] * f_hi:.1f} MPa, thấp, nhưng là tải "
            f"liên tục nên PLA bị rão. Ma sát thêm ở bạc trục ra (μ {mu}, r {r_b} mm, giả định) "
            f"≈ {t_fric} N·mm, tức khoảng 1–4 % mô-men kẹt.",
            dict(k_b=round(kx, 1), k_w=round(kz, 1), preload=(round(f_nom, 1), round(f_hi, 1)),
                 frame_stress=round(k_frame['vm_per_N_b'] * f_hi, 2), friction_torque=t_fric),
            "Mô-men không phải vấn đề chính. Vấn đề là tải ngang thường trực lên bạc nhựa và bánh răng ra của SG90 "
            "(làm mòn, tăng độ rơ), cộng với rão khung. Mục tiêu: lệch lắp ráp ≤ 0.1 mm (≈ 1.2 N). Đo độ rơ trục "
            "ra SG90 thật trước khi chốt."))
    out.append(finding(
        "B-3", "Critical", "UNKNOWN", "Đồng trục hốc horn ↔ ổ bạc đạn trên chính N3 (K3): chưa có số khả thi",
        "Hai ổ nằm trên hai đĩa cách nhau 31.9 mm. Theo hướng in hiện tại (sàn úp xuống), trục gật nằm NGANG: "
        "cả hai lỗ được tạo bởi cùng các lớp in, nên lệch theo Z chủ yếu do sai số bước lớp, lệch theo X do sai "
        "số XY. Độ tròn kém hơn (đỉnh lỗ là cầu/treo). Chưa có máy in, profile, phiếu đo nào được ghi lại, nên "
        "chưa thể ghi một con số dung sai có căn cứ.",
        {}, "Phiếu thử K3-coupon (kế hoạch kiểm tra): in 3 mẫu, đo đồng trục bằng chốt Ø6 xuyên qua hai lỗ hoặc "
            "đo tọa độ tâm. Dung sai K3 = giá trị đo được + biên, và phải ≤ độ rơ trục ra SG90 (B-2)."))
    return out


def frame_stiffness(work):
    """Relative stiffness of the bearing seat against the horn plate, CalculiX tet10, unit loads along b and w."""
    import cadquery as cq
    import gmsh
    import body as B
    body = B.Body()
    j = body.joints["head_pitch"]
    solid = body.parts["head_frame"]
    step = work / "n3.step"
    cq.exporters.export(solid, str(step))
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.importShapes(str(step))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", 1.6)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.4)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
    gmsh.model.mesh.generate(3)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    X = xyz.reshape(-1, 3)
    idx = {int(t): i for i, t in enumerate(tags)}
    et, _, enodes = gmsh.model.mesh.getElements(3)
    E = np.array([idx[int(n)] for n in enodes[list(et).index(11)]]).reshape(-1, 10)
    gmsh.finalize()
    a = np.array(j.R[:, 2])
    bdir = np.array(j.R[:, 0])
    wdir = np.array(j.R[:, 1])
    v = X - j.p
    t = v @ a
    r = np.linalg.norm(v - np.outer(t, a), axis=1)
    horn = (t > J.HP[0] - 0.05) & (t < J.HORN_TOP + 0.05) & (r > hw.HORN.disc_d / 2 + 0.15) & \
           (r < hw.HORN.disc_d / 2 + 0.25)                          # recess wall, where the horn disc bears
    horn |= (np.abs(t - J.HORN_TOP) < 0.05) & (r < hw.HORN.disc_d / 2 + 0.25)   # recess floor
    seat = (t > J.IP[0] + J.CAP_T - 0.05) & (t < J.BRG[1] + 0.05) & (np.abs(r - hw.HOLES["bearing"].d / 2) < 0.05)
    n_seat = int(seat.sum())
    loads = {}
    for name, d in (("b", bdir), ("w", wdir), ("a", -a)):
        F = np.zeros_like(X)
        F[seat] = d / n_seat                                           # 1 N total on the seat
        loads[name] = F
    inp = work / "n3k.inp"
    perm = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]
    with open(inp, "w") as f:
        f.write("*NODE\n")
        for i, x in enumerate(X):
            f.write(f"{i + 1},{x[0]:.5f},{x[1]:.5f},{x[2]:.5f}\n")
        f.write("*ELEMENT,TYPE=C3D10,ELSET=EALL\n")
        for i, e in enumerate(E):
            f.write(f"{i + 1}," + ",".join(str(e[p] + 1) for p in perm) + "\n")
        f.write("*NSET,NSET=FIX\n" + "".join(f"{n + 1}\n" for n in np.where(horn)[0]))
        f.write("*MATERIAL,NAME=PLA\n*ELASTIC\n2300.0,0.35\n*SOLID SECTION,ELSET=EALL,MATERIAL=PLA\n")
        f.write("*BOUNDARY\nFIX,1,3\n")
        for name, F in loads.items():
            f.write("*STEP\n*STATIC\n*CLOAD,OP=NEW\n")
            for n in np.where(np.abs(F).sum(1) > 0)[0]:
                for dd in range(3):
                    if F[n, dd] != 0:
                        f.write(f"{n + 1},{dd + 1},{F[n, dd]:.6e}\n")
            f.write("*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n")
    r_ = subprocess.run(["ccx", "-i", "n3k"], cwd=work, capture_output=True, text=True)
    if r_.returncode != 0:
        raise RuntimeError(r_.stdout[-500:])
    U, S, cur, kind = [], [], None, None
    for line in open(work / "n3k.frd"):
        if line.startswith(" -4  DISP"):
            cur, kind = np.zeros((len(X), 3)), "U"
        elif line.startswith(" -4  STRESS"):
            cur, kind = np.zeros((len(X), 6)), "S"
        elif line.startswith(" -3") and cur is not None:
            (U if kind == "U" else S).append(cur)
            cur = None
        elif cur is not None and line.startswith(" -1"):
            n = int(line[3:13]) - 1
            cur[n] = [float(line[13 + 12 * i:25 + 12 * i]) for i in range(cur.shape[1])]
    ub = float((U[0][seat] @ bdir).mean())
    uw = float((U[1][seat] @ wdir).mean())
    ua = float((U[2][seat] @ -a).mean())

    def vm(s):
        sx, sy, sz, sxy, syz, szx = s.T
        return np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    smax = [float(np.percentile(vm(s), 99.9)) for s in S]       # 99.9th percentile: ignores load-point spikes
    vm_a = vm(S[2])
    hot = X[np.argmax(vm_a)] - j.p
    return dict(k_b=1.0 / ub, k_w=1.0 / uw, k_a=1.0 / ua, vm_per_N_b=smax[0], vm_per_N_w=smax[1], vm_per_N_a=smax[2],
                hot_a_local=[round(float(hot @ bdir), 1), round(float(hot @ wdir), 1), round(float(hot @ a), 1)],
                nodes=len(X), elements=len(E),
                seat_nodes=n_seat, fixed_nodes=int(horn.sum()))


# ------------------------------------------------------------------ C. axial stack and the 31.9 gap
def check_stack():
    st = stack()
    # axial chain: N3 is located along a by the horn on the spline (horn top face on the recess floor).
    # Δh = how much higher the real horn top sits than the CAD (spline height + horn seating).
    gap = J.GAP                                         # holder idler wall to idler plate, nominal
    tower = J.HP[0] - J.S.tower_h                       # tower top to horn plate inner face
    lim_up = gap                                        # horn higher → idler plate moves toward the holder wall
    mg90s_dh = round((9.7 - 8.3), 2)
    res = [finding(
        "C-1", "Critical", "FAILED",
        "N3 bị định vị dọc trục hai lần (horn trên trục then hoa và lip bạc đạn), không có phần tử bù",
        "Theo joint.py: mặt trên đĩa horn tựa đáy hốc (a = HORN_TOP); vòng trong bạc đạn bị ốc M3 kẹp vào boss "
        "của neck_link; vòng ngoài tựa lip 0.5 mm của N3 về phía holder. Nếu horn thật ngồi cao hơn CAD Δh, N3 "
        f"dịch Δh về phía holder: khe đĩa idler ↔ thành holder (thiết kế {gap} mm) còn {gap} − Δh. Với vòng "
        "ngoài lắp chặt, chuỗi bị khóa hai đầu: lực dọc trục dồn vào then hoa servo và bạc đạn. Bằng chứng Δh "
        f"thật có thể lớn: STEP MG90S của nhà sản xuất cao hơn CAD {mg90s_dh} mm ở đỉnh then hoa "
        f"[{SRC['MG90S_STEP']}], tức là lớn hơn khe {gap} mm.",
        dict(gap_idler=gap, max_horn_rise_before_contact=lim_up, mg90s_step_spline_rise=mg90s_dh,
             tower_to_horn_plate=round(tower, 3)),
        "Chỉ định bạc đạn idler là ổ TỰ DO dọc trục (vòng ngoài lắp trượt) và horn là ổ ĐỊNH VỊ. Đo disc_low "
        "và spline_top của SG90 + horn thật. Nếu Δh thật > 0.4 thì phải sửa CAD (đổi GAP/HP), cần duyệt.")]
    # worst case of the idler gap with print tolerance (unknown → shown as a function)
    res.append(finding(
        "C-2", "Major", "UNKNOWN", "Khe 31.9 mm: chưa thể kiểm tra trường hợp xấu nhất vì dung sai in và kích thước "
                                   "SG90/horn đều chưa đo",
        f"Khe trong {st['gap_inner']} mm = HP[0] − IP[1]. Bên trong có: thành idler neck_link "
        f"{round(J.IW[1] - J.IW[0], 2)} + khe {gap} + vỏ servo {J.S.case_h} + khe vỏ {hw.CLEAR_SERVO} + tháp "
        f"{J.S.tower_h} + khe tháp {round(tower, 2)}. Điều kiện không cọ: 0.8 − Δh − t_N3 − t_N2 ≥ 0.2 (khe động "
        "tối thiểu, đề xuất), với t_N3, t_N2 là sai số in của khe 31.9 và vị trí thành idler.",
        dict(chain=dict(idler_wall=round(J.IW[1] - J.IW[0], 2), gap=gap, case=J.S.case_h, clear=hw.CLEAR_SERVO,
                        tower=J.S.tower_h, tower_gap=round(tower, 2), total=st["gap_inner"]),
             condition="0.8 - dh - t_N3 - t_N2 >= 0.2"),
        "Cần: Δh (đo SG90 + horn), t_N3 và t_N2 (từ phiếu T0). Đến lúc đó trạng thái giữ UNKNOWN."))
    return res, st


# ------------------------------------------------------------------ D. bearing seat
def check_bearing():
    seat = hw.HOLES["bearing"].d
    od_max, od_min = 6.0, 6.0 - 0.008
    st = stack()
    return [finding(
        "D-1", "Critical", "FAILED", "Ổ bạc đạn: kiểu lắp chưa được định nghĩa; Ø6.05 trên CAD là lắp có khe hở",
        f"Ổ in Ø{seat}, OD bạc đạn {od_min}–{od_max} [{SRC['MR63ZZ']}] → khe hở danh nghĩa "
        f"+{seat - od_max:.3f}…+{seat - od_min:.3f} mm trên đường kính. hardware.py lại ghi 'press fit; ream "
        "lightly if tight', tức là đang trông vào việc máy in làm lỗ nhỏ hơn danh nghĩa. Lỗ FDM thường nhỏ hơn "
        "CAD một lượng phụ thuộc máy/profile (chưa đo), nên kiểu lắp thật có thể là khe hở, trung gian hay chặt.",
        dict(seat_d=seat, od=(od_min, od_max), clearance=(round(seat - od_max, 3), round(seat - od_min, 3)),
             seat_depth=st["seat_depth"], bearing_w=2.5),
        "Theo C-1: ổ idler là ổ tự do → cần lắp TRƯỢT (vòng ngoài trượt dọc trục được bằng tay, không lắc ngang). "
        "Ghi yêu cầu chức năng thay cho số Ø, chốt Ø in bằng phiếu thử ổ bạc đạn (nhiều cỡ)."),
        finding(
        "D-2", "Major", "VERIFIED", "Giữ bạc đạn dọc trục: chỉ một phía (lip), phía ngoài không có vai",
        f"Vòng ngoài: lip {J.LIP} mm (Ø4.6) chặn về phía holder; phía nắp chỉ có khoảng trống cho long đền + đầu "
        f"ốc + 0.15 + nắp {J.CAP_T}. Vòng trong: kẹp giữa boss neck_link và long đền W3 + ốc M3×6 đầu dù. Như "
        "vậy khi tháo ốc idler, bạc đạn chỉ còn được giữ bằng ma sát ổ. Đây là đúng nguyên lý ổ tự do, nhưng "
        "tiêu chí 'lật ngược không rơi' (K1) thì mâu thuẫn với lắp trượt.",
        dict(lip=J.LIP, lip_hole=4.6, cap_t=J.CAP_T),
        "Đổi K1 thành tiêu chí đo được: (a) chốt chuẩn Ø6.00 lọt bằng tay, chốt Ø6.10 không lọt; (b) sau lắp, "
        "quay khớp hết hành trình không kẹt, độ rơ hướng kính tại mép đĩa ≤ TBD (đo).")]


# ------------------------------------------------------------------ E. walls, screws, loads
def check_walls():
    st = stack()
    h = hw.HORN
    screw_len = 4.0                                    # ST2x4 from the BOM
    protrusion = screw_len - h.disc_t                  # thread past the horn into the plate
    tip = J.HORN_TOP + protrusion
    into_skin = round(tip - st["pilot_bottom"], 3)
    # torque path: stall torque through the two horn screws at r = screw_r
    t_stall = (0.118, 0.177)                           # N·m, 1.2 and 1.8 kgf·cm (datasheets disagree)
    f_screw = [round(t * 1000 / (2 * h.screw_r), 1) for t in t_stall]
    bearing_stress = [round(f / (2.0 * st["pilot_depth"]), 2) for f in f_screw]
    m, com = 0.07125, (13.06, 26.11)                   # head_pitch_link from bazan_cad.urdf (CAD)
    t_grav = round(m * G * math.hypot(*com), 1)        # N·mm, worst angle
    walls = dict(horn_ring=round(J.HORN_R - (h.disc_d + 0.4) / 2, 2),
                 idler_cap_ring=round(J.IDLER_R - (J.IDLER_R - 1.0), 2),
                 idler_seat_wall=round(J.IDLER_R - hw.HOLES["bearing"].d / 2, 3),
                 idler_seat_wall_under_cap=round((J.IDLER_R - 1.0) - hw.HOLES["bearing"].d / 2, 3),
                 horn_floor=round(J.HP[1] - J.HORN_TOP, 2),
                 horn_floor_under_cap=round(J.HP[1] - J.CAP_T - J.HORN_TOP, 2))
    return [finding(
        "E-1", "Critical", "FAILED", "Vít horn ST2×4 đâm vào lớp da 0.40 mm phía ngoài lỗ mồi",
        f"Lỗ mồi Ø{hw.HOLES['pilot_st2'].d} mù, sâu {st['pilot_depth']} mm, đáy ở a = {st['pilot_bottom']}, chừa "
        f"da 0.40 mm tới mặt ngoài đĩa. Vít ST2×4 qua đĩa horn dày {h.disc_t} (MEASURE) nhô {protrusion} mm → "
        f"mũi vít ở a = {tip:.2f}, vượt đáy lỗ mồi {into_skin:+.2f} mm. Horn dày hơn/mỏng hơn ±0.3 làm mũi vít "
        f"vượt từ {into_skin - 0.3:+.2f} đến {into_skin + 0.3:+.2f} mm, tức là có thể thủng da. Thêm nữa, da 0.40 "
        "nhỏ hơn bề rộng một đường đùn của đầu 0.4 (≈ 0.45): slicer có thể bỏ hoặc in không kín.",
        dict(pilot_depth=st["pilot_depth"], skin=0.4, screw=screw_len, horn_t=h.disc_t, tip_past_pilot=into_skin),
        "Da 0.4 chỉ có tác dụng thẩm mỹ (che lỗ), không chịu lực. Hai hướng sửa, cần duyệt: (1) lỗ mồi xuyên, bỏ da; "
        "(2) giữ da nhưng dùng vít ST2×3 hoặc tăng lỗ mồi để mũi vít cách da ≥ 0.5. Chọn theo đo horn thật."),
        finding(
        "E-2", "Minor", "ASSUMED", "Tải qua vít horn khi servo kẹt (stall): ứng suất dập trong PLA thấp",
        f"Mô-men kẹt SG90 {t_stall} N·m [{SRC['SG90_TQ']}] truyền qua 2 vít ở r = {h.screw_r}: lực cắt mỗi vít "
        f"{f_screw} N, ứng suất dập lên nhựa (Ø2 × {st['pilot_depth']}) ≈ {bearing_stress} MPa. Mô-men trọng lực "
        f"đầu lớn nhất ≈ {t_grav} N·mm (m = {m * 1000:.1f} g, CoM {com} mm từ trục, từ URDF sinh từ CAD). Đĩa horn "
        "tròn nằm trong hốc tròn nên không có khóa hình dạng: toàn bộ mô-men đi qua 2 vít.",
        dict(stall_torque=t_stall, screw_shear=f_screw, bearing_stress=bearing_stress, gravity_torque=t_grav),
        "Chưa tính va đập khi ngã (chưa có tải va đập), và chưa có số bền lớp in. Kiểm tra thực nghiệm: thử "
        "kẹt servo 10 lần + thử ngã (kế hoạch kiểm tra)."),
        finding(
        "E-3", "Major", "ASSUMED", "Các thành mỏng khác: 1.80 (vành hốc horn) và 1.00 (vành nắp idler)",
        f"Vành hốc horn {walls['horn_ring']} mm: chỉ định tâm đĩa horn, tải hướng kính tĩnh ≈ 0.35 N; ≈ 4 đường "
        f"đùn. Vành nắp idler {walls['idler_cap_ring']} mm: chỉ giữ nắp, không chịu lực. Thành chịu lực quanh ổ "
        f"bạc đạn = {walls['idler_seat_wall_under_cap']} mm (dưới hốc nắp). Đáy hốc horn {walls['horn_floor']} mm, "
        f"dưới hốc nắp còn {walls['horn_floor_under_cap']} mm.",
        walls, "Không cần sửa cho tải tĩnh. Cần thử ngã (va đập lên vỏ đầu) trước khi chốt."),
        finding(
        "E-4", "Major", "UNKNOWN", "Mỏi, rão (creep) và ảnh hưởng hướng lớp in: chưa có dữ liệu",
        "PLA rão dưới tải lâu dài và mềm đi từ ≈ 50–60 °C; servo đầu đứng yên giữ đầu liên tục. Hướng in hiện tại "
        "làm hai thanh đỡ (bars x ±6, z 0–20) chịu uốn với mặt lớp song song mặt sàn, là mặt yếu khi đầu bị đẩy "
        "theo X.", {}, "Đo nhiệt độ vỏ SG90 khi giữ đầu 30 phút (tiêu chí M1 PLA/PETG). Thử ngã theo X."),
    ]


# ------------------------------------------------------------------ G. assembly / removal of N3
def check_removal(k):
    travel = round(max(J.HORN_TOP - J.HP[0], J.S.spline_top - (J.S.tower_h + 0.4)), 2)
    spread = round(travel - J.GAP, 2)
    num = dict(travel_needed=travel, axial_room=J.GAP, spread_needed=spread)
    txt = (f"Muốn tháo/lắp N3 thì horn (bắt vít vào N3) phải trượt khỏi then hoa: cần dịch {travel} mm dọc trục "
           f"(max của đáy hốc − mặt trong = {round(J.HORN_TOP - J.HP[0], 2)} và chiều dài ăn khớp then hoa "
           f"{round(J.S.spline_top - J.S.tower_h - 0.4, 2)}). Phía idler chỉ có {J.GAP} mm trước khi đĩa idler chạm "
           f"thành neck_link (đã kiểm bằng dịch khối CAD: chồng 36 mm³ ở 1.0 mm, 545 mm³ ở 4.0 mm). Vậy phải banh "
           f"hai đĩa của chữ U ra ≥ {spread} mm. Ổ servo chỉ mở về phía trục (+a), bị đĩa horn che, nên cũng không "
           "thể lắp servo khi N3 đang ở chỗ. Trình tự 'tháo ốc idler rồi nhấc N3 khỏi horn' trong ghi chú L4 của "
           "bản vẽ P1 vì vậy không làm được nếu không banh khung.")
    if k:
        f = k["k_a"] * spread
        sig = k["vm_per_N_a"] * f
        num |= dict(k_spread=round(k["k_a"], 2), force=round(f, 1), stress=round(sig, 1),
                    hot_spot_local=k["hot_a_local"])
        txt += (f" FEA (cùng mô hình, lực dọc trục đặt lên ổ idler): độ cứng banh k_a = {k['k_a']:.2f} N/mm → cần "
                f"≈ {f:.0f} N, ứng suất ≈ {sig:.0f} MPa (phân vị 99.9 %, tuyến tính) tại điểm nóng (b, w, a) = "
                f"{k['hot_a_local']} mm trong khung khớp. Giới hạn bền kéo PLA in thường được nêu khoảng "
                "40–60 MPa theo phương sợi, thấp hơn nhiều giữa các lớp (chưa đo với máy của bạn).")
    return [finding("G-1", "Critical", "FAILED", "N3 không tháo/lắp được theo trình tự đã ghi nếu không banh khung",
                    txt, num, "Không sửa trong bản vẽ. Hướng sửa CAD (cần duyệt, ảnh hưởng N2 và cả các khớp chân "
                    "cùng kiểu joint.py): (1) idler tháo rời được (đĩa idler bắt vít vào thanh), hoặc (2) khe GAP "
                    "≥ hành trình rút horn, hoặc (3) horn bắt vít từ phía ngoài để N3 rút theo hướng kính sau khi "
                    "tháo horn. Trước khi quyết: thử trên bản in T2 (foot) xem banh 3 mm có nứt không.")]


# ------------------------------------------------------------------ collision sweep over the pitch range
def sweep(step_deg=4.0):
    """Minimum distance over the head-pitch range between everything on the head link and everything else.

    Features on the joint axis (lip hole round the holder boss, spline inside the access hole, horn on the spline)
    are coaxial by design, so the sweep is run twice: whole solids, and with a Ø7 core round the axis removed."""
    import robot as R
    import body as B
    import leg as LEG
    import shell as SH
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    body, leg = B.Body(), LEG.Leg()
    sh = SH.BazanShell(body, leg)
    j = body.joints["head_pitch"]
    core = j.place(J.cyl(3.5, -60, 60))
    lo, hi = R.RANGES["head_pitch"]
    angles = [float(v) for v in np.arange(lo, hi + 1e-9, step_deg)] + [float(hi)]
    ref = R.assemble(body, leg, {}, include_legs=False)
    head = sorted(n for n in ref if R.link_of(n, body, leg) == "head_frame")
    near = ["neck_link", "servo_head_pitch", "horn_neck_yaw", "servo_neck_yaw", "neck_mount", "deck_2"]

    def dist(a, b):
        d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
        d.Perform()
        return round(d.Value(), 3)

    rows = {}
    for ang in angles:
        q = {"head_pitch": ang}
        c = R.assemble(body, leg, q, include_legs=False)
        shells = sh.placed(q)
        moving = {n: c[n] for n in head} | {n: shells[n] for n in sh.HEAD}
        fixed = {n: c[n] for n in near} | {n: s for n, s in shells.items() if n not in sh.HEAD}
        for a, sa in moving.items():
            for b, sb in fixed.items():
                if a == "horn_head_pitch" and b == "servo_head_pitch":
                    continue                       # horn pressed on the spline: one body
                v = R.overlap(sa, sb)
                d_all = dist(sa, sb)
                d_core = dist(sa.cut(core), sb.cut(core)) if a in ("head_frame", "horn_head_pitch") else d_all
                r = rows.setdefault((a, b), dict(moving=a, fixed=b, min_all=9e9, min_off_axis=9e9, overlap=0.0,
                                                 at_all=None, at_off_axis=None))
                if d_all < r["min_all"]:
                    r["min_all"], r["at_all"] = d_all, ang
                if d_core < r["min_off_axis"]:
                    r["min_off_axis"], r["at_off_axis"] = d_core, ang
                r["overlap"] = max(r["overlap"], round(v, 3))
    out = sorted(rows.values(), key=lambda r: (-r["overlap"], r["min_off_axis"]))
    return dict(range=(lo, hi), step=step_deg, angles=angles, pairs=out, not_modelled=[
        "dây servo và cáp (CSI, màn hình)", "đầu ốc tai M2 và đầu vít horn ST2",
        "vỏ SG90 thật (dùng hộp MG92B, lớn hơn)"])


def check_sweep(sw):
    p = {(r["moving"], r["fixed"]): r for r in sw["pairs"]}
    nl = p[("head_frame", "neck_link")]
    over = [r for r in sw["pairs"] if r["overlap"] > 0.05]
    min_shell = min((r for r in sw["pairs"] if r["moving"] in ("head_front", "head_back", "visor")),
                    key=lambda r: r["min_all"])
    off = min((r for r in sw["pairs"] if r["moving"] == "head_frame"), key=lambda r: r["min_off_axis"])
    return [finding(
        "F-1", "Major", "FAILED", "Hai khe giữa N3 và neck_link nhỏ hơn quy tắc khe động 1.0 mm của chính dự án",
        f"Quét {sw['range']}° bước {sw['step']}°, không có va chạm ({len(over)} cặp chồng khối). Khe nhỏ nhất "
        f"N3↔neck_link: {nl['min_all']} mm hướng kính (lỗ lip Ø4.6 quanh boss Ø4.2, đồng trục, không đổi theo góc) "
        f"và {nl['min_off_axis']} mm dọc trục (đĩa idler ↔ thành idler, = GAP). hardware.GAP_MOVING = "
        f"{hw.GAP_MOVING} mm. Khe 0.2 hướng kính chỉ đúng khi lỗ in ra đúng Ø4.6; lỗ FDM thường nhỏ hơn CAD.",
        dict(radial_lip_boss=nl["min_all"], axial_idler=nl["min_off_axis"], rule=hw.GAP_MOVING),
        "Khe hướng kính: tăng lỗ lip lên Ø5.0 (thay đổi trong N3, không đổi hình ngoài, cần duyệt) hoặc chấp nhận "
        "kèm điều kiện kiểm tra. Khe dọc trục: xem C-1/C-2, đây chính là khe bị Δh ăn mất."),
        finding(
        "F-2", "Info", "VERIFIED", "Không va chạm khung/vỏ trong toàn hành trình gật đầu (chỉ hình học CAD)",
        f"Khe nhỏ nhất ngoài trục: {off['moving']}↔{off['fixed']} = {off['min_off_axis']} mm. Vỏ đầu: "
        f"{min_shell['moving']}↔{min_shell['fixed']} = {min_shell['min_all']} mm tại {min_shell['at_all']}°. Chưa "
        f"mô hình: {', '.join(sw['not_modelled'])}.",
        dict(min_off_axis=off["min_off_axis"], shell=min_shell["min_all"]),
        "Phần chưa mô hình (dây, đầu ốc) được kiểm tra khi lắp thử T0.")]


# ------------------------------------------------------------------ Phase 2: print orientations, sliced for real
PRUSA_INI = """bed_shape = 0x0,220x0,220x220,0x220
max_print_height = 250
nozzle_diameter = 0.4
layer_height = 0.2
first_layer_height = 0.2
perimeters = 4
top_solid_layers = 5
bottom_solid_layers = 4
fill_density = 40%
fill_pattern = gyroid
filament_type = PLA
filament_diameter = 1.75
filament_density = 1.24
temperature = 210
first_layer_temperature = 215
bed_temperature = 60
support_material = {sup}
support_material_auto = 1
support_material_threshold = 45
support_material_contact_distance = 0.2
support_material_buildplate_only = {bp}
gcode_comments = 1
"""
ORIENT = {   # name: rotation matrix applied to the N3 solid in its datum frame O (rows = new x, y, z)
    "O1 sàn úp xuống (hiện tại)": ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
    "O2 đĩa horn xuống bàn": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
    "O3 đĩa idler xuống bàn": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
    "O4 tấm mặt xuống bàn": ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
}


def _gcode_scan(path):
    """Filament (g), time (s) and every extruding segment [(x0, y0, x1, y1, z)]."""
    import re
    segs, x, y, z, g, t = [], 0.0, 0.0, 0.0, None, None
    typ = ""
    for line in open(path):
        if line.startswith(";TYPE:"):
            typ = line[6:].strip()
        if line.startswith("; filament used [g]"):
            g = float(line.split("=")[1])
        elif line.startswith("; estimated printing time (normal mode)"):
            v = line.split("=")[1]
            t = sum(int(n) * m for n, m in zip(re.findall(r"(\d+)[dhms]", v)[::-1], (1, 60, 3600, 86400)))
        if not line.startswith(("G1 ", "G0 ")):
            continue
        w = dict((k[0], float(k[1:])) for k in line.split(";")[0].split()[1:] if k[0] in "XYZE")
        nx, ny = w.get("X", x), w.get("Y", y)
        z = w.get("Z", z)
        if w.get("E", 0) > 0 and ("X" in w or "Y" in w):
            segs.append((x, y, nx, ny, z, typ))
        x, y = nx, ny
    return g, t, segs


def _len_in_box(segs, box):
    (x0, x1), (y0, y1), (z0, z1) = box
    L = 0.0
    for ax, ay, bx_, by, z, _ in segs:
        if not (z0 <= z <= z1):
            continue
        n = max(2, int(math.hypot(bx_ - ax, by - ay) / 0.05))
        for i in range(n):
            u = (i + 0.5) / n
            px, py = ax + u * (bx_ - ax), ay + u * (by - ay)
            if x0 <= px <= x1 and y0 <= py <= y1:
                L += math.hypot(bx_ - ax, by - ay) / n
    return L


def _support_in_features(segs, Rm, off, ya):
    """Length of support extrusion that lands inside the functional bores (horn recess, access hole, bearing seat,
    lip hole), found by mapping every support segment back into the N3 datum frame."""
    feats = {"hốc horn Ø20.4": (10.2, J.HP[0] + ya, J.HORN_TOP + ya),
             "lỗ thao tác Ø5.2": (2.6, J.HORN_TOP + ya, J.HP[1] + ya),
             "ổ bạc đạn Ø6.05": (3.025, J.IP[0] + J.CAP_T + ya, J.BRG[1] + ya),
             "lỗ lip Ø4.6": (2.3, J.BRG[1] + ya, J.IP[1] + ya)}
    L = {k: 0.0 for k in feats}
    for ax, ay, bx_, by, z, typ in segs:
        if "upport" not in typ:
            continue
        n = max(2, int(math.hypot(bx_ - ax, by - ay) / 0.1))
        for i in range(n):
            u = (i + 0.5) / n
            pb = np.array([ax + u * (bx_ - ax), ay + u * (by - ay), z - 0.1])
            p = Rm.T @ (pb - off)
            r = math.hypot(p[0], p[2])
            for k, (rr, y0, y1) in feats.items():
                if r < rr - 0.05 and min(y0, y1) < p[1] < max(y0, y1):
                    L[k] += math.hypot(bx_ - ax, by - ay) / n
    return {k: round(v, 1) for k, v in L.items()}


def slice_compare(work):
    import shutil
    import cadquery as cq
    import body as B
    if not shutil.which("prusa-slicer"):
        return None
    body = B.Body()
    j = body.joints["head_pitch"]
    org = np.array(j.origin, float)
    n3 = body.parts["head_frame"].moved(cq.Location(cq.Vector(*(-org))))
    ya = -J.CENTER
    skin = ((-0.8, 0.8), (J.HP[1] - 0.4 + ya, J.HP[1] + ya), (7 - 0.8, 7 + 0.8))       # one pilot's skin, O frame
    (work / "pla.ini").write_text(PRUSA_INI.format(sup=0, bp=0))
    (work / "pla_sup.ini").write_text(PRUSA_INI.format(sup=1, bp=0))
    (work / "pla_bp.ini").write_text(PRUSA_INI.format(sup=1, bp=1))
    out = {}
    for name, Rm in ORIENT.items():
        Rm = np.array(Rm, float)
        from OCP.gp import gp_Trsf
        tr = gp_Trsf()
        tr.SetValues(*Rm[0], 0.0, *Rm[1], 0.0, *Rm[2], 0.0)
        s = n3.moved(cq.Location(tr))
        bb = s.BoundingBox()
        stl = work / "n3.stl"
        cq.exporters.export(s, str(stl), tolerance=0.01, angularTolerance=0.1)
        res = {}
        for tag in ("pla", "pla_sup", "pla_bp"):
            g = work / f"{tag}.gcode"
            subprocess.run(["prusa-slicer", "--export-gcode", "--load", str(work / f"{tag}.ini"), "--center", "110,110",
                            "-o", str(g), str(stl)], capture_output=True, text=True, check=True)
            res[tag] = _gcode_scan(g)
        # skin box in bed coordinates: rotate the corners, then the same offset PrusaSlicer used (centre, drop to 0)
        corners = np.array([[x, y, z] for x in skin[0] for y in skin[1] for z in skin[2]]) @ Rm.T
        off = np.array([110 - (bb.xmin + bb.xmax) / 2, 110 - (bb.ymin + bb.ymax) / 2, -bb.zmin])
        c = corners + off
        box = tuple((float(c[:, i].min()), float(c[:, i].max())) for i in range(3))
        skin_len = _len_in_box(res["pla"][2], box)
        sup_in = _support_in_features(res["pla_sup"][2], Rm, off, ya)
        sup_bp = _support_in_features(res["pla_bp"][2], Rm, off, ya)
        out[name] = dict(height=round(bb.zmax - bb.zmin, 1), grams=round(res["pla"][0], 2),
                         grams_with_support=round(res["pla_sup"][0], 2),
                         support_g=round(res["pla_sup"][0] - res["pla"][0], 2),
                         minutes=round(res["pla"][1] / 60), minutes_with_support=round(res["pla_sup"][1] / 60),
                         pitch_axis="dọc (vuông góc bàn)" if abs(Rm[2] @ np.array([0, 1, 0])) > 0.5 else "ngang",
                         support_g_buildplate_only=round(res["pla_bp"][0] - res["pla"][0], 2),
                         skin_extrusion_mm=round(skin_len, 1), support_in_features_mm=sup_in,
                         support_in_features_mm_buildplate_only=sup_bp)
    return dict(profile="PrusaSlicer, 0.4 nozzle, 0.2 layer, 4 perimeters, 5 top / 4 bottom, gyroid 40 %, PLA "
                        "(cùng profile với test_prints; KHÔNG phải profile máy in thật của bạn)", orientations=out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-fea", action="store_true")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--no-slice", action="store_true")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    k = None
    if not a.no_fea:
        with tempfile.TemporaryDirectory() as td:
            k = frame_stiffness(Path(td))
    stack_f, st = check_stack()
    res = dict(sources=SRC, stack=st, findings=check_inserts() + check_alignment(k) + stack_f + check_bearing()
               + check_walls() + check_removal(k), stiffness=k)
    if not a.no_sweep:
        res["sweep"] = sweep()
        res["findings"] += check_sweep(res["sweep"])
    if not a.no_slice:
        with tempfile.TemporaryDirectory() as td:
            res["slice"] = slice_compare(Path(td))
        if res["slice"]:
            for n, v in res["slice"]["orientations"].items():
                print(n, v)
    if a.no_fea or a.no_sweep or a.no_slice:
        print("(partial run: n3_review.json not written; run without --no-* to refresh it)")
    else:
        (OUT / "n3_review.json").write_text(json.dumps(res, indent=1, ensure_ascii=False, default=float))
    for f in res["findings"]:
        print(f"{f['id']:4s} {f['severity']:8s} {f['status']:8s} {f['title']}")
        for kk, vv in f["numbers"].items():
            print(f"       {kk}: {vv}")
    if k:
        print("stiffness", k)
    if "sweep" in res:
        print("sweep", res["sweep"]["range"])
        for r in res["sweep"]["pairs"][:12]:
            print("  ", r)


if __name__ == "__main__":
    main()
