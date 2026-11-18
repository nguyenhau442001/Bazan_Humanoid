# N3 Head Frame — Engineering Audit (Phase 1)

Bản vẽ: BZN-H-N3 rev P1 (SƠ BỘ). Chi tiết CAD: `head_frame` trong `body.py`, khớp `head_pitch` trong `joint.py`.
Ngày audit: 2026-11-18. **Audit này không sửa CAD.** Mọi con số dưới đây do `hardware/mechanical/n3_review.py`
tính ra từ source CAD và ghi vào `n3_review.json`; chạy lại script sau mỗi lần đo hay sửa thiết kế.

```powershell
cd hardware\mechanical
python n3_review.py              # đầy đủ: FEA CalculiX, quét va chạm, cắt lát PrusaSlicer (~6 phút)
python n3_review.py --no-fea --no-sweep --no-slice   # chỉ phần tính tay (vài giây)
```

## Trạng thái

| Ký hiệu | Nghĩa |
|---|---|
| VERIFIED | Suy ra trực tiếp từ source CAD hoặc dung sai của nhà sản xuất, không giả định |
| ASSUMED | Tính từ giá trị có nhãn MEASURE trong `hardware.py` hoặc từ tính chất vật liệu giả định |
| UNKNOWN | Cần số đo vật lý hoặc một quyết định mới tính được |
| FAILED | Kiểm tra trượt ngay trên số liệu CAD/BOM (hoặc trên mọi giá trị đầu vào hợp lý) |

## Kết luận

**N3 rev P1 không được phát hành chế tạo.** Kết quả: 7 lỗi FAILED (6 Critical), 4 mục UNKNOWN, 3 mục ASSUMED,
2 mục VERIFIED.

Ba vấn đề lớn nhất **không phải do bản vẽ trình bày kém**. Chúng nằm ở thiết kế của khớp:

1. **G-1. Không tháo hay lắp được N3** nếu không banh khung chữ U ra khoảng 3.1 mm. Theo FEA, banh như vậy cần
   khoảng 41 N và gây ứng suất khoảng 32 MPa tại gốc thanh đỡ phía horn.
2. **B-1. Trục SG90 lệch trục idler 0.3–0.4 mm ngay trên CAD**, vì ổ servo và tán tai trên neck_link được dựng
   cho MG92B.
3. **C-1. N3 bị định vị dọc trục hai lần** (horn và lip bạc đạn) mà không có phần tử bù. Khe 0.8 mm nhỏ hơn
   chênh lệch chiều cao then hoa đo được trên STEP MG90S của nhà sản xuất (1.4 mm).

G-1 và C-1 nằm trong `joint.py`, tức là kiểu khớp dùng chung cho **mọi khớp servo của robot**, kể cả chân.
Mình chưa kiểm tra từng khớp, nhưng có thể thử ngay trên bản in T2 (`foot`).

## Bảng phát hiện

| ID | Mức | Trạng thái | Vấn đề | Chi tiết chịu sửa |
|---|---|---|---|---|
| G-1 | Critical | FAILED | Tháo/lắp N3 cần banh U ≥ 3.1 mm (≈ 41 N, ≈ 32 MPa) | joint.py (N3 + N2) |
| B-1 | Critical | FAILED | Trục SG90 lệch trục idler +0.30 (tới 0.40) mm | neck_link (N2) |
| C-1 | Critical | FAILED | Định vị dọc trục kép, khe bù 0.8 mm | joint.py, N3 |
| A-1 | Critical | FAILED | Lỗ tán Ø3.2 = OD tán Ø3.2 ghi trong BOM | BOM + N3 |
| D-1 | Critical | FAILED | Kiểu lắp ổ bạc đạn chưa định nghĩa; Ø6.05 là lắp có khe hở trên CAD | N3 |
| E-1 | Critical | FAILED | Mũi vít ST2×4 vượt đáy lỗ mồi 0.1 mm, đâm vào da 0.40 | N3 + BOM |
| B-3 | Critical | UNKNOWN | Đồng trục hốc horn ↔ ổ bạc đạn (K3) chưa có số khả thi | N3 + quy trình |
| F-1 | Major | FAILED | Khe lip↔boss 0.2 (hướng kính) và 0.8 (dọc trục) < quy tắc 1.0 | N3 |
| B-2 | Major | ASSUMED | Lệch trục ép khung thường trực 3.5–4.7 N | — |
| C-2 | Major | UNKNOWN | Chưa có trường hợp xấu nhất cho khe 31.9 | — |
| A-2 | Major | UNKNOWN | Chưa có số kéo tuột/xoay của tán trong sàn 3 mm | — |
| D-2 | Major | VERIFIED | Vòng ngoài bạc đạn chỉ được chặn một phía | — |
| E-3 | Major | ASSUMED | Thành 1.80 / 1.00: đủ cho tải tĩnh, chưa xét va đập | — |
| E-4 | Major | UNKNOWN | Rão, mỏi, hướng lớp: chưa có dữ liệu | — |
| E-2 | Minor | ASSUMED | Tải vít horn khi kẹt servo: ứng suất dập 1.7–2.5 MPa | — |
| F-2 | Info | VERIFIED | Không va chạm trong −30…+32° (chỉ phần có trong CAD) | — |

---

## A. Tán cấy nhiệt M2 (A1–A4, B1–B2)

**A-1 · Critical · FAILED. Lỗ bằng đúng OD tán.**
`hardware.py` ghi tán `INS-M2` là "3 long, 3.2 OD" và lỗ `ins_m2_thru` là Ø3.2. Như vậy độ dôi hướng kính danh
nghĩa bằng **0.00 mm**.

Đối chiếu với tán thương mại:
- Tán M2 bán cho lỗ Ø3.2 có OD gai khoảng 3.6–3.7.
- Ví dụ TFC: L 4.0, W 3.7, lỗ khuyến nghị 3.2, thành tối thiểu 1.6 mm.
- Ví dụ Accu HTFBI-M2: L 3.9, lỗ khuyến nghị 3.2 +0.1/−0.

Vậy BOM và lỗ đang mô tả hai loại tán khác nhau. Lỗi nằm ở **đặc tả**: khi chưa có mã tán thì Ø lỗ không có căn
cứ.

Hướng sửa: chọn một mã tán có datasheet, ghi mã đó vào BOM, lấy Ø lỗ theo datasheet, rồi xác nhận bằng phiếu thử
V1. Không đổi Ø lỗ trước khi có mã tán.

**A-2 · Major · UNKNOWN. Độ giữ của tán L3 trong sàn 3 mm, lỗ xuyên.**
- Khoảng cách tâm lỗ đến mép sàn: 4.0 mm với 4 lỗ vỏ, 14.0 mm với 2 lỗ cam. Với OD 3.7, thành còn 2.15 mm, vẫn
  ≥ 1.6 mm tối thiểu của TFC.
- Ốc M2×6 của vỏ đầu kéo tán theo đúng hướng nó được ép vào. Đây là hướng yếu nhất của tán gai.
- Chưa có số đo lực kéo tuột và mô-men xoay tuột, nên không kết luận được độ bền.

## B. Đồng trục trục gật đầu

**B-1 · Critical · FAILED. Ổ servo dựng cho MG92B nhưng khớp lắp SG90.**
`joint.py` dùng `S = hw.MG92B` cho mọi khớp, còn `JOINT_SERVO["head_pitch"] = "SG90"`. Tính với số datasheet
SG90 trong `hardware.py`:

| | SG90 trong ổ MG92B | MG92B (đúng loại) |
|---|---|---|
| Trục ra khi tâm lỗ tai trùng tán (theo b) | **+0.30 mm** | 0.00 |
| Sai bước lỗ tai, mỗi bên | 0.10 mm | 0.00 |
| Lệch xấu nhất khi đã siết tai | **0.40 mm** | 0.00 |
| Vùng trục có thể nằm khi tai lỏng, theo b | −0.20…+0.80 | −0.20…+0.20 |
| Vùng trục có thể nằm khi tai lỏng, theo w | ±0.40 | ±0.20 |

Trục idler (tán M3 trên neck_link) nằm đúng tâm khớp, nên lệch 0.3–0.4 mm là lệch giữa **hai điểm tựa của
N3**.

Số datasheet SG90 không đáng tin. Các bản datasheet công bố kích thước thân khác nhau: 22.2 × 11.8,
23.1 × 12.2 và 22.0 × 11.5 mm. Vì vậy lệch thật có thể lớn hơn 0.4 mm, và **phải đo servo thật**.

**B-2 · Major · ASSUMED. Hệ quả của lệch trục.**
FEA CalculiX: lưới tet10, 68 582 nút, PLA E = 2300 MPa (giả định). Giữ cố định hốc horn và đặt 1 N lên ổ bạc đạn,
cho kết quả:
- Độ cứng tương đối giữa hai ổ: **k_b = 11.7 N/mm**, **k_w = 10.1 N/mm**.
- Phần mềm nhất là sàn 3 mm, chỗ hai thanh đỡ cắm vào.

Hệ quả:
- Lệch 0.30–0.40 mm ép khung với **3.5–4.7 N thường trực** lên trục ra SG90 và bạc đạn. Để so sánh, trọng lượng
  đầu chỉ khoảng 0.70 N.
- Ứng suất khung khoảng 3.4 MPa: thấp, nhưng là tải liên tục nên PLA bị rão.
- Ma sát thêm ở bạc nhựa trục ra khoảng 2.3–4.7 N·mm (giả định μ 0.2–0.4, r 2.5 mm). Con số này chỉ bằng
  1–4 % mô-men kẹt.

Lưu ý: **ước lượng tay ban đầu của mình sai.** Mình từng nghĩ khung quá cứng, nhưng FEA cho thấy khung đủ mềm.
Vì vậy vấn đề không phải kẹt khớp mà là tải ngang thường trực lên bánh răng ra bằng nhựa của SG90, gây mòn và tăng
độ rơ.

Mục tiêu đề xuất: lệch lắp ráp ≤ 0.1 mm (khoảng 1.2 N).

**B-3 · Critical · UNKNOWN. Đồng trục trên chính N3 (K3).**
Hai ổ nằm trên hai đĩa cách nhau 31.9 mm. Có hai điều mà P1 chưa thấy:

1. **Hốc horn không định tâm horn.** Hốc Ø20.4 trong khi đĩa horn Ø20, nên horn có khe hở hướng kính 0.2 mm.
   Vị trí hướng kính của horn trên N3 thật ra do **2 vít tự ren trong lỗ mồi in Ø1.6** và **2 lỗ người dùng tự
   khoan trên horn** quyết định (`screw_r = 7`, "drill if missing"). Sai số khoan tay có thể cỡ vài phần mười
   mm. Vì vậy K3 hiện gồm ba nguồn sai số: in N3, khoan horn, và độ đảo then hoa của horn.
2. Theo hướng in hiện tại, trục gật nằm ngang. Hai lỗ được tạo từ cùng các lớp in, nên sai lệch theo Z chủ yếu do
   bước lớp và theo X do sai số XY. Đổi lại, độ tròn của lỗ kém vì đỉnh lỗ là cầu hoặc mặt treo.

Chưa có máy in, profile hay phiếu đo nào được ghi lại, nên **không ghi số dung sai**. Cách làm: chốt bằng phiếu
thử V3, và giá trị phải ≤ độ rơ trục ra đo được của SG90.

## C. Khe 31.9 mm và chuỗi kích thước dọc trục

**C-1 · Critical · FAILED. Định vị dọc trục kép.**
Theo `joint.py`, N3 bị khóa dọc trục ở cả hai đầu:
- Mặt trên đĩa horn tựa đáy hốc, tại `HORN_TOP` = 8.4 mm trên mặt vỏ servo. Horn được ép lên then hoa, nên N3 bị
  khóa dọc trục bởi chiều cao then hoa.
- Vòng trong bạc đạn bị ốc M3 kẹp vào boss của neck_link. Vòng ngoài tựa lip 0.5 mm của N3.

Hệ quả:
- Nếu horn thật ngồi cao hơn CAD một đoạn Δh, đĩa idler tiến về thành neck_link. Khe 0.8 mm còn lại 0.8 − Δh.
- STEP MG90S của nhà sản xuất có đỉnh then hoa cao hơn CAD **1.4 mm** (`test_print.md`), lớn hơn khe 0.8 mm.
- Nếu vòng ngoài lắp chặt, chuỗi bị khóa hai đầu và lực dọc trục dồn vào then hoa servo.

Hướng sửa về nguyên lý (cần duyệt): **horn là gối định vị, bạc đạn idler là gối tự do dọc trục**, nghĩa là vòng
ngoài lắp trượt. Đây là cách bố trí gối định vị/gối tự do kinh điển.

**C-2 · Major · UNKNOWN. Trường hợp xấu nhất của khe 31.9.**
Chuỗi kích thước bên trong khe 31.9:

| Thành idler | Khe | Vỏ servo | Khe vỏ | Tháp | Khe tháp | Tổng |
|---|---|---|---|---|---|---|
| 3.6 | 0.8 | 22.7 | 0.2 | 4.0 | 0.6 | 31.9 |

Điều kiện để không cọ: **0.8 − Δh − t_N3 − t_N2 ≥ 0.2**. Trong đó Δh lấy từ số đo SG90 và horn; t_N3, t_N2 là
sai số in, lấy từ phiếu T0. Khi chưa có ba số này, trạng thái vẫn là UNKNOWN.

## D. Ổ bạc đạn MR63ZZ

**D-1 · Critical · FAILED. Kiểu lắp chưa được định nghĩa.**
- Ổ in Ø6.05; OD bạc đạn 5.992–6.000 (dung sai 0/−0.008). Trên CAD đây là **khe hở +0.050…+0.058 mm**.
- `hardware.py` lại ghi "press fit; ream lightly if tight", tức là đang trông vào việc máy in làm lỗ nhỏ hơn danh
  nghĩa.
- Theo C-1, ổ idler phải là gối tự do, cần **lắp trượt**. Vậy yêu cầu chức năng phải được ghi ra, rồi chốt Ø in
  bằng phiếu thử V2.

**D-2 · Major · VERIFIED. Giữ dọc trục.**
- Vòng ngoài: chỉ có lip 0.5 mm (lỗ Ø4.6) chặn về phía holder; phía nắp không có vai.
- Vòng trong: kẹp giữa boss của neck_link và long đền W3 + ốc M3×6 đầu dù.

Bố trí này đúng với nguyên lý gối tự do. Nhưng tiêu chí K1 "lật ngược không rơi" mâu thuẫn với lắp trượt, nên cần
thay bằng tiêu chí đo được (xem `N3_VALIDATION_PLAN.md`).

## E. Thành mỏng, vít, tải

**E-1 · Critical · FAILED. Vít horn và lớp da 0.40.**
- Lỗ mồi Ø1.6 mù, sâu 2.5 mm, chừa da 0.40 mm tới mặt ngoài đĩa.
- Vít ST2×4 xuyên qua đĩa horn dày 1.5 (giá trị MEASURE) nhô ra 2.5 mm, nên **mũi vít vượt đáy lỗ mồi 0.1 mm**.
- Nếu horn dày ±0.3 mm thì mũi vít vượt từ −0.2 đến +0.4 mm, có thể thủng da.
- Da 0.40 mm mỏng hơn bề rộng một đường đùn (khoảng 0.45 mm). PrusaSlicer vẫn sinh 13–17 mm đường đùn trong vùng
  da mỗi lỗ, nhưng độ kín thì chưa xác nhận.

Lưu ý: **da 0.40 chỉ có tác dụng che lỗ, không chịu lực.** Nó không phải "thành mỏng ở vùng bắt vít" theo nghĩa
kết cấu. Nhận xét trong review đúng ở chỗ có rủi ro, nhưng cơ chế hỏng thật là chiều dài vít.

**E-2 · Minor · ASSUMED. Đường truyền mô-men.**
- Đĩa horn tròn trong hốc tròn nên không có khóa hình dạng: toàn bộ mô-men đi qua 2 vít ở r = 7 mm.
- Mô-men kẹt SG90 là 0.118–0.177 N·m (các datasheet nêu 1.2 và 1.8 kgf·cm). Lực cắt mỗi vít 8.4–12.6 N, ứng suất
  dập lên nhựa 1.7–2.5 MPa.
- Mô-men trọng lực của đầu lớn nhất khoảng 20.4 N·mm (m = 71.3 g, CoM cách trục 29 mm, lấy từ URDF sinh từ CAD).
- Phần chưa tính: va đập khi ngã.

**E-3 · Major · ASSUMED. Các thành khác.**

| Vùng | Bề dày | Vai trò |
|---|---|---|
| Vành hốc horn | 1.80 | Chỉ định tâm, tải tĩnh khoảng 0.35 N |
| Vành nắp idler | 1.00 | Chỉ giữ nắp |
| Thành ổ bạc đạn dưới hốc nắp | 3.48 | Chịu lực |
| Đáy hốc horn | 2.80 (1.80 dưới hốc nắp) | |

Không cần sửa cho tải tĩnh. Cần thử ngã (V6) trước khi chốt.

**E-4 · Major · UNKNOWN. Rão, mỏi, hướng lớp.**
- PLA rão dưới tải liên tục và mềm đi từ khoảng 50–60 °C.
- Với hướng in O1, hai thanh đỡ đứng, nên ứng suất uốn ở gốc thanh vuông góc với lớp in, là hướng yếu nhất.
- Đây chính là chỗ FEA G-1 báo điểm nóng.

## F. Khe hở và va chạm trong hành trình

Quét −30…+32° bước 4°, khoảng cách tối thiểu tính bằng OCC BRepExtrema.

**F-1 · Major · FAILED.** Khe N3↔neck_link: **0.2 mm hướng kính** (lỗ lip Ø4.6 quanh boss Ø4.2, đồng trục) và
**0.8 mm dọc trục** (đĩa idler ↔ thành idler). Cả hai nhỏ hơn quy tắc `hardware.GAP_MOVING` = 1.0 mm của chính
dự án. Lỗ FDM thường in nhỏ hơn CAD, nên khe 0.2 mm có thể thành cọ.

**F-2 · Info · VERIFIED (chỉ phần có trong CAD).**
- Không cặp nào chồng khối.
- Khe nhỏ nhất ngoài trục là 0.8 mm (chính là khe dọc trục ở F-1).
- Vỏ đầu cách phần cổ và servo ≥ 7.16 mm trong toàn hành trình.
- Chưa mô hình: dây servo và cáp, đầu ốc tai và đầu vít horn, vỏ SG90 thật (đang dùng hộp MG92B, lớn hơn).

## G. Lắp ráp và tháo

**G-1 · Critical · FAILED.**
Để rút horn khỏi then hoa (hoặc rút N3 khỏi horn), N3 phải dịch **3.9 mm** dọc trục. Phía idler chỉ có **0.8 mm**
trước khi chạm neck_link. Mình đã kiểm bằng cách dịch khối CAD:

| Dịch dọc trục | 0.8 mm | 1.0 mm | 2.0 mm | 3.0 mm | 4.0 mm |
|---|---|---|---|---|---|
| Thể tích chồng với neck_link | 0 mm³ | 36 mm³ | 208 mm³ | 377 mm³ | 545 mm³ |

Ổ servo chỉ mở về phía trục và bị đĩa horn che, nên cũng không lắp được servo khi N3 đang ở chỗ. Kết luận: **phải
banh hai đĩa ra ≥ 3.1 mm**.

FEA dọc trục: k_a = 13.2 N/mm. Banh 3.1 mm cần khoảng **41 N**, gây ứng suất khoảng **32 MPa** ở gốc thanh đỡ
phía horn. Đây là FEA tuyến tính và góc trong chưa có bo, nên con số chỉ mang tính chỉ báo. Dù vậy nó đã ngang
giới hạn bền giữa các lớp của PLA in.

Hệ quả: trình tự tháo trong ghi chú L4 của bản vẽ P1 ("tháo ốc idler rồi nhấc N3 khỏi horn") không thực hiện được
nếu không banh khung. **Đây là phát hiện mà review (và bản P1) chưa nêu.**

## Đối chiếu với các nhận xét trong review

| Nhận xét | Kết luận audit |
|---|---|
| 2.1 Lỗ tán = OD tán | Đồng ý → A-1 FAILED. Mức sàn 3 mm / tán L3 → A-2 UNKNOWN |
| 2.2 K3 đồng trục TBD | Đồng ý, và tệ hơn: có sẵn lệch 0.3–0.4 mm do ổ MG92B (B-1); hốc horn không định tâm horn (B-3) |
| 2.3 Khe 31.9 chưa chứng minh | Đồng ý → C-2. Gốc rễ là định vị dọc trục kép (C-1) và lắp/tháo (G-1) |
| 2.4 Giữ bạc đạn | Đồng ý → D-1, D-2. Đề xuất định nghĩa gối tự do và lắp trượt |
| 2.5 Thành 0.40 | Cơ chế hỏng khác: da không chịu lực, lỗi thật là chiều dài vít (E-1) |
| 3.1 Hướng in chưa chứng minh | Đã cắt lát thật 4 hướng → `N3_DFM_DFA_REPORT.md` |
| 3.2 Support / doa | Đã kiểm G-code: support tự động rơi vào hốc horn và ổ bạc đạn → DFM report |
| 3.3 Thông số in | Đúng: chưa có profile máy in thật của bạn → yêu cầu M-6 |

## Cổng (gate) trước khi được sửa CAD

Theo quy tắc của task, CAD chỉ được sửa sau khi các mục sau có câu trả lời:

| # | Cần gì | Từ ai | Giải quyết mục |
|---|---|---|---|
| M-1 | Đo 1–2 con SG90: length, width, shaft_off, hole_pitch, hole_d, case_h, tab_low, tower_h, spline_top, **độ rơ hướng kính trục ra** | Bạn (thước cặp) | B-1, B-2, C-1, C-2 |
| M-2 | Horn tròn có thật trong bộ SG90 của bạn không (SG90 thường kèm horn tay đòn); nếu có: disc_d, disc_t, disc_low khi ép hết, lỗ có sẵn ở r = ? | Bạn | B-3, C-1, E-1 |
| M-3 | Mã tán cấy M2 đã mua (link hoặc ảnh + OD, L đo bằng thước cặp) | Bạn | A-1, A-2 |
| M-4 | Bạc đạn MR63ZZ đã mua (đo OD) | Bạn | D-1 |
| M-5 | Kết quả T0: sai số lỗ của máy in | Bạn | C-2, D-1, F-1 |
| M-6 | Máy in, đầu phun, profile slicer bạn sẽ dùng thật | Bạn | Thông số in P5 |
| D-1 | Chọn hướng sửa G-1 (3 phương án trong DFM/DFA report) | Bạn duyệt | G-1, C-1 |
| D-2 | Ổ + tán tai riêng cho SG90 trên neck_link (N2) | Bạn duyệt | B-1 |

## Nguồn

- Source CAD: `hardware/mechanical/joint.py`, `body.py`, `hardware.py`; URDF `bazan_cad.urdf` (khối lượng, CoM).
- MR63ZZ: OD và bore dung sai 0/−0.008, bề rộng 0/−0.12 — [TFL MR63zz](https://www.sdtflbearing.com/?p=11464),
  [SKD SMR63zz](https://www.skdin.com/products/productid-SMR63zz).
- Tán cấy M2: [TFC threaded inserts brochure](https://www.controlsdrivesautomation.com/orgfiles/ZORGF000011/CDA/EnhancedEntries/TFC/TFC%20Threaded%20Inserts%20Brochure%20UK.PDF),
  [Accu HTFBI-M2](https://accu-components.com/us/brass-threaded-inserts/985938-HTFBI-M2-BR).
- SG90 (số liệu mâu thuẫn): [datasheet copy 1](https://bit.nkust.edu.tw/~csshieh/teach/111A/mc/datasheet/SG90.pdf),
  [datasheet copy 2](https://datasheetspdf.com/pdf/791970/TowerPro/SG90/1),
  [Wiltronics HA9740](https://www.wiltronics.com.au/wp-content/uploads/datasheets/HA9740%20Data%20Sheet.pdf).
