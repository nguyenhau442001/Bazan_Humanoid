# N3 Head Frame — Interface Validation

Số liệu lấy từ `n3_review.py` (source CAD) và nguồn ghi trong cột "Nguồn". Kích thước của linh kiện mua được ghi
**giá trị CAD** và **giá trị đo thật**. Cột đo thật đang trống vì chưa có số đo nào. Đó là lý do mọi kiểm tra phụ
thuộc linh kiện mua đều chưa thể PASS.

## 1. Danh sách giao diện

| # | Giao diện | Đặc tính N3 | Linh kiện / chi tiết đối tiếp | Kiểu ghép mong muốn | Trạng thái |
|---|---|---|---|---|---|
| I-1 | Horn ↔ N3 | Hốc Ø20.4 ↧3.8, 2 lỗ mồi Ø1.6 r 7, da 0.40 | Horn tròn SG90 + 2 × ST2×4 | Định tâm + truyền mô-men | FAILED (E-1), UNKNOWN (M-2) |
| I-2 | Then hoa ↔ horn | — | Trục ra SG90 | Ép, vít tâm | UNKNOWN (M-1, M-2) |
| I-3 | Bạc đạn ↔ N3 | Ổ Ø6.05 ↧4.8, lip Ø4.6 ↧0.5 | MR63ZZ 3×6×2.5 | **Gối tự do: lắp trượt** (đề xuất) | FAILED (D-1) |
| I-4 | Bạc đạn ↔ neck_link | — | Boss Ø4.2 + tán M3 + W3 + M3×6 đầu dù | Kẹp vòng trong | VERIFIED trên CAD |
| I-5 | Lip ↔ boss | Lỗ Ø4.6 | Boss Ø4.2 (neck_link) | Khe động | FAILED (F-1: 0.2 < 1.0) |
| I-6 | Đĩa idler ↔ thành idler | Mặt trong đĩa idler | neck_link | Khe động dọc trục | FAILED (F-1: 0.8 < 1.0; C-1) |
| I-7 | Sàn ↔ vỏ đầu C3/C4 | 4 × lỗ tán Ø3.2 xuyên (A1–A4) | Tán M2 + M2×6 | Tán cấy nhiệt | FAILED (A-1) |
| I-8 | Sàn ↔ N4 | 2 × lỗ tán Ø3.2 xuyên (B1–B2) | Tán M2 + M2×6 | Tán cấy nhiệt | FAILED (A-1) |
| I-9 | Tấm mặt ↔ màn hình | 4 khe dây rút 1.6 × 3 | Dây rút 2.5 | Lắp lỏng | VERIFIED trên CAD |

## 2. Kích thước linh kiện mua

| Linh kiện | Thông số | CAD (`hardware.py`) | Datasheet | Đo thật | Ghi chú |
|---|---|---|---|---|---|
| SG90 | Dài × rộng | 22.2 × 11.8 | 22.2 × 11.8 / 23.1 × 12.2 / 22.0 × 11.5 | — | Các datasheet mâu thuẫn |
| SG90 | shaft_off | 5.9 | không ghi | — | Quyết định B-1 |
| SG90 | hole_pitch / hole_d | 27.6 / 2.0 | không ghi | — | |
| SG90 | tower_h / spline_top | 4.0 / 8.3 | không ghi | — | STEP MG90S: 5.75 / 9.7 |
| SG90 | Độ rơ hướng kính trục ra | — | không ghi (bạc nhựa) | — | Quyết định dung sai K3 |
| SG90 | Mô-men kẹt | 0.177 N·m (URDF) | 1.8 / 1.2 kgf·cm | — | Mâu thuẫn |
| Horn | disc_d / disc_t / disc_low | 20 / 1.5 / 6.9 | — | — | Có kèm SG90 không? (M-2) |
| Horn | Lỗ vít ở r 7 | "drill if missing" | — | — | Khoan tay → sai số K3 |
| MR63ZZ | OD / bề rộng | 6 / 2.5 | 6 (0/−0.008) / 2.5 (0/−0.12) | — | VERIFIED theo datasheet |
| Tán M2 | OD × L | 3.2 × 3 | 3.6–3.7 × 3.9–4 cho lỗ Ø3.2 | — | BOM chưa có mã |
| ST2×4 | Chiều dài dưới đầu | 4 | — | — | E-1 |

## 3. Chuỗi kích thước

### 3.1 Dọc trục (trục gật, a = 0 tại mặt vỏ servo)

| Phần tử | Từ | Đến | Dày |
|---|---|---|---|
| Đĩa idler N3 (IP) | −33.60 | −27.30 | 6.30 *(gồm nắp 1.0, đầu ốc, long đền)* |
| Bạc đạn (BRG) | −30.30 | −27.80 | 2.50 |
| Lip N3 | −27.80 | −27.30 | 0.50 |
| Khe đĩa idler ↔ thành | −27.30 | −26.50 | **0.80** |
| Thành idler neck_link (IW) | −26.50 | −22.90 | 3.60 |
| Khe đáy vỏ | −22.90 | −22.70 | 0.20 |
| Vỏ servo | −22.70 | 0.00 | 22.70 |
| Tháp | 0.00 | 4.00 | 4.00 |
| Khe tháp ↔ đĩa horn | 4.00 | 4.60 | 0.60 |
| Đĩa horn N3 (HP) | 4.60 | 11.20 | 6.60 |

Khe trong: 4.60 − (−27.30) = **31.90**.

Chuỗi này bị khóa ở hai đầu (C-1), qua mặt horn tại a = 8.4 và qua lip/vòng trong tại a = −27.8.

**Điều kiện xấu nhất** (đề xuất, khe động tối thiểu 0.2): **0.8 − Δh − t_N3 − t_N2 ≥ 0.2**.

| Đại lượng | Ý nghĩa | Giá trị |
|---|---|---|
| Δh | Horn thật cao hơn CAD | UNKNOWN (M-1, M-2). Tham khảo: STEP MG90S = +1.4 → FAILED nếu SG90 giống |
| t_N3 | Sai số in của khe 31.9 | UNKNOWN (M-5) |
| t_N2 | Sai số vị trí thành idler neck_link | UNKNOWN (M-5) |

Với giả định t_N3 = t_N2 = 0.1 (chưa đo), Δh cho phép ≤ 0.4 mm.

### 3.2 Hướng kính (đồng trục hai gối)

| Nguồn lệch | Giá trị | Trạng thái |
|---|---|---|
| Trục ra SG90 so với tán M3 idler (ổ MG92B) | +0.30, tới 0.40 mm | FAILED (B-1), số ASSUMED |
| Khe ổ servo khi tai lỏng | b −0.20…+0.80, w ±0.40 | ASSUMED |
| Horn trong hốc: khe hướng kính | 0.20 mm | VERIFIED; vị trí thật do vít/lỗ khoan quyết định |
| Lỗ khoan tay trên horn | UNKNOWN | — |
| Đồng trục hốc horn ↔ ổ bạc đạn khi in | UNKNOWN | Phiếu V3 |
| Khe ổ bạc đạn | +0.050…+0.058 trên CAD | Phụ thuộc máy in (M-5) |

**Tác dụng của lệch (FEA, ASSUMED E = 2300 MPa):**
- Độ cứng tương đối của khung: k_b = 11.7, k_w = 10.1 N/mm.
- Mỗi 0.1 mm lệch tạo khoảng 1.2 N tải ngang thường trực lên trục ra servo.

### 3.3 Hành trình

| Kiểm tra | Kết quả | Trạng thái |
|---|---|---|
| Hành trình | −30…+32° (`robot.RANGES`) | — |
| Va chạm khung ↔ cổ | không (khe ngoài trục nhỏ nhất 0.8) | VERIFIED (CAD) |
| Vỏ đầu ↔ cổ/servo | ≥ 7.16 mm | VERIFIED (CAD) |
| Dây, đầu ốc, servo thật | chưa mô hình | UNKNOWN → lắp thử V5 |

## 4. Rủi ro giao diện còn mở

| Rủi ro | Hậu quả nếu xảy ra | Chặn bởi |
|---|---|---|
| Δh > 0.4 | Đĩa idler cọ neck_link, lực dọc trục vào then hoa | M-1, M-2, D-1 |
| Lệch trục > độ rơ SG90 | Tải ngang thường trực, mòn bánh răng ra | M-1, D-2 |
| Tán xoay khi siết | Không bắt được vỏ/N4 | M-3, V1 |
| Ổ bạc đạn chặt | Mất tính tự do dọc trục → C-1 | M-4, M-5, V2 |
| Vít horn thủng da | Mặt ngoài nứt, phồng | M-2, V4 |
| Banh U 3.1 mm khi lắp | Nứt gốc thanh đỡ | D-1 (thiết kế), V5 |
