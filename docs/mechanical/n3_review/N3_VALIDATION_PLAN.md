# N3 Head Frame — Validation Plan (T0 → P2)

Kế hoạch này biến các mục UNKNOWN và ASSUMED trong audit thành số đo.

**Thứ tự bắt buộc:** đo linh kiện (V0) → phiếu thử nhỏ (V1–V4) → lắp thử (V5) → chạy thử (V6) → mới sửa CAD và
nâng revision. **Không nâng P1 → P2 chỉ vì bản vẽ đẹp hơn.**

Mỗi kết quả ghi vào bảng ở cuối file, kèm ảnh. Sau mỗi lần đo, sửa `hardware.py` rồi chạy lại:

```powershell
cd hardware\mechanical
python n3_review.py
```

## Dụng cụ cần có

| Dụng cụ | Dùng cho |
|---|---|
| Thước cặp điện tử 0.01 mm (độ chính xác thực tế khoảng ±0.03) | V0–V5 |
| Mũi khoan (chuôi dùng làm chốt kiểm): Ø2.3, Ø2.4, Ø4.8, Ø5.0, Ø5.9, Ø6.0, Ø6.1 | V2, V3 |
| Cân treo hành lý (đọc N hoặc kg) + lục giác 1.5 mm | V1 (lực = cân × 9.81; mô-men = lực × tay đòn) |
| Nhiệt kế hồng ngoại hoặc can nhiệt | V6 |
| Robot đã có INA219 (đo dòng servo) | V6 |

## V0 · Đo linh kiện (M-1…M-4)

| Linh kiện | Đo gì | Ghi vào |
|---|---|---|
| SG90 (2 con nếu có) | length, width, case_h, shaft_off (tâm trục → đầu gần), tab_span, tab_t, tab_low, hole_pitch, hole_d, tower_h, spline_top | `hardware.SG90`, bỏ "SG90" khỏi `UNMEASURED` |
| SG90 | **Độ rơ hướng kính trục ra.** Kẹp vỏ servo, gắn horn, đẩy mép horn qua lại bằng lực khoảng 2 N, đo dịch chuyển tại r = 10 bằng thước cặp tựa cố định | Bảng kết quả (quyết định dung sai K3) |
| Horn kèm SG90 | Có horn tròn không? Nếu có: disc_d, disc_t, disc_low (ép hết vào trục), Ø và bán kính lỗ có sẵn | `hardware.HORN` |
| Tán M2 | Mã hoặc link mua; OD gai, L | `hardware.FASTENERS["INS-M2"]` |
| MR63ZZ | OD, bề rộng | Bảng kết quả |
| ST2×4 | Chiều dài dưới đầu, Ø ren, Ø đầu, cao đầu | Bảng kết quả |

**Đạt:** mọi ô có số. Script báo B-1, C-1, E-1 chuyển sang trạng thái tính từ số đo (không còn ASSUMED).

## V1 · Phiếu thử tán cấy (A-1, A-2)

- **Mẫu:** tấm 30 × 20 × 3 mm, 6 lỗ xuyên, đường kính theo datasheet của mã tán đã chọn: −0.1 / danh nghĩa / +0.1,
  mỗi cỡ 2 lỗ. In cùng hướng với sàn N3 (mặt trên lên).
- **Thao tác:** ép tán bằng mỏ hàn theo nhiệt độ của datasheet; để nguội 5 phút.

| Đo | Cách | Đạt | Loại |
|---|---|---|---|
| Phẳng mặt | Thước cặp đo độ cao tán so với mặt | 0…−0.1 (chìm nhẹ) | Nhô > 0.05 hoặc lệch nghiêng |
| Mô-men xoay | M2×6 + lục giác, treo cân ở tay đòn đã đo, tăng tới khi tán xoay | ≥ TBD (≥ mô-men siết khuyến nghị của mã tán × 2) | Xoay trước ngưỡng |
| Kéo tuột | Treo cân vào ốc M2 qua miếng chặn, kéo đều | ≥ TBD (ghi số, so với tải vỏ đầu khi ngã ở V6) | — |

Kết quả: Ø lỗ tán cho N3 (A1–A4, B1–B2) và cho mọi lỗ `ins_m2_thru` của robot.

## V2 · Phiếu thử ổ bạc đạn (D-1)

- **Mẫu:** khối 40 × 12 × 8, 5 ổ Ø5.95 / 6.00 / 6.05 / 6.10 / 6.15 sâu 4.8, có lip Ø4.6 dày 0.5. In **2 bản**:
  một bản trục lỗ nằm ngang (như O1), một bản trục lỗ đứng (như O2).

| Đo | Cách | Đạt (gối tự do) | Loại |
|---|---|---|---|
| Chốt | Chuôi mũi khoan Ø6.0 / Ø6.1 | Ø6.0 lọt bằng tay, Ø6.1 không lọt | Ø6.0 không lọt, hoặc Ø6.1 lọt |
| Độ oval | Thước cặp đo 3 phương cách 60° | Chênh ≤ TBD (ghi số) | — |
| Lắp MR63ZZ | Đẩy bằng ngón tay | Vào hết lip, trượt ra được bằng tay, không lắc ngang cảm nhận được | Phải dùng lực ép, hoặc lắc ngang |

Kết quả: Ø in cho ổ N3 theo từng hướng in; quyết định có cần doa hay không.

## V3 · N3 thật, hai hướng in (B-3, DFM §1)

In N3 theo **O1** (support chỉ từ bàn) và **O2**, mỗi hướng 1 chi tiết, dùng Ø ổ chốt từ V2.

| Đo | Cách | Ghi |
|---|---|---|
| Khe trong 31.9 | Thước cặp ngàm trong, đo ở 3 điểm | t_N3 |
| Tâm hốc horn và tâm ổ bạc đạn (X, Z từ datum A, B) | Đo khoảng cách từ mặt sàn (A) và mặt trước (B) tới mép trên/dưới, trước/sau của mỗi lỗ; tâm = trung bình | Đồng trục ≈ hiệu hai tâm (sai số đo khoảng ±0.05) |
| Độ tròn Ø20.4, Ø6.05 | 3 phương | — |
| Da 0.40 | Nhìn ngược sáng, ảnh | Kín / thủng |

**Đạt:** đồng trục đo được ≤ độ rơ trục ra SG90 (V0) và ≤ 0.1 mm (mục tiêu B-2). Giá trị đạt được trở thành dung
sai K3 ghi trên bản vẽ P2. Hướng in được chọn là hướng có đồng trục tốt hơn mà vẫn qua V5.

## V4 · Vít horn và da (E-1)

Trên N3 của V3: đặt horn thật vào hốc, bắt 2 × ST2×4 (hoặc vít đã chọn) qua lỗ đã khoan trên horn.

| Đạt | Loại |
|---|---|
| Horn nằm phẳng đáy hốc; mặt ngoài đĩa horn không phồng, không nứt, không thủng | Phồng / thủng / nứt da; hoặc vít tuột ren khi siết |

## V5 · Lắp thử cụm cổ–đầu (G-1, C-1, C-2, F-1)

Dùng neck_link (N2) in cùng máy, SG90 và horn thật, MR63ZZ, ốc thật. **Trước tiên thử banh U trên T2 (`foot`)**,
vì T2 dùng cùng kiểu khớp và rẻ hơn.

| Bước | Đo / quan sát | Đạt | Loại |
|---|---|---|---|
| Lắp N3 theo trình tự hiện tại | Có phải banh U không? Banh bao nhiêu (đo khe đĩa khi banh) | Không cần banh, **hoặc** banh được mà không trắng nhựa/nứt sau **5 lần** | Trắng nhựa, nứt, hoặc không lắp được |
| Lắp tự định tâm (DFM §5.3) | Thực hiện được | — | — |
| Khe đĩa idler ↔ neck_link | Thước lá / thước cặp | ≥ 0.2 | < 0.2 hoặc cọ |
| Lip ↔ boss | Quay bằng tay, nghe/nhìn cọ | Không cọ | Cọ |
| Quay hết −30…+32° bằng tay (chưa cấp điện) | Cảm giác | Êm đều, không điểm kẹt | Có điểm nặng |
| Dây servo, cáp camera và màn hình | Đi dây, quay hết hành trình | Không căng, không kẹp | Căng/kẹp |
| Tháo N3 để thay servo | Làm theo hướng dẫn | Tháo được, không hỏng | — |

## V6 · Chạy thử có điện

| Thử | Cách | Đạt | Loại |
|---|---|---|---|
| Dòng giữ | INA219: dòng servo đầu khi giữ 0°, đo (a) servo không tải, (b) có N3 | (b) ≤ TBD × (a), ghi số | Dòng tăng mạnh khi có N3 (dấu hiệu bó) |
| 200 chu kỳ −30…+32° | Script servo | Không kẹt, độ rơ tại mép sàn không tăng so với trước | Tăng độ rơ / tiếng kêu |
| Độ rơ | Đẩy mép sàn đầu theo X, Z bằng khoảng 2 N, đo dịch chuyển | Ghi số → tiêu chí P2 | — |
| Nhiệt | Giữ đầu 30 phút ở 0° và +32° | Vỏ servo ≤ 45 °C thì giữ PLA | > 45 °C thì xét PETG (M1) |
| Kẹt (stall) | Chặn đầu bằng tay ở +20°, 3 s, 10 lần | Không trượt horn, không nứt quanh vít | Trượt / nứt |
| Ngã | **TBD**: cần quyết định tải thử (độ cao, bề mặt) trước khi làm | — | — |

## Bảng kết quả (điền)

| Mục | Ngày | Giá trị | Ảnh | Ghi chú |
|---|---|---|---|---|
| V0 SG90 #1 | | | | |
| V0 SG90 #2 | | | | |
| V0 horn | | | | |
| V0 tán M2 | | | | |
| V0 MR63ZZ | | | | |
| V1 | | | | |
| V2 ngang | | | | |
| V2 đứng | | | | |
| V3 O1 | | | | |
| V3 O2 | | | | |
| V4 | | | | |
| V5 | | | | |
| V6 | | | | |

## Điều kiện để phát hành N3 rev P2

1. Không còn mục FAILED hay UNKNOWN nào trong `n3_review.json` liên quan giao diện (A, B, C, D, E-1, F-1, G-1).
2. V1–V6 đạt, có ảnh và số.
3. Dung sai K3, Ø ổ, Ø lỗ tán trên bản vẽ là **số đã đo được**, không phải số đẹp.
4. Bản vẽ có bảng lịch sử sửa đổi P1 → P2 với lý do từng thay đổi.
