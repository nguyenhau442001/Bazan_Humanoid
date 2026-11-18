# N3 Head Frame — trạng thái sau rev P2

Sinh bởi `hardware/mechanical/n3_review.py` từ CAD hiện tại (không sửa tay). Audit P1 gốc: [N3_ENGINEERING_AUDIT.md](N3_ENGINEERING_AUDIT.md), số liệu P1: `n3_review_P1.json`. Số linh kiện lấy từ datasheet trên mạng (chưa đo): mọi mục dựa vào chúng là ASSUMED, không phải VERIFIED.

| ID | Mức | P1 | P2 | Vấn đề | Bằng chứng | Còn phải làm |
|---|---|---|---|---|---|---|
| A-1 | Critical | FAILED | **ASSUMED** | Lỗ tán M2 theo datasheet tán đã chọn | BOM: CNC Kitchen TC-M2x3.0: L3.0, knurl Ø3.6, hole Ø3.2, wall ≥ 1.3. Lỗ N3 Ø3.2; thành nhỏ nhất quanh 8 tán (gai Ø3.6) = 1.40 mm (≥ 1.3). | Đúng theo datasheet; xác nhận kéo tuột/xoay bằng V1. |
| A-2 | Major | UNKNOWN | **UNKNOWN** | Lực kéo tuột / mô-men xoay tán trong sàn 3 mm | Không có số liệu nhà sản xuất cho PLA in; cần phiếu thử. | V1. |
| B-1 | Critical | FAILED | **ASSUMED** | Trục SG90 so với trục idler trong ổ chung | Số SG90 theo TowerPro (lắp giống MG90S): lệch khi tâm tai trùng tán = -0.00 mm, sai bước tai 0.00. Thêm vào đó N3i bắt bằng lỗ hở Ø2.3 nên tự định tâm ±0.15 khi lắp. | Đo 1 con SG90 (M-1) để chuyển sang VERIFIED. |
| B-2 | Major | ASSUMED | **ASSUMED** | Tải ngang thường trực do lệch trục | k_b = 12.5 N/mm (FEA, N3 + N3i coi như liền), lệch còn lại 0.00 mm → 0.0 N (P1: 3.5–4.7 N). | Lắp tự định tâm (siết 2 ốc G/H sau cùng). |
| B-3 | Major | UNKNOWN | **UNKNOWN** | Đồng trục horn ↔ bạc đạn (K3) sau lắp | Không còn phụ thuộc in N3 (N3i tự định tâm). Còn: vị trí lỗ khoan horn, độ đảo then hoa. | Nghiệm thu bằng dòng giữ servo và quay tay (V5, V6). |
| C-1 | Major | FAILED | **UNKNOWN** | Chiều cao horn thật so với khe 0.8 | Ổ idler là gối tự do (lắp trượt) và N3i trôi ±0.15 dọc trục, nên không còn khóa hai đầu. Khe đĩa idler ↔ N2 = 0.8 − Δh. STEP MG90S cho Δh = 1.4 (> 0.8); datasheet cho Δh = 0. | Đo spline_top + disc_low (M-1, M-2). Nếu Δh > 0.4 → tăng GAP (cần duyệt, đổi bề rộng đầu). |
| C-2 | Major | UNKNOWN | **UNKNOWN** | Trường hợp xấu nhất khe 31.9 | Chờ Δh và sai số in (T0). | M-1, M-5. |
| D-1 | Critical | FAILED | **ASSUMED** | Kiểu lắp ổ bạc đạn | Định nghĩa: gối tự do, lắp trượt (hardware.HOLES['bearing']: Ø6.05, chốt Ø6.00 lọt / Ø6.10 không lọt). | Chốt Ø in bằng V2. |
| D-2 | Info | VERIFIED | **VERIFIED** | Giữ dọc trục một phía (đúng nguyên lý gối tự do) | joint.py. | — |
| E-1 | Critical | FAILED | **ASSUMED** | Vít horn và da 0.40 | ST2×3 qua horn 1.5: mũi vít cách đáy lỗ mồi 0.90 mm (horn ±0.3 → 0.60…1.20). | V4. |
| E-2 | Minor | ASSUMED | **ASSUMED** | Tải vít horn khi kẹt servo | 8.4–12.6 N mỗi vít, dập ≈ 4.2 MPa trên phần ren 1.5 mm. | V6 (kẹt). |
| E-4 | Major | UNKNOWN | **UNKNOWN** | Rão / mỏi / va đập | Chưa có dữ liệu. | V6. |
| F-1 | Minor | FAILED | **FAILED** | Khe N3/N3i ↔ N2 so với quy tắc 1.0 mm | Hướng kính lip Ø5.0 ↔ boss Ø4.2 = 0.4 mm (P1 0.2); dọc trục 0.8 mm. Hai khe này đồng trục quanh cùng gối bạc đạn, nên chỉ cọ nếu bạc đạn rơ; quy tắc 1.0 là cho khe giữa hai gối khác nhau. | Chấp nhận có điều kiện V5 (quay không cọ), hoặc tăng GAP (C-1). |
| F-2 | Info | VERIFIED | **VERIFIED** | Va chạm trong −30…+32° | 0 cặp chồng khối (CAD). | Dây, đầu ốc: V5. |
| G-1 | Critical | FAILED | **VERIFIED** | Lắp/tháo N3 không banh khung | Dịch khối CAD 4.4 mm dọc trục: chồng lớn nhất {'N3+horn': 0.0, 'N3i': 0.0} mm³ (horn trên then hoa bỏ qua). N3i tách rời, bắt 2 × M2×6 từ dưới. | — |
| G-2 | Critical | mới | **FAILED** | Cùng lỗi G-1 ở các khớp chân (joint.py dùng chung) | Khớp bị chặn sau (mm): {'hip_roll': 1.0, 'hip_pitch': 1.0, 'knee': 1.0, 'ankle_pitch': 1.0, 'ankle_roll': 1.0}. Sửa ở chân đổi hình ngoài (đường ghép, ốc) → cần duyệt cùng phương án gia cố ống chân. | Phương án như N3i cho từng driven link; ảnh trước/sau để duyệt. |
| H-1 | Major | mới | **ASSUMED** | Mặt kẹp vòng trong lớn hơn vai tối đa của bạc đạn | Boss Ø4.2 và long đền W3 Ø5.0 > db max 3.7 (NSK MF63): có thể chạm nắp chắn ZZ (quay theo vòng ngoài). Nắp chắn thường lõm so với mặt vòng, nên long đền phẳng thường không chạm; boss in thì không chắc phẳng. Không có long đền ≤ Ø3.7 bán phổ thông. | V5: siết ốc idler xong, vòng ngoài phải quay trơn; nếu nặng → thay boss bằng 1 shim thép. |
| H-2 | Minor | mới | **FAILED** | Tán M3 (gai Ø4.6) lớn hơn boss Ø4.2 chứa nó | Lỗ tán Ø4.0 bắt đầu ngay mặt boss: thành boss 0.1 mm, khi ép tán boss vỡ, mặt tán thành mặt kẹp. Chức năng vẫn còn (mặt đồng phẳng) nhưng hình học CAD không in được đúng. | Sửa joint.py (ảnh hưởng mọi khớp) cùng đợt G-2. |

FEA (N3 + N3i, PLA E 2300 giả định): k_b 12.5, k_w 10.2, k_a 13.6 N/mm.

Chạy lại: `cd hardware\mechanical; python n3_review.py` (khoảng 7 phút).
