# N3 Head Frame — DFM / DFA Report (Phase 2–3)

Số liệu từ `n3_review.py`: cắt lát PrusaSlicer 2.7 thật trên khối N3 xuất từ CAD, kiểm tra G-code, FEA CalculiX,
và dịch/quét khối CAD.

**Profile cắt lát:** đầu 0.4, lớp 0.2, 4 perimeter, 5 lớp trên / 4 lớp dưới, gyroid 40 %, PLA. Đây là profile
**phân tích** dùng chung với `test_prints`, **không phải** profile máy in của bạn (M-6). Vì vậy thời gian và khối
lượng chỉ dùng để so sánh các hướng in, không dùng để lập kế hoạch.

## 1. Hướng in: so sánh bằng cắt lát thật

| Hướng | Cao (mm) | PLA (g) | Thời gian (phút, không support) | Trục gật |
|---|---|---|---|---|
| **O1 sàn úp xuống (P1)** | 35 | 17.5 | 128 | ngang |
| O2 đĩa horn xuống bàn | 48 | 18.3 | 87 | dọc |
| O3 đĩa idler xuống bàn | 48 | 18.3 | 87 | dọc |
| O4 tấm mặt xuống bàn | 54 | 18.3 | 109 | ngang |

Support và chiều dài đường đùn support rơi vào các lỗ chức năng (đo từ G-code):

| Hướng | Support tự động (g) | Hốc horn | Ổ bạc đạn | Lỗ thao tác | Lip | Support chỉ từ bàn (g) | Hốc horn | Ổ bạc đạn | Lỗ thao tác |
|---|---|---|---|---|---|---|---|---|---|
| **O1** | 2.26 | **4154 mm** | **596 mm** | 263 | 31 | **0.04** | 0 | 0 | 0 |
| O2 | 7.87 | 3809 | 0 | 329 | 0 | 1.36 | 248 | 0 | 294 |
| O3 | 11.02 | 2919 | 537 | 0 | 30 | 5.72 | 313 | 480 | 0 |
| O4 | 5.21 | 4273 | 619 | 268 | 32 | 0.56 | 0 | 0 | 0 |

Cách đo: mỗi đoạn đùn có nhãn `;TYPE:Support material` được chiếu ngược về hệ tọa độ chuẩn O của N3, rồi kiểm
xem nó có nằm trong hình trụ của lỗ chức năng hay không.

### Nhận xét

1. **Ghi chú P3 của P1 ("cấm support trong hốc horn, ổ bạc đạn, lỗ thao tác") mâu thuẫn với chính hướng in P1
   nếu bật support tự động.** Slicer đặt khoảng 4.2 m đường đùn support vào hốc horn và khoảng 0.6 m vào ổ bạc
   đạn. Với "support on build plate only", O1 còn 0.04 g support và **không có support trong lỗ chức năng nào**
   (VERIFIED trên G-code). Cái giá: các lỗ ngang (Ø20.4, Ø6.05, Ø5.2, Ø4.6) in không có đỡ, đỉnh lỗ là mặt treo.
   Độ võng phụ thuộc máy và phải đo (V2, V3).
2. **O2 (trục dọc)** cho lỗ tròn và mặt ổ bạc đạn, lip sạch support. Đổi lại cao hơn 13 mm và cần 1.36 g support
   (đứng trên bàn, chui qua lỗ thao tác vào hốc horn). Đồng trục khi đó phụ thuộc độ thẳng trục Z và lệch lớp
   (layer shift) trên 31.9 mm, thay vì sai số XY như O1.
3. **O3** loại: support nhiều nhất, và vẫn có support trong ổ bạc đạn ngay cả khi chỉ cho support từ bàn.
   **O4** loại: cao nhất, lỗ vẫn ngang như O1, không có lợi gì thêm.
4. **Hướng lớp và điểm nóng ứng suất:**
   - Điểm nóng của G-1 nằm ở gốc thanh đỡ phía horn, chỗ thanh nối sàn.
   - O1: thanh đứng, nên ứng suất uốn ở gốc **vuông góc với lớp** (yếu nhất).
   - O2: thanh nằm, ứng suất uốn **dọc theo lớp**, nhưng sàn lại đứng.
   - Chưa thể chọn hướng nào chỉ bằng lý luận.

**Kết luận DFM:** chưa chọn hướng in. Đề xuất in N3 cả O1 (support chỉ từ bàn) và O2 trong phiếu V3, đo độ tròn,
đồng trục và thử banh (G-1), rồi mới chốt. Đây đúng là điều review yêu cầu: hướng in phải dựa trên bằng chứng thực
nghiệm.

## 2. Các đặc tính khó in

| Đặc tính | Vấn đề | Đề xuất (không đổi hình ngoài) |
|---|---|---|
| Hốc Ø20.4 nằm ngang (O1) | Cung trên treo khoảng 14 mm (nơi góc > 45°) | Lỗ dạng giọt nước (teardrop) hoặc chấp nhận, kiểm bằng V3 |
| Ổ Ø6.05 nằm ngang (O1) | Võng ở đỉnh làm ổ chặt | Chốt Ø theo V2; nếu cần doa, theo §3 |
| Da 0.40 trên lỗ mồi | Nhỏ hơn 1 đường đùn; slicer sinh 13–17 mm đùn trong vùng da (có in, độ kín chưa rõ) | Theo E-1: lỗ mồi xuyên hoặc vít ngắn hơn |
| Lỗ lip Ø4.6 | Khe 0.2 với boss (F-1); lỗ in nhỏ là cọ | Ø5.0 (cần duyệt) hoặc doa |
| Lỗ tán Ø3.2 xuyên | Phụ thuộc mã tán (A-1) | Chờ M-3 |
| Góc trong thanh ↔ sàn | Không bo; FEA báo điểm nóng tại đây | Bo R ≥ 2 (đổi hình bên trong, cần duyệt) |

## 3. Hậu xử lý (thay cho P4 "sửa bằng dao/mũi doa")

| Lỗ | Được doa? | Dụng cụ | Tiêu chí |
|---|---|---|---|
| Ổ bạc đạn Ø6.05 | Chỉ khi V2 cho thấy cần | Mũi doa tay Ø6 H7, **dẫn hướng bằng bạc dẫn in đặt trong hốc horn** (đồ gá, cần thiết kế và duyệt) để giữ đồng trục | Chốt Ø6.00 lọt, Ø6.10 không lọt; MR63ZZ trượt dọc trục bằng tay |
| Lỗ lip Ø4.6 | Có | Mũi khoan Ø4.8–5.0 bằng tay | Boss Ø4.2 quay không cọ |
| Hốc horn Ø20.4 | Không | Chỉ cạo bavia | Horn đặt vào không cấn |
| Lỗ thao tác Ø5.2 | Có | Mũi khoan Ø5.2 | Tua vít PH0 lọt |
| Lỗ tán Ø3.2 | Không doa | — | Theo datasheet tán |

**Lỗ nào bị oval > TBD (từ V3) thì loại chi tiết, không doa cứu.** Doa tự do (không dẫn hướng) ổ bạc đạn là
**cấm**, vì nó phá đồng trục K3.

## 4. Phiếu quy trình in (template, M-6)

Spec này phải đi kèm mã bản vẽ: `BZN-H-N3-PS` liên kết với `BZN-H-N3`. Mọi giá trị máy là **TBD** cho đến khi bạn
cung cấp.

| Thông số | Giá trị | Nguồn |
|---|---|---|
| Máy in / firmware | TBD | M-6 |
| Đầu phun | TBD (phân tích dùng 0.4) | M-6 |
| Lớp | TBD (phân tích dùng 0.2) | M-6 |
| Perimeter / top / bottom | TBD (phân tích 4 / 5 / 4) | M-6 |
| Infill | TBD (phân tích gyroid 40 %) | M-6 |
| Vật liệu: hãng, màu, lô, sấy | TBD | M-6 |
| Nhiệt độ đầu / bàn | TBD | M-6 |
| Slicer + phiên bản profile | TBD | M-6 |
| Hướng in | O1 hoặc O2, chốt sau V3 | §1 |
| Support | **Chỉ từ bàn**; cấm trong Ø20.4, Ø6.05, Ø5.2, Ø4.6 | §1 (G-code) |
| Bù lỗ XY | Theo T0 | M-5 |

## 5. Lắp ráp và bảo trì (DFA)

### 5.1 Trình tự P1 có làm được không?

| Bước P1 | Làm được? | Bằng chứng |
|---|---|---|
| Tháo vỏ C3/C4 | UNKNOWN | Bản vẽ C3/C4 chưa vẽ lại |
| Tháo N4 (2 × M2×6 từ trên) | VERIFIED trên CAD: ốc từ trên, lỗ bậc trên N4 | `body.py` |
| Tháo ốc idler M3 (lục giác 2.0 qua phía nắp) | VERIFIED: đầu dù Ø5.7 nằm trong ổ Ø6.05 | `joint.py` |
| Vặn ốc tâm horn qua lỗ Ø5.2 | VERIFIED: đường dao PH0 Ø3 lọt | `joint.tool_paths` |
| **Nhấc N3 khỏi horn** | **FAILED** nếu không banh U ≥ 3.1 mm (≈ 41 N, ≈ 32 MPa) | G-1 |
| Thay servo | Phụ thuộc bước trên | G-1 |
| Thay bạc đạn | Làm được: đẩy ra phía nắp bằng chốt Ø4 qua lỗ lip. Lực đi qua bi, nên chỉ làm khi thay bạc mới | `joint.py` |
| Đi dây servo, cáp | UNKNOWN: dây chưa mô hình | V5 |

### 5.2 Phương án sửa G-1 + C-1 (để bạn chọn; chưa sửa CAD)

| | (1) Tăng khe GAP 0.8 → ≥ 4.4 + gối idler tự do | (2) Đĩa idler tháo rời | (3) Horn bắt từ ngoài, N3 lắp theo hướng kính |
|---|---|---|---|
| Ý tưởng | Đủ chỗ trượt 3.9 mm; vòng ngoài lắp trượt nên bù được Δh | Đĩa idler bắt 2 ốc M2 vào thanh; tháo đĩa xong N3 trượt ra | Mặt horn phẳng, ốc từ ngoài vào horn; boss phẳng; N3 hạ từ trên xuống |
| Giải quyết | G-1, C-1 (Δh cho phép lớn) | G-1 | G-1, C-1 |
| Đổi chi tiết | N3 (đĩa idler dịch ra 3.6), N2 (boss dài thêm) | N3 tách 2 phần + 2 tán | N3 (hốc horn), N2 (boss), horn (cần lỗ ren hoặc horn nhôm) |
| Hình ngoài | Đầu rộng thêm khoảng 3.6 mm ở phía idler: **phải kiểm vỏ C3/C4** | Không đổi | Gần như không đổi (nắp che) |
| Rủi ro | Boss dài hơn, ốc M3 chịu uốn | Thêm một mối ghép vào chuỗi đồng trục K3 (cần chốt định vị) | Ren trong horn nhựa yếu; spline horn nhôm cho SG90: UNKNOWN |
| Ảnh hưởng toàn robot | Mọi khớp dùng `joint.py` | Chỉ N3 (hoặc từng khớp) | Mọi khớp dùng `joint.py` |

Mình **không đề xuất chọn ngay**. Trước hết cần thử V5 trên bản in T2 (`foot`, cùng kiểu khớp) để biết banh 3 mm
có nứt thật không. Nếu không nứt và không trắng nhựa sau 5 lần, phương án "chấp nhận banh có kiểm soát + ghi
hướng dẫn" cũng có thể được xem xét.

### 5.3 Lệch trục B-1: lắp tự định tâm

Đây là cách giảm lệch trục mà không cần sửa CAD, có thể dùng ngay cho lần lắp thử:

1. Đặt servo vào ổ, ốc tai **chỉ vặn vào, chưa siết**. Ổ có khe b −0.20…+0.80 và w ±0.40.
2. Lắp N3: horn lên then hoa, rồi ốc idler M3 siết đủ.
3. Quay khớp hết hành trình 3 lần bằng tay.
4. Siết 2 ốc tai ở vị trí giữa hành trình.

Điều kiện: lỗ tai SG90 phải khoan Ø2.4 (đã có trong `test_print.md`) để servo trôi được khoảng 0.2. Nếu lệch thật
(M-1) > 0.2 thì cách này không đủ, khi đó cần D-2 (ổ riêng cho SG90 trên N2).

## 6. Khuyến nghị bảo trì

- Ốc tai và ốc idler: kiểm tra lại sau 10 giờ chạy đầu tiên, vì PLA rão (E-4).
- Nếu servo đầu nóng > 45 °C khi giữ đầu (đo theo V6), xét đổi sang PETG (tiêu chí M1 của bản vẽ).
- Ghi số lần tháo lắp N3 nếu chọn phương án banh có kiểm soát; thay N3 khi thấy vết trắng ở gốc thanh.
