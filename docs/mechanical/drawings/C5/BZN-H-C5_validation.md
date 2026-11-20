# BZN-H-C5 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py C5`. Lỗi: **1**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 49 = giá trị trên CAD | đạt | 49.000 |
| tờ 1 [front] 47.18 = giá trị trên CAD | đạt | 47.180 |
| tờ 1 [left] 8.43 = giá trị trên CAD | đạt | 8.432 |
| tờ 1 [front] 24.5 = giá trị trên CAD | đạt | 24.500 |
| tờ 1 [front] 54.16 = giá trị trên CAD | đạt | 54.163 |
| tờ 1 [front] 37.74 = giá trị trên CAD | đạt | 37.742 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [ 8.43 49.   47.18], ghi [8.43, 47.18, 49.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 2 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 2 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 166 < đứng y0 208; cạnh x0 161 > đứng x1 109 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 208, 109, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 157, 109, 166] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [161, 208, 169, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [335, 69, 372, 123] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | **LỖI** | '49' × 'A1' |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 212, 104, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [154, 212, 162, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
