# BZN-L-L4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 48.6 = giá trị trên CAD | đạt | 48.600 |
| tờ 1 [front] 75 = giá trị trên CAD | đạt | 75.000 |
| tờ 1 [left] 28.8 = giá trị trên CAD | đạt | 28.800 |
| tờ 1 [left] 15 = giá trị trên CAD | đạt | 15.000 |
| tờ 1 [left] 48 = giá trị trên CAD | đạt | 48.000 |
| tờ 1 [left] 15 = giá trị trên CAD | đạt | 15.000 |
| tờ 1 [left] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [left] 8 = giá trị trên CAD | đạt | 8.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [28.8 48.6 75. ], ghi [28.8, 48.6, 75.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 10 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2, C1, C2, C3, C4, D1, D2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 10 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 138 < đứng y0 180; cạnh x0 161 > đứng x1 109 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 180, 109, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 109, 109, 138] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [161, 180, 189, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [333, 76, 380, 146] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 222, 79, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [129, 222, 144, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
