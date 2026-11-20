# BZN-L-L4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 58.5 = giá trị trên CAD | đạt | 58.500 |
| tờ 1 [front] 132 = giá trị trên CAD | đạt | 132.000 |
| tờ 1 [left] 33.8 = giá trị trên CAD | đạt | 33.800 |
| tờ 1 [left] 17.5 = giá trị trên CAD | đạt | 17.500 |
| tờ 1 [left] 100 = giá trị trên CAD | đạt | 100.000 |
| tờ 1 [left] 17.5 = giá trị trên CAD | đạt | 17.500 |
| tờ 1 [left] 18 = giá trị trên CAD | đạt | 18.000 |
| tờ 1 [left] 8.5 = giá trị trên CAD | đạt | 8.500 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [ 33.8  58.5 132. ], ghi [33.8, 58.5, 132.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 10 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2, C1, C2, C3, C4, D1, D2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 10 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 147 < đứng y0 189; cạnh x0 141 > đứng x1 89 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 189, 89, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 130, 89, 147] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [141, 189, 158, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [341, 73, 371, 136] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 193, 84, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [134, 193, 151, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
