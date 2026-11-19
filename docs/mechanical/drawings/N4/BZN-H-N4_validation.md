# BZN-H-N4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py N4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 32 = giá trị trên CAD | đạt | 32.000 |
| tờ 1 [front] 32.84 = giá trị trên CAD | đạt | 32.841 |
| tờ 1 [left] 29.79 = giá trị trên CAD | đạt | 29.789 |
| tờ 1 [front] 21 = giá trị trên CAD | đạt | 21.000 |
| tờ 1 [front] 5.5 = giá trị trên CAD | đạt | 5.500 |
| tờ 1 [front] 22.46 = giá trị trên CAD | đạt | 22.458 |
| tờ 1 [front] 5.34 = giá trị trên CAD | đạt | 5.338 |
| tờ 1 [top] 20 = giá trị trên CAD | đạt | 20.000 |
| tờ 1 [top] 6 = giá trị trên CAD | đạt | 6.000 |
| tờ 1 [top] 6 = giá trị trên CAD | đạt | 6.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [29.79 32.   32.84], ghi [29.79, 32.0, 32.84] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 8 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, A3, A4, A5, A6, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 8 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 147 < đứng y0 189; cạnh x0 176 > đứng x1 124 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 189, 124, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 88, 124, 147] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [176, 189, 236, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [320, 73, 388, 134] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 193, 119, 227] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [169, 193, 229, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
