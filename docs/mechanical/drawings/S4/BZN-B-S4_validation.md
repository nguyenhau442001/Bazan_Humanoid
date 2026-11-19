# BZN-B-S4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py S4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 66 = giá trị trên CAD | đạt | 66.000 |
| tờ 1 [front] 6 = giá trị trên CAD | đạt | 6.000 |
| tờ 1 [left] 29 = giá trị trên CAD | đạt | 29.000 |
| tờ 1 [top] 40 = giá trị trên CAD | đạt | 40.000 |
| tờ 1 [top] 13 = giá trị trên CAD | đạt | 13.000 |
| tờ 1 [top] 14.5 = giá trị trên CAD | đạt | 14.500 |
| tờ 1 [top] 50 = giá trị trên CAD | đạt | 50.000 |
| tờ 1 [top] 8 = giá trị trên CAD | đạt | 8.000 |
| tờ 1 [top] 30.7 = giá trị trên CAD | đạt | 30.700 |
| tờ 1 [top] 0.85 = giá trị trên CAD | đạt | 0.850 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [29. 66.  6.], ghi [6.0, 29.0, 66.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 6 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2, B3, B4 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 6 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 207 < đứng y0 249; cạnh x0 178 > đứng x1 126 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 249, 126, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 178, 126, 207] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [178, 249, 207, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [322, 66, 388, 110] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 253, 121, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [171, 253, 200, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
