# BZN-B-S1 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py S1`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 29 = giá trị trên CAD | đạt | 29.000 |
| tờ 1 [front] 9 = giá trị trên CAD | đạt | 9.000 |
| tờ 1 [left] 64 = giá trị trên CAD | đạt | 64.000 |
| tờ 1 [top] 19 = giá trị trên CAD | đạt | 19.000 |
| tờ 1 [top] 5 = giá trị trên CAD | đạt | 5.000 |
| tờ 1 [top] 55.9 = giá trị trên CAD | đạt | 55.900 |
| tờ 1 [top] 4.05 = giá trị trên CAD | đạt | 4.050 |
| tờ 1 [top] 8 = giá trị trên CAD | đạt | 8.000 |
| tờ 1 [top] 40 = giá trị trên CAD | đạt | 40.000 |
| tờ 1 [top] 12 = giá trị trên CAD | đạt | 12.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [64. 29.  9.], ghi [9.0, 29.0, 64.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 6 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, A3, A4, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 6 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 204 < đứng y0 246; cạnh x0 141 > đứng x1 89 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 246, 89, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 140, 89, 204] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [141, 246, 205, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [322, 66, 388, 109] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 250, 84, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [134, 250, 198, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
