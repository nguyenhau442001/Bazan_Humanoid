# BZN-C-C1 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py C1`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 116.4 = giá trị trên CAD | đạt | 116.400 |
| tờ 1 [front] 95.5 = giá trị trên CAD | đạt | 95.500 |
| tờ 1 [left] 55.2 = giá trị trên CAD | đạt | 55.200 |
| tờ 1 [top] 102 = giá trị trên CAD | đạt | 102.000 |
| tờ 1 [top] 7.2 = giá trị trên CAD | đạt | 7.200 |
| tờ 1 [top] 35 = giá trị trên CAD | đạt | 35.000 |
| tờ 1 [top] 96 = giá trị trên CAD | đạt | 96.000 |
| tờ 1 [top] 10.2 = giá trị trên CAD | đạt | 10.200 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [ 55.2 116.4  95.5], ghi [55.2, 95.5, 116.4] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 4 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 4 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 165 < đứng y0 207; cạnh x0 170 > đứng x1 118 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 207, 118, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 138, 118, 165] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [170, 207, 198, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [325, 68, 383, 138] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 211, 113, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [163, 211, 191, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
