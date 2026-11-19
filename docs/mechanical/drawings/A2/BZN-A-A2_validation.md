# BZN-A-A2 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py A2`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 29.2 = giá trị trên CAD | đạt | 29.200 |
| tờ 1 [front] 48.6 = giá trị trên CAD | đạt | 48.600 |
| tờ 1 [left] 29 = giá trị trên CAD | đạt | 29.000 |
| tờ 1 [left] 17 = giá trị trên CAD | đạt | 17.000 |
| tờ 1 [left] 36.6 = giá trị trên CAD | đạt | 36.600 |
| tờ 1 [top] 20.9 = giá trị trên CAD | đạt | 20.900 |
| tờ 1 [top] 7.3 = giá trị trên CAD | đạt | 7.300 |
| tờ 1 [front] 20.9 = giá trị trên CAD | đạt | 20.900 |
| tờ 1 [front] 27.8 = giá trị trên CAD | đạt | 27.800 |
| tờ 1 [front] 3.2 = giá trị trên CAD | đạt | 3.200 |
| tờ 1 [left] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [left] 29.6 = giá trị trên CAD | đạt | 29.600 |
| tờ 1 [front] 17.1 = giá trị trên CAD | đạt | 17.100 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [29.  29.2 48.6], ghi [29.0, 29.2, 48.6] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 9 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, C1, D1, D2, E1, E2, F1, G1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 9 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 164 < đứng y0 206; cạnh x0 141 > đứng x1 89 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 206, 89, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 135, 89, 164] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [141, 206, 170, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [338, 78, 369, 115] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 210, 84, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [134, 210, 154, 245] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
