# BZN-H-C4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py C4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 58.4 = giá trị trên CAD | đạt | 58.400 |
| tờ 1 [front] 70.72 = giá trị trên CAD | đạt | 70.724 |
| tờ 1 [left] 21.8 = giá trị trên CAD | đạt | 21.800 |
| tờ 1 [top] 40 = giá trị trên CAD | đạt | 40.000 |
| tờ 1 [top] 9.2 = giá trị trên CAD | đạt | 9.200 |
| tờ 1 [top] 11 = giá trị trên CAD | đạt | 11.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [21.8  58.4  70.72], ghi [21.8, 58.4, 70.72] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 4 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 4 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 142 < đứng y0 184; cạnh x0 170 > đứng x1 118 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 184, 118, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 120, 118, 142] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [170, 184, 192, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [335, 68, 381, 141] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 224, 84, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [134, 224, 145, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
