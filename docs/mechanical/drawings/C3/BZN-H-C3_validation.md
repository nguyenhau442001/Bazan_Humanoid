# BZN-H-C3 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py C3`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 65.2 = giá trị trên CAD | đạt | 65.200 |
| tờ 1 [front] 70.72 = giá trị trên CAD | đạt | 70.724 |
| tờ 1 [left] 61.62 = giá trị trên CAD | đạt | 61.616 |
| tờ 1 [top] 40 = giá trị trên CAD | đạt | 40.000 |
| tờ 1 [top] 12.6 = giá trị trên CAD | đạt | 12.600 |
| tờ 1 [top] 19 = giá trị trên CAD | đạt | 19.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [61.62 65.2  70.72], ghi [61.62, 65.2, 70.72] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 4 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 4 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 142 < đứng y0 184; cạnh x0 177 > đứng x1 125 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 184, 125, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 81, 125, 142] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [177, 184, 239, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [323, 78, 383, 144] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 224, 88, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [138, 224, 168, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
