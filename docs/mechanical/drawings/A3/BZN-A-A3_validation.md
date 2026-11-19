# BZN-A-A3 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py A3`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 28.2 = giá trị trên CAD | đạt | 28.200 |
| tờ 1 [front] 70.1 = giá trị trên CAD | đạt | 70.100 |
| tờ 1 [left] 29 = giá trị trên CAD | đạt | 29.000 |
| tờ 1 [front] 12 = giá trị trên CAD | đạt | 12.000 |
| tờ 1 [front] 58.1 = giá trị trên CAD | đạt | 58.100 |
| tờ 1 [top] 20.9 = giá trị trên CAD | đạt | 20.900 |
| tờ 1 [top] 8.3 = giá trị trên CAD | đạt | 8.300 |
| tờ 1 [left] 8.3 = giá trị trên CAD | đạt | 8.300 |
| tờ 1 [left] 27.8 = giá trị trên CAD | đạt | 27.800 |
| tờ 1 [left] 3.2 = giá trị trên CAD | đạt | 3.200 |
| tờ 1 [front] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [front] 5 = giá trị trên CAD | đạt | 5.000 |
| tờ 1 [left] 17.1 = giá trị trên CAD | đạt | 17.100 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [29.  28.2 70.1], ghi [28.2, 29.0, 70.1] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 8 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, C1, C2, D1, D2, E1, F1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 8 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 143 < đứng y0 185; cạnh x0 140 > đứng x1 88 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 185, 88, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 114, 88, 143] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [140, 185, 169, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [341, 73, 372, 132] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [59, 224, 69, 241] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [119, 224, 134, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
