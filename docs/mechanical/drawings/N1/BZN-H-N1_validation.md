# BZN-H-N1 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py N1`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 32 = giá trị trên CAD | đạt | 32.000 |
| tờ 1 [front] 22.7 = giá trị trên CAD | đạt | 22.700 |
| tờ 1 [left] 38.6 = giá trị trên CAD | đạt | 38.600 |
| tờ 1 [front] 16 = giá trị trên CAD | đạt | 16.000 |
| tờ 1 [front] 10.3 = giá trị trên CAD | đạt | 10.300 |
| tờ 1 [top] 25 = giá trị trên CAD | đạt | 25.000 |
| tờ 1 [top] 3.5 = giá trị trên CAD | đạt | 3.500 |
| tờ 1 [top] 28 = giá trị trên CAD | đạt | 28.000 |
| tờ 1 [top] 4 = giá trị trên CAD | đạt | 4.000 |
| tờ 1 [top] 16 = giá trị trên CAD | đạt | 16.000 |
| tờ 1 [top] 27.8 = giá trị trên CAD | đạt | 27.800 |
| tờ 1 [top] 7.6 = giá trị trên CAD | đạt | 7.600 |
| tờ 1 [top] 21.5 = giá trị trên CAD | đạt | 21.500 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [38.6 32.  22.7], ghi [22.7, 32.0, 38.6] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 8 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, B2, B3, B4, C1, C2, D1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 8 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 168 < đứng y0 210; cạnh x0 176 > đứng x1 124 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 210, 124, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 90, 124, 168] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [176, 210, 253, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [311, 68, 396, 139] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 236, 87, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [137, 236, 176, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
