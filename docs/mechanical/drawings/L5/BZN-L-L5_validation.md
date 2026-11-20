# BZN-L-L5 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L5`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 58.5 = giá trị trên CAD | đạt | 58.500 |
| tờ 1 [front] 133.25 = giá trị trên CAD | đạt | 133.250 |
| tờ 1 [left] 30.8 = giá trị trên CAD | đạt | 30.800 |
| tờ 1 [left] 16.3 = giá trị trên CAD | đạt | 16.300 |
| tờ 1 [left] 118.75 = giá trị trên CAD | đạt | 118.750 |
| tờ 1 [top] 32 = giá trị trên CAD | đạt | 32.000 |
| tờ 1 [top] 16.3 = giá trị trên CAD | đạt | 16.300 |
| tờ 1 [left] 10 = giá trị trên CAD | đạt | 10.000 |
| tờ 1 [left] 11.3 = giá trị trên CAD | đạt | 11.300 |
| tờ 1 [left] 49.5 = giá trị trên CAD | đạt | 49.500 |
| tờ 1 [left] 4 = giá trị trên CAD | đạt | 4.000 |
| tờ 1 [left] 18.75 = giá trị trên CAD | đạt | 18.750 |
| tờ 1 [left] 18 = giá trị trên CAD | đạt | 18.000 |
| tờ 1 [left] 7.3 = giá trị trên CAD | đạt | 7.300 |
| tờ 1 [left] 24.25 = giá trị trên CAD | đạt | 24.250 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [ 30.8   58.5  133.25], ghi [30.8, 58.5, 133.25] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 12 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, C1, D1, D2, D3, D4, E1, F1, F2, G1, H1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 12 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 146 < đứng y0 188; cạnh x0 141 > đứng x1 89 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 188, 89, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 131, 89, 146] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [141, 188, 157, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [339, 70, 371, 136] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 192, 84, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [134, 192, 150, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
