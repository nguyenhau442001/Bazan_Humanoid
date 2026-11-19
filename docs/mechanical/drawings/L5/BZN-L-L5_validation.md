# BZN-L-L5 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L5`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 48.6 = giá trị trên CAD | đạt | 48.600 |
| tờ 1 [front] 71.6 = giá trị trên CAD | đạt | 71.600 |
| tờ 1 [left] 25.8 = giá trị trên CAD | đạt | 25.800 |
| tờ 1 [left] 13.8 = giá trị trên CAD | đạt | 13.800 |
| tờ 1 [left] 59.6 = giá trị trên CAD | đạt | 59.600 |
| tờ 1 [top] 18.2 = giá trị trên CAD | đạt | 18.200 |
| tờ 1 [top] 13.8 = giá trị trên CAD | đạt | 13.800 |
| tờ 1 [left] 27.8 = giá trị trên CAD | đạt | 27.800 |
| tờ 1 [left] 3.2 = giá trị trên CAD | đạt | 3.200 |
| tờ 1 [left] 11.6 = giá trị trên CAD | đạt | 11.600 |
| tờ 1 [left] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [left] 6.8 = giá trị trên CAD | đạt | 6.800 |
| tờ 1 [left] 17.1 = giá trị trên CAD | đạt | 17.100 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [25.8 48.6 71.6], ghi [25.8, 48.6, 71.6] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 10 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, C1, D1, D2, E1, F1, F2, G1, H1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 10 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 141 < đứng y0 183; cạnh x0 161 > đứng x1 109 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 183, 109, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 116, 109, 141] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [161, 183, 186, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [330, 74, 380, 146] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 223, 79, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [129, 223, 142, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
