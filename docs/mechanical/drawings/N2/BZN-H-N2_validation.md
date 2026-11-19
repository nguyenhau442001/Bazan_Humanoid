# BZN-H-N2 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py N2`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 27.3 = giá trị trên CAD | đạt | 27.300 |
| tờ 1 [front] 23.2 = giá trị trên CAD | đạt | 23.200 |
| tờ 1 [left] 34.6 = giá trị trên CAD | đạt | 34.600 |
| tờ 1 [top] 15.3 = giá trị trên CAD | đạt | 15.300 |
| tờ 1 [top] 22.6 = giá trị trên CAD | đạt | 22.600 |
| tờ 1 [front] 7.3 = giá trị trên CAD | đạt | 7.300 |
| tờ 1 [front] 14.9 = giá trị trên CAD | đạt | 14.900 |
| tờ 1 [left] 27.8 = giá trị trên CAD | đạt | 27.800 |
| tờ 1 [left] 3.2 = giá trị trên CAD | đạt | 3.200 |
| tờ 1 [left] 14.9 = giá trị trên CAD | đạt | 14.900 |
| tờ 1 [left] 22.6 = giá trị trên CAD | đạt | 22.600 |
| tờ 1 [top] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [top] 8.3 = giá trị trên CAD | đạt | 8.300 |
| tờ 1 [left] 17.1 = giá trị trên CAD | đạt | 17.100 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [34.6 27.3 23.2], ghi [23.2, 27.3, 34.6] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 10 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, C1, C2, D1, E1, E2, F1, G1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 10 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 167 < đứng y0 209; cạnh x0 167 > đứng x1 115 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 209, 115, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 97, 115, 167] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [167, 209, 236, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [313, 74, 387, 141] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 213, 107, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [160, 213, 229, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
