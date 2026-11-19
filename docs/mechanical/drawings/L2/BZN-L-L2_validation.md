# BZN-L-L2 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L2`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 38 = giá trị trên CAD | đạt | 38.000 |
| tờ 1 [front] 3.3 = giá trị trên CAD | đạt | 3.300 |
| tờ 1 [left] 76 = giá trị trên CAD | đạt | 76.000 |
| tờ 1 [top] 24 = giá trị trên CAD | đạt | 24.000 |
| tờ 1 [top] 7 = giá trị trên CAD | đạt | 7.000 |
| tờ 1 [top] 62 = giá trị trên CAD | đạt | 62.000 |
| tờ 1 [top] 7 = giá trị trên CAD | đạt | 7.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [76.  38.   3.3], ghi [3.3, 38.0, 76.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 4 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, A3, A4 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 4 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 210 < đứng y0 252; cạnh x0 150 > đứng x1 98 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 252, 98, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 134, 98, 210] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [150, 252, 226, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [315, 66, 395, 115] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 257, 74, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [124, 257, 162, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
