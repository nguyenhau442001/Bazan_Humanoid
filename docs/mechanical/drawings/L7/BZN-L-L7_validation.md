# BZN-L-L7 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L7`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 42 = giá trị trên CAD | đạt | 42.000 |
| tờ 1 [front] 44.58 = giá trị trên CAD | đạt | 44.577 |
| tờ 1 [left] 58.5 = giá trị trên CAD | đạt | 58.500 |
| tờ 1 [front] 21 = giá trị trên CAD | đạt | 21.000 |
| tờ 1 [front] 14.5 = giá trị trên CAD | đạt | 14.500 |
| tờ 1 [top] 21 = giá trị trên CAD | đạt | 21.000 |
| tờ 1 [top] 27.35 = giá trị trên CAD | đạt | 27.350 |
| tờ 1 [front] 18 = giá trị trên CAD | đạt | 18.000 |
| tờ 1 [front] 5.5 = giá trị trên CAD | đạt | 5.500 |
| tờ 1 [top] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [top] 14 = giá trị trên CAD | đạt | 14.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [58.5  42.   44.58], ghi [42.0, 44.58, 58.5] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 9 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, C1, D1, D2, E1, E2, F1, G1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 9 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 168 < đứng y0 210; cạnh x0 154 > đứng x1 102 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 210, 102, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 110, 102, 168] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [154, 210, 212, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [328, 73, 386, 139] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 221, 97, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [147, 214, 206, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
