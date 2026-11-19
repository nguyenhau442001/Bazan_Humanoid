# BZN-B-S3 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py S3`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 65 = giá trị trên CAD | đạt | 65.000 |
| tờ 1 [front] 9 = giá trị trên CAD | đạt | 9.000 |
| tờ 1 [left] 31 = giá trị trên CAD | đạt | 31.000 |
| tờ 1 [top] 58 = giá trị trên CAD | đạt | 58.000 |
| tờ 1 [top] 3.5 = giá trị trên CAD | đạt | 3.500 |
| tờ 1 [top] 23 = giá trị trên CAD | đạt | 23.000 |
| tờ 1 [top] 4 = giá trị trên CAD | đạt | 4.000 |
| tờ 1 [top] 40 = giá trị trên CAD | đạt | 40.000 |
| tờ 1 [top] 12.5 = giá trị trên CAD | đạt | 12.500 |
| tờ 1 [top] 3 = giá trị trên CAD | đạt | 3.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [31. 65.  9.], ghi [9.0, 31.0, 65.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 6 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, A3, A4, B1, B2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 6 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 204 < đứng y0 246; cạnh x0 177 > đứng x1 125 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 246, 125, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 173, 125, 204] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [177, 246, 208, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [321, 66, 389, 111] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 250, 120, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [170, 250, 201, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
