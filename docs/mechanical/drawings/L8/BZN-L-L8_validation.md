# BZN-L-L8 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py L8`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 42 = giá trị trên CAD | đạt | 42.000 |
| tờ 1 [front] 4.3 = giá trị trên CAD | đạt | 4.300 |
| tờ 1 [left] 42 = giá trị trên CAD | đạt | 42.000 |
| tờ 1 [top] 29 = giá trị trên CAD | đạt | 29.000 |
| tờ 1 [top] 6.5 = giá trị trên CAD | đạt | 6.500 |
| tờ 1 [top] 17 = giá trị trên CAD | đạt | 17.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [42.  42.   4.3], ghi [4.3, 42.0, 42.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 2 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 2 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 209 < đứng y0 251; cạnh x0 154 > đứng x1 102 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 251, 102, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 167, 102, 209] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [154, 251, 196, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [334, 71, 376, 99] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 255, 97, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [147, 255, 189, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
