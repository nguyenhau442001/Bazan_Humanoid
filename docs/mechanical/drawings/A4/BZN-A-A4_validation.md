# BZN-A-A4 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py A4`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 10 = giá trị trên CAD | đạt | 10.000 |
| tờ 1 [front] 82 = giá trị trên CAD | đạt | 82.000 |
| tờ 1 [left] 24 = giá trị trên CAD | đạt | 24.000 |
| tờ 1 [left] 12 = giá trị trên CAD | đạt | 12.000 |
| tờ 1 [left] 70 = giá trị trên CAD | đạt | 70.000 |
| tờ 1 [left] 14 = giá trị trên CAD | đạt | 14.000 |
| tờ 1 [left] 5 = giá trị trên CAD | đạt | 5.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [24. 10. 82.], ghi [10.0, 24.0, 82.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 4 đặc tính |
| mã đặc tính không trùng | đạt | A1, B1, B2, C1 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 4 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 131 < đứng y0 173; cạnh x0 122 > đứng x1 70 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 173, 70, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 107, 70, 131] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [122, 173, 146, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [344, 69, 366, 142] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 218, 60, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [110, 218, 122, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 1:2 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 1:2 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
