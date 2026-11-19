# BZN-B-S5 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py S5`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 [front] 21 = giá trị trên CAD | đạt | 21.000 |
| tờ 1 [front] 6 = giá trị trên CAD | đạt | 6.000 |
| tờ 1 [left] 26 = giá trị trên CAD | đạt | 26.000 |
| tờ 1 [top] 10.5 = giá trị trên CAD | đạt | 10.500 |
| tờ 1 [top] 20 = giá trị trên CAD | đạt | 20.000 |
| tờ 1 [top] 3 = giá trị trên CAD | đạt | 3.000 |
| tờ 1 [top] 20.2 = giá trị trên CAD | đạt | 20.200 |
| tờ 1 [top] 0.4 = giá trị trên CAD | đạt | 0.400 |
| tờ 1 [top] 10 = giá trị trên CAD | đạt | 10.000 |
| tờ 1 [top] 8 = giá trị trên CAD | đạt | 8.000 |
| kích thước bao = hộp bao chính xác của khối đặc | đạt | CAD [26. 21.  6.], ghi [6.0, 21.0, 26.0] |
| mỗi đặc tính trong bảng là một lỗ có thật trên khối đặc | đạt | 6 đặc tính |
| mã đặc tính không trùng | đạt | A1, A2, B1, B2, B3, B4 |
| bảng = mọi lỗ của chi tiết trong CAD | đạt | 6 |
| thẳng hàng đứng/bằng (cùng X giấy cho cùng điểm) | đạt | 0.0e+00 |
| thẳng hàng đứng/cạnh (cùng Y giấy cho cùng điểm) | đạt | 0.0e+00 |
| góc thứ nhất: hình bằng nằm dưới, hình cạnh nằm phải hình đứng, không chạm nhau | đạt | bằng y1 201 < đứng y0 243; cạnh x0 154 > đứng x1 102 |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [60, 243, 102, 255] |
| tờ 1 hình top: trong khung, không chạm khung tên | đạt | [60, 149, 102, 201] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [154, 243, 206, 255] |
| tờ 1 hình iso: trong khung, không chạm khung tên | đạt | [322, 66, 388, 113] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình top là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình A: trong khung, không chạm khung tên | đạt | [55, 247, 97, 259] |
| tờ 2 hình B: trong khung, không chạm khung tên | đạt | [147, 247, 199, 259] |
| tỉ lệ tờ 2 hình A là tỉ lệ ISO 5455 | đạt | 2:1 |
| tỉ lệ tờ 2 hình B là tỉ lệ ISO 5455 | đạt | 2:1 |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
