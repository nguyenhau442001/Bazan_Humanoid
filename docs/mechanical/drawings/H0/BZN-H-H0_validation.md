# BZN-H-H0 · kiểm tra bản vẽ tự động

Sinh bởi `hardware/mechanical/drawings_parts.py H0`. Lỗi: **0**.

| Kiểm tra | Kết quả | Chi tiết |
| --- | --- | --- |
| tờ 1 hình front: trong khung, không chạm khung tên | đạt | [55, 180, 120, 257] |
| tờ 1 hình left: trong khung, không chạm khung tên | đạt | [160, 180, 239, 257] |
| tỉ lệ tờ 1 hình front là tỉ lệ ISO 5455 | đạt | 1:1 |
| tỉ lệ tờ 1 hình left là tỉ lệ ISO 5455 | đạt | 1:1 |
| tờ 1: chữ nằm trong khung | đạt | ok |
| tờ 1: chữ không chồng nhau | đạt | ok |
| tờ 2 hình xiso: trong khung, không chạm khung tên | đạt | [114, 131, 211, 216] |
| tờ 2: chữ nằm trong khung | đạt | ok |
| tờ 2: chữ không chồng nhau | đạt | ok |
| khung tên đủ trường | đạt | đủ |
| đánh số tờ liên tục | đạt |  |
| mọi mục trong danh mục có đúng một số chỉ dẫn (balloon) | đạt | 13 số chỉ dẫn, 13 mục |
| trình tự lắp nằm trên khổ giấy | đạt | đáy 168 |
| số tán M2 = số lỗ tán M2 của N2, N3, N4 trong CAD | đạt | 16 |
| số ốc M2×6 = tai servo + vỏ + N4 + N3i + trụ board trong CAD | đạt | 16 |
| cụm đầu lắp xong không chồng khối (robot.collisions ở tư thế 0) | đạt |  |
