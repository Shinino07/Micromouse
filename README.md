# Micromouse: UET Makerthon 2026

Robot micromouse chạy mê cung 16×16: STM32F405, 5 cảm biến IR, gyro SPI, 2 encoder và 2 motor DC.
Firmware viết bằng C++ (logic) trên HAL/CubeMX, kèm các bài test riêng cho từng bộ phận.
Mọi hằng số đều **đo trên robot thật**.

## Thư mục
| Thư mục | Nội dung |
|---|---|
| [`logic_code/`](logic_code/README.md) | **Firmware thi đấu**: dò đường (loang + Dijkstra), biên dịch đường thành cua lớn và đi chéo, PID dựa trên mô hình motor, bám tường, 3 lượt tự động. Có mô phỏng trên PC (`sim/`) và công cụ đọc log/bản đồ (`tools/readlog.py`) |
| [`test_sensor/`](test_sensor/README.md) | Test và **hiệu chuẩn 5 mắt IR**: d = k·v^−n, hồi quy log-log từ dữ liệu đo (`tools/sensor_data.xlsx`, `tools/fit_kn.py`) |
| `test_encoder/` | Test 2 encoder: chiều đếm, số xung trên một vòng bánh |
| [`test_dc/`](test_dc/README.md) | **Bàn đo động cơ và PID**: chiều motor, mô hình KS/KV/KA, quét bộ PID, bám tường trong hành lang, gyro (dấu, hệ số, khoảng cách 2 bánh). Chạy được khi đã rút dây nhờ bấm nút; kết quả và hộp đen lưu trong flash (`tools/tune.py`) |
| `Structure/` | File cấu hình chân gốc `micromouse_v2.ioc` |

## Phần cứng và hằng số đã đo
| Bộ phận | Kết quả |
|---|---|
| Encoder | 271 xung/vòng bánh, **3.038 xung/mm** (bánh ~28.4 mm); bánh phải đếm ngược |
| Motor (slow decay) | KS 0.517 V, KV 0.00244 V/(mm/s), KA 0.000358 V/(mm/s²), quán tính xoay 0.74 (R² 0.998) |
| Gyro | Chip ghi ICM-20602 (bản clone, tương thích MPU-6500), lắp ngược: dấu −1, hệ số 1.0029, khoảng cách 2 bánh 84.3 mm |
| IR | 1 trước, 2 chéo 45°, 2 bên; sai số sau hiệu chuẩn 0.7–1.7 mm |
| Thân | Rộng 70 mm; trục bánh cách mép trước 65 mm, mép sau 25 mm |

## Build
Mỗi project là một **project CMake** của CubeIDE 2.x: **File › Import › STM32 CMake Project**, preset **Debug**, bấm nút búa.
Hoặc dùng dòng lệnh, với cmake, ninja và arm-none-eabi-gcc đi kèm CubeIDE:
```
cmake --preset Debug
cmake --build build/Debug
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -d build/Debug/<ten>.elf -v -rst
```
