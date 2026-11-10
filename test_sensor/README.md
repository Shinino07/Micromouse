# test_sensor — hiệu chuẩn 5 cảm biến IR

> **Cấu hình chân hiện tại (09/11/2026, theo `Downloads/micromousev2.ioc`):** PA5/PC1/PC2 là **ADC** chứ không phải chân bật/tắt đèn phát, và **không có UART**.
> Vì vậy firmware mặc định chạy `IR_PULSED = 0`: đèn phát phải luôn sáng, `value` là ADC thô, **không khử được ánh sáng môi trường**.
> Xem kết quả bằng CubeIDE Debug → **Live Expressions**: `g_ir_cap` (5 số của lần CAP gần nhất, chép vào Excel), `g_ir_cap_count`, `g_ir_live` (8 kênh ADC thô), `g_ir_log` (nhật ký chữ).
> Nút BTN_Start = CAP vẫn dùng được. Nếu mạch thật có chân bật/tắt đèn: đổi lại các chân đó thành GPIO Output trong `.ioc` và build với `-DIR_PULSED=1`.


Công thức: **d [mm] = k · value^(−n)**, trong đó `value` = ADC lúc bật đèn phát − ADC lúc tắt đèn.

## Cấu trúc
| File | Vai trò |
|---|---|
| `test_sensor.ioc` | Bản copy của `Structure/micromouse_v2.ioc` (chỉ đổi tên project) |
| `Core/Src/ir_sensor.c`, `Core/Inc/ir_sensor.h` | Đọc 5 sensor, in qua UART, xử lý nút bấm |
| `Core/Inc/sensor_calib.h` | Hệ số k, n — do `tools/fit_kn.py` tự ghi |
| `Core/Src/main.c` | Chỉ thêm 3 dòng trong vùng `USER CODE` |
| `tools/sensor_data.xlsx` | File để điền số đo (sheet S1..S5) |
| `tools/fit_kn.py` | Tính k, n, vẽ đồ thị, ghi `sensor_calib.h` |

## 1. Nạp firmware
1. STM32CubeIDE → **File › Import… › STM32CubeIDE › STM32 CMake Project** → chọn `C:\Micromouse\test_sensor` (project dạng CMake).
2. Build rồi nạp qua ST-Link.
3. Nối USB-UART: PA9 (TX) → RX của mạch chuyển, PA10 (RX) → TX, nối chung GND. Mở terminal ở **115200 8N1** (PuTTY, Tera Term hoặc Serial Monitor).

Motor được giữ PWM = 0 trong suốt bài test.

## 2. Tìm đèn nào thuộc sensor nào (làm 1 lần)
Đặt tường cách robot khoảng 5 cm ở phía trước, hai bên hông và hướng chéo, rồi nhấn **BTN_Mode** (hoặc gõ `m`). Firmware in ra ma trận 3 nhóm đèn × 5 sensor. Ở mỗi cột, nhóm có số lớn nhất là đèn của sensor đó.
Điền kết quả vào `IR_SENSOR_GROUP[]` trong `ir_sensor.c` (0 = front, 1 = side, 2 = diag). Nếu để `IR_AUTO_GROUP` thì firmware tự chọn nhóm cho tín hiệu mạnh nhất.

## 2b. Kiểm tra thời gian chờ đèn (làm 1 lần)
Đặt tường cách khoảng 5 cm rồi gõ `s`. Firmware in bảng `value` theo thời gian bật đèn từ 10 đến 400 µs. Chọn mức nhỏ nhất mà từ đó trở đi `value` gần như không tăng nữa (lệch dưới 1% so với hàng 400 µs), rồi đặt `IR_ON_SETTLE_US` không nhỏ hơn mức đó. Mặc định đang để 120 µs.
**Firmware chạy thi phải dùng đúng thời gian chờ này**, vì k và n chỉ đúng với đúng cách đo lúc hiệu chuẩn.

## 3. Đo 20 điểm cho mỗi sensor
1. Gõ `l` để tắt in LIVE cho dễ đọc.
2. Đặt tường thẳng góc với sensor cần đo, ở khoảng cách ghi trong cột `distance_mm` (20, 30, …, 210 mm). Khoảng cách tính từ mặt mắt sensor đến tường.
3. Nhấn **BTN_Start** (hoặc gõ `c`). Firmware in một dòng dạng `CAP7  S1  S2  S3  S4  S5`. Lấy số ở cột của sensor đang đo, điền vào cột `adc` của sheet tương ứng (S1..S5).
4. Nếu có dòng `# CANH BAO ... bao hoa` thì điểm đó quá gần: bỏ trống, script cũng tự loại.

Có thể đổi khoảng cách hoặc thêm dòng tùy ý (ví dụ đo lặp 2 lần cùng một khoảng cách). Script cần tối thiểu 3 điểm cho mỗi sensor.

## 4. Tính k, n
```
cd C:\Micromouse\test_sensor\tools
python fit_kn.py
```
Kết quả:
- Bảng k, n, R², RMSE và sai số max cho từng sensor (lưu thêm ra `output/ket_qua_kn.csv`).
- `output/S1.png` … `S5.png`: đồ thị log-log kèm **đường trung bình** (hồi quy) và đường cong ở thang thường; `output/tong_hop.png` gom cả 5 sensor.
- Tự ghi `Core/Inc/sensor_calib.h`. Build và nạp lại thì các dòng LIVE và CAP sẽ in thêm `|mm`, dùng để kiểm tra lại bằng thước.

Tùy chọn: `--dmin 30 --dmax 180` để chỉ dùng điểm trong khoảng đó; `--no-header` để không ghi file `.h`.
Nếu R² < 0.98 thì nên đo lại, thường do ghi nhầm số hoặc đặt tường bị nghiêng. Script cũng in ra điểm nào lệch đường fit quá 8% để cậu đo lại riêng điểm đó. Sai số của từng điểm được lưu trong `output/sai_so_tung_diem.csv`.

## Để đo chính xác nhất
- Đo trên **tấm tường thật của mê cung** (mặt trắng), không dùng giấy hay bìa khác màu.
- Tường phải **vuông góc** với trục của sensor. Dùng thước kẹp hoặc khối kê cố định để giữ khoảng cách.
- Pin phải đầy, đúng loại sẽ dùng khi thi. Nếu đèn phát lấy điện thẳng từ pin, độ sáng sẽ đổi theo điện áp pin và làm lệch k.
- Đo ở chỗ ánh sáng ổn định, tránh nắng chiếu trực tiếp (có dòng cảnh báo `anh sang moi truong`).
- Khoảng cách được tính từ **mặt mắt sensor**. Khi dùng trong code chạy thi, nhớ cộng thêm độ lệch từ mắt sensor tới tâm robot.

## Ghi chú kỹ thuật
- ADC1 quét liên tục 5 kênh bằng DMA circular, mỗi vòng khoảng 16 µs. Thứ tự đo: tắt đèn → chờ 100 µs → chụp *off* → bật một nhóm → chờ 120 + 20 µs → chụp *on* → tắt. Ba nhóm đèn được bật lần lượt để không chiếu nhiễu sang nhau.
- `value = |on − off|`, nên mạch thu mắc kiểu nào cũng đo được, kể cả kiểu có ánh sáng thì ADC giảm. Trong bảng MAP, số âm nghĩa là mạch thu đang mắc kiểu đó.
- CAP lấy trung bình 64 lần đo, LIVE lấy trung bình 16 lần. Các thông số thời gian nằm ở đầu `ir_sensor.c`.
- Nếu sinh lại code từ `test_sensor.ioc`, phần code trong `USER CODE` được giữ nguyên.
