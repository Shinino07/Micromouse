# logic_code: firmware chạy thi (STM32F405, C++)

> **Trạng thái 10/11/2026:** đã tích hợp **mọi thứ đo được trên robot thật** (encoder, motor, PID, gyro, cảm biến IR, bám tường).
> Mô phỏng với mô hình motor thật: mê cung vòng loại chạy 3 lượt **không va chạm** (97.4 / 15.2 / 12.4 s); 60 mê cung ngẫu nhiên lỗi 3/60.
> Chưa chạy firmware này trong mê cung thật. Hướng dẫn chạy lần đầu ở mục cuối.

## Phần cứng (đã kiểm chứng)
| Bộ phận | Chi tiết |
|---|---|
| MCU | STM32F405RGT6, HSI 16 → 168 MHz, cấu hình chân như `Structure/micromouse_v2.ioc` |
| Motor | Trái TIM8 (PC6 = CH1, PC7 = CH2), phải TIM4 (PB6, PB7), 20 kHz. **Băm xung SLOW DECAY**: một chân luôn mở, chân kia băm (1 − duty). Kiểu băm cũ (fast decay) làm bánh trôi, chạy vượt profile |
| Encoder | Trái TIM3, phải TIM2. **271 xung/vòng bánh, 3.038 xung/mm**. Phải đếm ngược khi đi tiến |
| Gyro | Chip ghi **ICM-20602** (bản clone), SPI2, CS = PB12, **lắp ngược dưới đáy** → `GYRO_SIGN = −1`. Mã WHO_AM_I không ổn định (0x2E / 0xFF / 0x3F), nên code khởi tạo theo bảng thanh ghi MPU-6500 rồi kiểm tra bằng gia tốc kế (≈ 1 g). Mỗi 200 ms có watchdog ghi lại cấu hình nếu chip tự reset. **Reset 3 lần trong 3 s** (nguồn hoặc dây IMU chập chờn, log 10/11) thì robot bỏ gyro, tính góc quay bằng 2 encoder cho tới khi khởi động lại. Chạy được chế độ 1, nhưng góc trôi dần nên **không dùng để thi** |
| IR | 5 cặp mắt, đèn phát bật/tắt theo 3 nhóm (PC2 trước, PC1 bên, PA5 chéo); giá trị = \|bật − tắt\|, d = k·v^−n |
| Nút | **SW2 = Start (PA11)**, **SW3 = Mode (PA12)**, đều tích cực mức thấp |

Bố trí mắt (`cfg::IrId`):

| Vai trò | Mắt | Hướng | Dùng để |
|---|---|---|---|
| `IR_F` | S3 | thẳng trước | phát hiện tường cuối ô kế tiếp (bão hoà khi < 35 mm, nên **không** dùng để canh tường) |
| `IR_DL` / `IR_DR` | S4 / S2 | chéo 45° | **canh tường trước** (tính góc và khoảng cách từ 2 điểm chạm tường) |
| `IR_L` / `IR_R` | S5 / S1 | 2 bên | bám tường, bắt mép tường để sửa vị trí. Bão hoà khi < 41 / 31 mm, lúc đó bỏ qua và dùng mắt bên kia |

## Hằng số đã đo (`core/config.h`)
| Nhóm | Giá trị | Cách đo |
|---|---|---|
| Encoder | `COUNTS_PER_MM` 3.038, `ENC_SIGN` +1 / −1 | quay tay 10 vòng × 3, đẩy 600 mm × 2 |
| Motor | `MOTOR_SIGN` +1 / +1; `KS` 0.517 V, `KV` 0.00244 V/(mm/s), `KA` 0.000358 V/(mm/s²), `ROT_INERTIA` 0.74 | `test_dc`: chạy tới/lùi 4 nấc + xoay, hồi quy R² 0.998 |
| PID | `WN_POS` 14, `WN_ANG` 30 rad/s (tính tự động từ mô hình) | mô phỏng; `test_dc` chạy thật 10/15 tới 600 mm/s, bám profile lệch ≤ 2 mm |
| Bám tường | `WALL_LAT_BW` 4, `WALL_HEADING_GAIN` 3, lọc mắt bên 10 mm, ngưỡng có tường 115 mm | hành lang thật 400–600 mm/s: lệch 23–33 mm về ±2–3 mm |
| Gyro | `GYRO_SIGN` −1, `GYRO_SCALE` 1.0029, `TRACK_MM` 84.3 | tự xoay 10 vòng theo gyro, thừa 16° (5.5° do trôi lúc dừng, đã sửa bằng bù ma sát) |
| Kích thước | rộng 70 mm, trục → mép trước 65 mm, → mép sau 25 mm | thước |
| IR | k, n của 5 mắt; mắt bên cách tâm 37.8 / 38.5 mm | `test_sensor` + đặt robot giữa hành lang |

**Còn ước lượng (`[DO]`):** vị trí dọc `fwd` của mắt trước, mắt chéo và mắt bên trong `IR_POSE`, và `FRONT_ALIGN_X`. Hai giá trị này quyết định robot canh tường trước dừng đúng tâm ô và bắt mép tường đúng chỗ. Cách chỉnh nằm ở mục cuối.

## Cách robot chạy (chế độ AUTO)
1. **Lượt 1, dò đường:** loang từ ô xuất phát tới đích, đi hết 2 ô đích, dò tiếp các ô có thể nằm trên đường ngắn nhất cho tới khi chứng minh xong (hoặc quá 240 s), rồi về ô xuất phát. **Ô mới: dừng ở tâm ô, đứng yên nhìn 3 phía** rồi mới ghi bản đồ và chọn hướng (mắt trước chỉ tin trong ~14 cm, mắt bên ở mép ô có thể còn thấy đầu tường). Có tường trước thì bò tới cách tường ~38 mm, canh tường rồi xoay. Ô đã biết hết tường: chạy thẳng qua (có canh gác tường bất ngờ); rẽ thì cũng dừng-nhìn-xoay. Mô phỏng 16/11 (mắt giống robot thật): vòng loại tới đích lượt 1 sau ~187 s, cả buổi 3 lượt xong ~420 s; 20 mê cung ngẫu nhiên không lỗi.
2. **Lượt 2, nhanh an toàn** (`RUN_SAFE`), về nhà, nghỉ ≥ 2.6 s rồi tự chạy tiếp.
3. **Lượt 3, nhanh nhất** (`RUN_FAST`), về nhà.

Bản đồ được lưu vào flash sau mỗi lượt. Đường nhanh tìm bằng Dijkstra theo thời gian, rồi biên dịch thành cua lớn 90°/180°, vào và ra chéo 45°/135°, V90 và đoạn chéo. Trước khi chạy, robot quét khe hở toàn quỹ đạo theo kích thước thân thật. Thiếu khe hở thì bỏ chéo, rồi bỏ cua lớn.

**Chế độ 1 (FOLLOW, để test sân):** đi thẳng. Ở mép ô, nếu mắt trước *nghi* có tường thì robot dừng ở tâm ô, **đứng yên đọc lại** 60 ms rồi mới quyết định: tường thật cách mắt khoảng 26 mm, không có tường thì > 200 mm. Rẽ về phía trống (ưu tiên trái), ngõ cụt thì quay đầu, rẽ bằng xoay tại chỗ. Đang đi thẳng qua ô mà mắt trước mới thấy tường ở cuối ô (< 110 mm) thì robot phanh dừng ở tâm ô rồi nhìn lại. Mỗi ô ghi 1 dòng log `FOLLOW (x,y) T..[gần-xa %] L.. R..` và mỗi lần dừng ghi `dung nhin (x,y): T.. L.. R.. cheo .. ..`.

## Nút bấm
- **SW3 (Mode):** bấm nhanh để đổi chế độ. Robot **kêu N bíp**, tức chế độ N−1: 0 **AUTO (mặc định khi bật, bản thi)**, 1 đi thẳng + rẽ (test sân), 2 dò đường, 3 nhanh an toàn, 4 nhanh nhất, 5 test chạy thẳng 5 ô, 6 test xoay 10 vòng, 7 in IR, 8 xoá bản đồ. Muốn khoá ở chế độ 1 khi test sân: `LOCK_FOLLOW = true` (`core/config.h`) và `mode_ = Mode::FOLLOW` (`core/mouse.h`).
- **Giữ SW3 khoảng 1 s (nghe 1 bíp) rồi nhả:** **bắt đầu** chế độ đang chọn. Robot chờ 1 s, đo bias gyro 0.6 s rồi chạy.
- **Đang chạy, bấm SW3 (hoặc SW2):** dừng ngay, lưu bản đồ, về menu.
- **SW2 (Start):** bấm rồi nhả là bắt đầu chế độ đang chọn; giữ ≥ 2 s là khởi động lại chip. Đã kiểm tra lại 16/11 bằng cách đọc thẳng chân PA11/PA12 qua SWD: cả 2 nút bấm/nhả sạch. Lần mất nguồn hôm 10/11 là do pin yếu và giắc pin lỏng, không phải do nút. SW3 giữ ~1 s vẫn bắt đầu được như SW2.
- **FAULT** (lệch > 60 mm hoặc 0.6 rad trong 150 ms): tắt motor, lưu bản đồ, click 5 lần.

## Đưa lên robot
1. CubeIDE → **File › Import… › STM32 CMake Project** → `C:\Micromouse\logic_code\micromouse_fw`, preset **Debug**, bấm nút **búa**.
2. Hoặc nạp bằng dòng lệnh (dây SWD nối dài nên dùng 100 kHz):
   ```
   STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -d micromouse_fw/build/Debug/micromouse_fw.elf -v -rst
   ```
3. **Log được ghi vào flash** (sector 10). Mỗi dòng có thời điểm tính từ lúc khởi động, kèm dòng `=== KHOI DONG` ghi lý do reset. Cắm hay rút ST-Link làm robot reset nhưng log vẫn còn. Chạy xong, cắm ST-Link rồi chạy:
   ```
   python tools/readlog.py        # log tu lan khoi dong co chay gan nhat + ve ban do robot da do
   python tools/readlog.py all    # toan bo log
   python tools/readlog.py map    # chi ban do
   ```
   Bản đồ được vẽ từ flash sector 11, để so với map thật xem robot đọc tường có đúng không.

## Chạy lần đầu trong mê cung thật (theo thứ tự)
1. **Chế độ 6 (in IR):** đặt robot giữa ô, nhìn thẳng tường trước. Đọc 2 mắt chéo: lý thuyết khoảng **48 mm** mỗi mắt. Lệch nhiều thì sửa `FRONT_ALIGN_X` cho đến khi robot canh tường xong mà trục bánh nằm đúng tâm ô.
2. **Chế độ 4 (thẳng 5 ô):** kiểm tra dừng đúng 900 mm. Robot rung hoặc kêu thì giảm `WN_POS`.
3. **Chế độ 5 (xoay 10 vòng):** sau 10 vòng lệch dưới 2° là đạt. Rung thì giảm `WN_ANG` (30 → 22 → 15).
4. **Chế độ 1 (dò đường)** ở tốc độ tìm kiếm, xem robot có đọc đúng tường không, rồi mới chạy **AUTO**.
5. Nếu robot sửa vị trí ở mép tường bị sớm hoặc muộn, chỉnh `IR_POSE[IR_L/IR_R].fwd` (vị trí dọc của mắt bên).

## Công cụ đo và hiệu chỉnh: `test_dc`
Firmware riêng để đo motor, PID, gyro và bám tường mà không cần cắm dây khi chạy (`test_dc/tools/tune.py`). Có hộp đen lưu vào flash và tiếng bíp báo trạng thái. Xem `test_dc/README.md`.

## Mô phỏng
```
mkdir -p build
g++ -std=c++17 -O2 -o build/sim.exe core/*.cpp sim/sim_main.cpp
build/sim.exe sim/mazes/vongloai.txt          # 3 luot tren map vong loai
build/sim.exe --random 20 7                   # 20 me cung ngau nhien + 300 kiem tra lap duong
build/sim.exe --clear sim/mazes/vongloai.txt  # khe ho tung chuyen dong
SIM_MODE=1 build/sim.exe sim/mazes/chu_n.txt # che do 1 trong me cung chu n (giong san test)
SIM_FSPIKE=0.1 ...                             # mat truoc doc nham 10% so mau (thu chong nhieu)
SIM_NOGYRO=0 ...                               # IMU hong: tinh goc bang encoder tu dau (hoac sau N ms)
```
Mô phỏng dùng đúng `config.h`: mô hình motor thật, gyro lắp ngược, encoder 3.038 xung/mm, mắt IR bão hoà khi quá gần. Sửa `config.h` xong nên chạy lại mô phỏng trước khi nạp.
