# test_dc: test động cơ và chỉnh PID

Firmware nhận lệnh qua ST-Link do `tools/tune.py` gửi, nên không cần bấm nút. **Bấm SW2 hoặc SW3 bất kỳ lúc nào để tắt motor ngay.**
Mỗi lệnh đều có giới hạn thời gian và giới hạn duty (0.6). Nếu sai số bám vượt 25–30 mm trong 50 ms, robot tự dừng.

## 1. Thuật toán PID (giống hệt `logic_code/core/motion.cpp`)

Mô hình mỗi bánh: **điện áp = KS·sgn(v) + KV·v + KA·a**.
- KS: điện áp để thắng ma sát.
- KV: điện áp để giữ tốc độ (do sức phản điện động của motor).
- KA: điện áp để tăng tốc (do khối lượng robot).

Bộ điều khiển chia làm 2 kênh, rồi trộn lại: bánh trái = u_f − u_r, bánh phải = u_f + u_r.

| Kênh | Feedforward (biết trước cần bao nhiêu điện áp) | Feedback PID (sửa sai lệch) |
|---|---|---|
| Tịnh tiến u_f | KV·v_ref + KA·a_ref + KS | KP·e_s + KD·e_v + KI·∫e_s |
| Xoay u_r | KV·w_ref + KA·ROT_INERTIA·α_ref | KP_ANG·e_θ + KD_ANG·e_ω + KI_ANG·∫e_θ |

**Feedforward** làm gần hết việc. Nếu mô hình đúng, robot bám profile mà PID gần như không phải làm gì. PID chỉ sửa phần mô hình sai, pin yếu, sàn khác.

Hệ số PID **không dò tay** mà tính từ mô hình, theo băng thông mong muốn ωn (rad/s) và hệ số tắt dần ζ = 0.8:
- KP = KA·ωn²
- KD = max(0, 2ζωn·KA − KV)
- KI = 2·KP (tịnh tiến), 4·KP (xoay)

Như vậy cả bộ PID chỉ còn **2 núm cần chỉnh** là WN_POS và WN_ANG, thay vì 6 hệ số.

## 2. Thuật toán test hệ số

| Bước | Lệnh | Robot làm gì | Ra được gì |
|---|---|---|---|
| 1 | `python tune.py dir` | **Nhấc robot lên**, mỗi bánh quay ~0.6 s ở duty 0.15 và 0.30 | MOTOR_SIGN, phát hiện dây motor trái/phải bị đổi |
| 2 | `python tune.py straight` | Tự chạy thẳng **chậm** 400 mm @ 150 mm/s, giữ 2 bánh đều nhau | Kiểm tra lại xung/mm: đo bằng thước rồi chạy `python tune.py cpm <mm>` |
| 3 | `python tune.py ident` | Chạy tới 100→200→300 mm/s rồi lùi về; xoay tại chỗ ±180° ở 2 tốc độ | Hồi quy bình phương tối thiểu ra KS, KV, KA, ROT_INERTIA |
| 4 | `python tune.py sweep` | Chạy tới-lùi và xoay qua lại, tăng dần ωn | ωn lớn nhất còn ổn định (vọt lố < 1 mm, duty không rung), lấy 80% |
| 5 | `python tune.py verify` | Chạy thử bộ cuối | Sai số bám; đồ thị ở `tools/output/` |

**Bước 3: vì sao dùng tích phân thay cho đạo hàm.** Encoder chỉ có 0.33 mm/xung, nên lấy đạo hàm 2 lần để ra gia tốc thì nhiễu rất lớn. Thay vào đó, mỗi cửa sổ 50 ms dùng dạng tích phân:
mean(duty) = KS·mean(sgn v) + KV·Δs/T + KA·Δv/T.
Hồi quy trên vài trăm cửa sổ. Thử trên dữ liệu giả lập, sai số là KS 1%, KV 0.4%, KA 7%.

**Bước 4: tiêu chí chọn.** Băng thông càng cao thì robot bám càng sát. Nhưng encoder thô làm nhiễu vận tốc bị khuếch đại, đến một lúc duty sẽ rung hoặc robot vọt lố. Thuật toán dừng tăng ở mức đó và lùi lại 20% để dự phòng khi pin yếu hoặc chạy trên sàn khác.

Cuối cùng, `tune.py` in ra các dòng để chép vào `logic_code/core/config.h`: KV_MOTOR, KA_MOTOR, KS_MOTOR, ROT_INERTIA, MOTOR_SIGN, WN_POS, WN_ANG.
Hệ số tính theo duty sẽ được nhân với VBAT_NOMINAL (7.6 V) cho khớp với port. Port hiện chưa đo pin, nên dùng đúng con số này.

## Ghi chú
- Ở đây kênh xoay dùng encoder (d = (s_phải − s_trái)/2). Code chính dùng gyro, nên sau khi test MPU cần chạy `verify` lại.
- TRACK_MM và GYRO_SCALE đo ở bài test gyro (xoay 10 vòng).
