#pragma once
#include <cstdint>

// Don vi: mm, s, rad. Hang so danh dau [DO] phai do tren robot that, [TUNE] chinh khi chay thu.

namespace mm::cfg {

// ---------------- Me cung (luat UET Makerthon 2026) ----------------
constexpr int   N         = 16;
constexpr float CELL      = 180.0f;
constexpr float HALF      = 90.0f;
constexpr float DIAG_STEP = 127.2792206f;  // CELL / sqrt(2)
constexpr float WALL_HALF = 6.0f;          // tuong / cot day 12 mm
constexpr float CORRIDOR  = 168.0f;
constexpr int   GOAL_X0 = 7, GOAL_Y0 = 7;  // 4 o (7,7) (8,7) (7,8) (8,8)

// ---------------- Thoi gian ----------------
constexpr float DT_S = 0.001f;              // vong dieu khien 1 kHz

// ---------------- Co khi / encoder ----------------
constexpr float TRACK_MM          = 84.3f;  // da do 2026-11-10: TB 5 lan xoay tai cho (82.5-87.3, banh truot khac nhau)
// Do 2026-11-09: quay tay 10 vong x3 -> trai 271.1, phai 271.0 xung/vong banh;
// day tay 600 mm (2 lan tot, lech 0.16%) -> 3.038 xung/mm (TB 2 banh), banh ~28.4 mm.
// Tam thoi dung chung 1 so cho 2 banh: khi day tay banh hay truot nen chua tach duoc trai/phai.
// [KIEM TRA LAI] khi robot tu chay thang cham (test dong co) roi do quang duong bang thuoc.
constexpr float COUNTS_PER_MM_L   = 3.038f; // xung / mm banh trai
constexpr float COUNTS_PER_MM_R   = 3.038f; // xung / mm banh phai
constexpr int   ENC_SIGN_L        = +1;     // da do: banh trai dem tang khi tien
constexpr int   ENC_SIGN_R        = -1;     // da do: banh phai dem GIAM khi tien
constexpr int   MOTOR_SIGN_L      = +1;     // da do 2026-11-10 (test_dc dir): duty duong -> banh tien
constexpr int   MOTOR_SIGN_R      = +1;     // da do 2026-11-10
constexpr float GYRO_SIGN         = -1.0f;  // da do 2026-11-10: chip gyro lap UP NGUOC duoi day xe (truc Z nguoc)
constexpr float GYRO_SCALE        = 1.0029f; // da do 2026-11-10: tu xoay 10 vong theo gyro, thua 16 do (3616 / 3605.5)

constexpr float ROBOT_HALF_WIDTH  = 35.0f;  // da do 2026-11-10: robot rong 70 mm (chi dung cho mo phong va kiem tra)
constexpr float ROBOT_FRONT       = 65.0f;  // da do 2026-11-10: truc banh -> mep truoc
constexpr float ROBOT_BACK        = 25.0f;  // da do 2026-11-10: truc banh -> mep sau
constexpr float START_OFFSET      = -(HALF - WALL_HALF - ROBOT_BACK - 2.0f); // vi tri truc banh so voi tam o xuat phat khi dat sat tuong sau

// ---------------- Dong co (feedforward) ----------------
constexpr float VBAT_NOMINAL = 7.6f;        // [DO] LiPo 2S; neu co mach do pin thi port tra ve gia tri that
// Do 2026-11-10 bang test_dc (tools/tune.py ident flash), SLOW DECAY, tren ban, R2 = 0.998 / 0.992.
// He so duty x VBAT_NOMINAL (port chua do pin nen duty = V / VBAT_NOMINAL -> nhat quan).
constexpr float KV_MOTOR     = 0.00244f;    // V / (mm/s)   dien ap de giu van toc   (duty 0.000322)
constexpr float KA_MOTOR     = 0.000358f;   // V / (mm/s^2) dien ap de tang toc      (duty 0.0000470)
constexpr float KS_MOTOR     = 0.517f;      // V            vuot ma sat tinh         (duty 0.068)
constexpr float ROT_INERTIA  = 0.74f;       // KA xoay / KA tinh tien (KV xoay = 0.91 KV)
constexpr float DUTY_LIMIT   = 0.95f;
constexpr float V_HEADROOM   = 0.88f;       // chi dung 88% dien ap pin cho profile, phan con lai de PID sua sai
constexpr float V_WHEEL_MAX  = (V_HEADROOM * VBAT_NOMINAL - KS_MOTOR) / KV_MOTOR;  // mm/s toc do banh toi da

// ---------------- PID: tinh tu mo hinh motor theo bang thong mong muon ----------------
// Vong vi tri:  KA*s'' + (KV+KD)*s' + KP*s = 0  ->  KP = KA*wn^2,  KD = 2*z*wn*KA - KV
// Vong goc:     G = 2/TRACK (vi sai dien ap -> toc do quay)
// sim 2026-11-10 voi mo hinh motor that: 10/15 -> re bi rong vao tuong (6-7/20 me cung loi), 14/30 -> 0-2/20.
// test_dc chay that 10/15 voi goc tu ENCODER; code chinh giu goc bang GYRO (it nhieu) nen chiu duoc 30.
// [TUNE] chay that ma robot rung / keu -> giam WN_ANG truoc (30 -> 22 -> 15), roi WN_POS.
constexpr float WN_POS = 14.0f;   // rad/s bang thong vong vi tri
constexpr float WN_ANG = 30.0f;   // rad/s bang thong vong goc
constexpr float ZETA   = 0.8f;
constexpr float G_ROT  = 2.0f / TRACK_MM;
constexpr float KP_POS = KA_MOTOR * WN_POS * WN_POS;
constexpr float KD_POS = (2 * ZETA * WN_POS * KA_MOTOR - KV_MOTOR) > 0 ? (2 * ZETA * WN_POS * KA_MOTOR - KV_MOTOR) : 0.0f;
constexpr float KI_POS = KP_POS * 2.0f;
constexpr float KP_ANG = KA_MOTOR * ROT_INERTIA * WN_ANG * WN_ANG / G_ROT;
constexpr float KD_ANG = (2 * ZETA * WN_ANG * KA_MOTOR * ROT_INERTIA - KV_MOTOR) > 0
                             ? (2 * ZETA * WN_ANG * KA_MOTOR * ROT_INERTIA - KV_MOTOR) / G_ROT : 0.0f;
constexpr float KI_ANG = KP_ANG * 4.0f;
constexpr float I_LIMIT_V = 1.5f;  // gioi han khau tich phan (V)
constexpr float WALL_LAT_BW = 4.0f;   // [TUNE] 1/s toc do keo ve giua hanh lang (KWALL = BW / v)
constexpr float KWALL_MAX   = 0.015f; // rad / mm
constexpr float WALL_ANG_LIMIT = 0.12f;
#ifndef MM_DIAG_POST_CTRL
#define MM_DIAG_POST_CTRL 0   // robot that: mat ben cach tam 38 mm -> di cheo qua cot chi con ~17 mm < nguong bao hoa
#endif
constexpr bool  DIAG_POST_CTRL = MM_DIAG_POST_CTRL;    // [TUNE] bam cot khi di cheo (mat ben luot qua cot hai ben)
constexpr float DIAG_POST_NOM  = 63.64f - WALL_HALF * 1.41421f - 26.0f;  // [DO] so do mat ben khi cot ngang mat, robot o giua duong cheo
constexpr float DIAG_POST_SEEN = 2.2f * DIAG_POST_NOM;  // chi tinh cuc tieu nho hon muc nay
constexpr float KWALL_DIAG     = 0.004f;  // [TUNE] rad / mm
constexpr float WALL_HEADING_GAIN = 3.0f;  // [TUNE] 1/s: dung tuong de keo goc gyro ve dung huong me cung
constexpr float KP_FRONT_DIST = 0.08f;  // [TUNE] V / mm  can khoang cach tuong truoc
constexpr float KP_FRONT_ANG  = 25.0f;  // [TUNE] V / rad can goc theo tuong truoc

// ---------------- Cam bien IR ----------------
// Robot that (2026-11-10): 1 mat truoc doc truc xe (S3), 2 mat cheo 45 do (S4 trai, S2 phai), 2 mat ben (S5 trai, S1 phai).
enum IrId : uint8_t { IR_F = 0, IR_DL, IR_DR, IR_L, IR_R, IR_COUNT };
struct IrPose { float fwd, lat, ang; };  // so voi truc banh; lat, ang duong = ben trai
constexpr IrPose IR_POSE[IR_COUNT] = {
  {58.0f,   0.0f,  0.0f},          // S3 truoc       [DO] fwd uoc luong (mep truoc cach truc 65)
  {50.0f,  22.0f,  0.7853982f},    // S4 cheo trai   [DO] fwd, lat uoc luong
  {50.0f, -22.0f, -0.7853982f},    // S2 cheo phai   [DO] fwd, lat uoc luong
  {40.0f,  37.8f,  1.5707963f},    // S5 ben trai: lat da do = 90 - 52.2 (hanh lang 180 mm, robot o giua)
  {40.0f, -38.5f, -1.5707963f},    // S1 ben phai: lat da do = 90 - 51.5; fwd [DO] uoc luong
};
// gan hon muc nay mat bao hoa (doc sai, dung im): khong dung de dieu khien (do luc hieu chuan 2026-11-09)
constexpr float IR_SAT_MM[IR_COUNT] = {35.0f, 30.0f, 30.0f, 41.0f, 31.0f};
constexpr float IR_NONE          = 999.0f;  // khong thay gi
constexpr float SIDE_NOM_L       = CORRIDOR / 2 - 37.8f;  // so do mat ben trai khi robot o giua (me cung that 46.2 mm)
constexpr float SIDE_NOM_R       = CORRIDOR / 2 - 38.5f;  // so do mat ben phai khi robot o giua (45.5 mm)
constexpr float SIDE_NOMINAL     = 0.5f * (SIDE_NOM_L + SIDE_NOM_R);
// robot sat 1 ben thi tuong ben kia cach ~ chuan + 50 mm -> nguong chuan + ~70 (o ben khong co tuong: > 230 mm).
// Nguong cu 105 lam robot "chay mu" khi xuat phat sat tuong (test_dc 2026-11-10)
constexpr float SIDE_WALL_TH     = 115.0f;
constexpr float SIDE_FILT_MM     = 10.0f;   // loc mat ben theo quang duong cho bam tuong (tuong co van/khe vai mm)
// can tuong truoc bang 2 mat cheo: truc banh o tam o -> mat tuong truoc cach truc CORRIDOR/2
constexpr float FRONT_ALIGN_X    = CORRIDOR / 2;          // [DO] chinh khi dat robot giua o nhin tuong (bu sai fwd mat cheo)
constexpr float DIAG_FRONT_NOM   = (FRONT_ALIGN_X - 50.0f) * 1.41421356f;   // so do mat cheo khi can xong (~48 mm)
constexpr float SIDE_EDGE_HYST   = 12.0f;
constexpr float FRONT_WALL_TH    = 190.0f;  // [DO] mat truoc (luc lay mau) < nguong -> co tuong o cuoi o ke tiep
constexpr float FRONT_STOP_MM    = CORRIDOR / 2 - 58.0f;  // so do mat truoc khi truc banh o tam o (~26 mm: BAO HOA -> khong dung de can)
constexpr float SIDE_SENSOR_FWD  = 40.0f;   // = IR_POSE[IR_L].fwd
constexpr float SENSE_BEFORE_EDGE = 8.0f;   // lay mau tuong o ke tiep truoc mep o bao nhieu mm
constexpr float SENSE_GAP        = 12.0f;  // lay mau mat ben them 1 lan som hon SENSE_GAP mm
constexpr float WALL_END_AXLE    = SIDE_SENSOR_FWD - WALL_HALF;  // [TUNE] truc banh truoc mep o bao nhieu khi mat ben het thay tuong/cot
constexpr float WALL_START_AXLE  = SIDE_SENSOR_FWD + WALL_HALF;  // [TUNE] truc banh truoc mep o bao nhieu khi mat ben bat dau thay cot/tuong
constexpr float EDGE_CORR_MAX    = 25.0f;
constexpr float SIDE_JUMP_MM     = 14.0f;   // nhay > muc nay trong 1 mau -> bo qua dieu khien tuong

// ---------------- An toan ----------------
constexpr float CLEAR_MIN_MM   = 12.0f;   // [TUNE] khe ho toi thieu quy dao luot nhanh; thieu -> bo cua cheo/cua lon
constexpr float FAULT_POS_MM   = 60.0f;
constexpr float FAULT_ANG_RAD  = 0.6f;
constexpr uint16_t FAULT_MS    = 150;

// ---------------- Toc do cho tung che do [TUNE] ----------------
struct RunParams {
  float v_search;     // mm/s  toc do do duong (khong doi trong luot)
  float v_max;        // mm/s  thang truc
  float v_diag;       // mm/s  thang cheo
  float accel;        // mm/s^2
  float decel;        // mm/s^2
  float a_lat;        // mm/s^2 gia toc ngang toi da khi cua
  float v_turn_cap;   // mm/s
  bool  diagonal;
  bool  big_turns;    // cua lon 90/180
};
constexpr RunParams RUN_SEARCH = {300.0f,  300.0f,  300.0f, 2000.0f, 2000.0f, 2500.0f,  300.0f, false, false};
constexpr RunParams RUN_RETURN = {300.0f,  800.0f,  600.0f, 2500.0f, 2500.0f, 3000.0f,  600.0f, false, true};
constexpr RunParams RUN_SAFE   = {300.0f, 1200.0f,  900.0f, 3500.0f, 3500.0f, 4000.0f,  900.0f, true,  true};
constexpr RunParams RUN_FAST   = {300.0f, 2000.0f, 1400.0f, 5500.0f, 5000.0f, 6000.0f, 1300.0f, true,  true};
constexpr float PIVOT_W_MAX = 8.0f;   // rad/s
constexpr float PIVOT_ALPHA = 60.0f;  // rad/s^2
constexpr float FRONT_ALIGN_S = 0.80f;  // toi da (thuong xong som hon nhieu)

// ---------------- Thuat toan ----------------
constexpr float EXPLORE_TIME_LIMIT_S = 240.0f;  // qua thoi gian thi ve nha du chua chung minh duong ngan nhat
constexpr uint16_t COST_F_FIRST = 150, COST_F_NEXT = 80, COST_TURN = 210, COST_ZIGZAG = 95,
                   COST_SAME_TURN = 330, COST_PIVOT90 = 400, COST_PIVOT180 = 700;
constexpr uint32_t RUN_REST_MS = 2600;

// ---------------- Nut bam (SW2 = BTN_Start PA11, SW3 = BTN_Mode PA12) ----------------
// Nut doi (1 lan bam dem 2 lan voi chong doi kieu khoa 30 ms) -> chi nhan khi muc on dinh du BTN_STABLE_MS
constexpr uint32_t BTN_STABLE_MS = 30;
// Reset phan mem: dang chay bam SW2 -> dung ngay, ve menu (giu ban do); giu SW2 >= BTN_RESET_HOLD_MS -> khoi dong lai chip
constexpr uint32_t BTN_RESET_HOLD_MS = 2000;
#ifndef MM_GOAL_CORNER
#define MM_GOAL_CORNER 0
#endif
#ifndef MM_START_CORNER
#define MM_START_CORNER 0
#endif
#ifndef MM_EDGE_RISE
#define MM_EDGE_RISE 1
#endif
#ifndef MM_FA_FILTER
#define MM_FA_FILTER 1
#endif
constexpr bool START_CORNER_ALIGN = MM_START_CORNER;  // ve toi o xuat phat: can tuong nam + tuong tay
constexpr bool EDGE_RISE = MM_EDGE_RISE;              // sua vi tri ca khi mat ben bat dau thay cot
constexpr bool FA_FILTER = MM_FA_FILTER;              // loc goc khi can tuong truoc
constexpr bool GOAL_CORNER_ALIGN = MM_GOAL_CORNER;  // can goc trong dich truoc khi ve
constexpr bool FAST_RETURN = false;  // true: ve nha bang duong nhanh (RUN_RETURN); false: ve kieu do duong (an toan)  // >= 2 s dung o o xuat phat de tu bat dau luot sau

}  // namespace mm::cfg
