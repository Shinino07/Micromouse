/**
 * dc_test.h - Ban thu dong co + chinh PID, dieu khien qua ST-Link (tools/tune.py), khong can nut.
 *
 * Host ghi tham so vao g_p[], roi ghi g_cmd. Ngat 1 kHz chay lenh, ghi log moi 2 ms vao g_log[],
 * xong thi dat g_status va tang g_seq.
 * An toan: duty bi chan o P_DUTY_MAX, moi lenh co gioi han thoi gian P_TIME_MS,
 *          sai so bam > P_ERR_MAX trong 50 ms -> dung, bam SW2 hoac SW3 -> tat motor ngay.
 */
#ifndef DC_TEST_H
#define DC_TEST_H

#include <stdint.h>

/* Chi so g_p[] - PHAI KHOP voi danh sach PARAMS trong tools/tune.py */
enum {
  P_DUTY_L, P_DUTY_R, P_TIME_MS,             /* OPEN: duty tho (+ = kenh CH1), gioi han thoi gian (ca 2 lenh) */
  P_MSIGN_L, P_MSIGN_R, P_CPM,               /* PROFILE: dau motor, xung/mm */
  P_NSTEP, P_V0, P_V1, P_V2, P_V3,           /* tinh tien: so nac + van toc moi nac (mm/s, am = lui) */
  P_H0, P_H1, P_H2, P_H3, P_ACC,             /* quang duong giu moi nac (mm), gia toc (mm/s^2) */
  P_ROT, P_ROT_V, P_ROT_ACC,                 /* xoay tai cho: quang duong moi banh (mm, + = quay trai) */
  P_KS, P_KV, P_KA, P_KP, P_KD, P_KI,        /* he so tinh tien, don vi duty (0..1) */
  P_KV_R, P_KA_R, P_KP_R, P_KD_R, P_KI_R,    /* he so xoay (theo mm tren banh) */
  P_DUTY_MAX, P_ERR_MAX,
  P_WALL_BW, P_WALL_KMAX, P_WALL_LIM,        /* bam tuong: BW (1/s, 0 = tat), k_w toi da (rad/mm), goc toi da (rad) */
  P_REF_L, P_REF_R, P_WALL_TH,               /* khoang cach mat ben trai/phai khi robot o giua, nguong co tuong (mm) */
  P_HALF_TRACK, P_FRONT_STOP,                /* nua truc banh (mm, doi goc -> mm banh), dung khi tuong truoc < (mm) */
  P_WALL_HEAD,                               /* sua huong theo do doc tuong (1/s, 0 = tat; logic_code dung 3) */
  P_DECAY,                                   /* 1 = slow decay (1 chan luon mo, chan kia bam xung: tuyen tinh + tu phanh) */
  P_RNSTEP, P_RV0, P_RV1, P_RV2, P_RV3,      /* xoay theo nac (sau khi tinh tien xong): van toc banh (mm/s, + = trai) */
  P_RH0, P_RH1, P_RH2, P_RH3, P_RACC,        /* quang duong banh giu moi nac (mm), gia toc (mm/s^2) */
  P_LOG_DIV,                                 /* ghi log moi N ms (0 = 2) */
  P_BIAS_MS,                                 /* dung yen do bias gyro truoc khi chay (ms, 0 = khong) */
  P_GYRO_ROT, P_GSIGN,                       /* 1 = vong xoay dung goc GYRO (x dau) thay encoder; dung P_HALF_TRACK */
  P_COUNT
};

enum { DC_CMD_OPEN = 1, DC_CMD_PROFILE = 2, DC_CMD_SAVE = 3, DC_CMD_HAND = 5 };   /* HAND: noi bo, xoay tay do gyro */   /* SAVE: luu g_p + g_arm vao flash */
enum { DC_IDLE = 0, DC_RUN = 1, DC_OK = 2, DC_STOP_BTN = 3, DC_STOP_ERR = 4, DC_STOP_TIME = 5, DC_STOP_FRONT = 6,
       DC_SAVED = 7, DC_SAVE_FAIL = 8, DC_GYRO_HAND = 9, DC_STOP_GYRO = 11 };   /* 11: gyro chet -> khong quay */
/* g_arm: 1 = bam SW2/SW3 chay PROFILE; 2 = bai gyro: SW2 = xoay tay (tich phan gyro toi khi bam lai), SW3 = PROFILE */

/* mat IR (thu tu ADC = S1..S5 cua test_sensor) */
enum { IR_RS = 0, IR_RD = 1, IR_F = 2, IR_LD = 3, IR_LS = 4, IR_NUM = 5 };
#define IR_NONE_MM 999.0f

typedef struct {
  int32_t enc_l, enc_r;    /* so dem tho (chua doi dau) */
  int16_t duty_l, duty_r;  /* x10000: lenh gui ra driver (da nhan dau motor) */
  float s_ref, d_ref;      /* tham chieu tinh tien / vi sai banh (mm) */
  int16_t ir_l, ir_r;      /* mat ben trai / phai, mm x10 (9990 = khong thay) */
  int16_t ir_f, wall;      /* mat truoc mm x10; hieu chinh bam tuong (mm tren banh) x100 */
  float gyro_deg;          /* goc gyro tich phan 1 kHz tu luc het do bias (do, chua nhan dau/he so) */
} DcLog;                   /* 32 byte */

#define DC_LOG_MAX 2000u   /* 4 s */

/* Chay khong day: host ghi g_p roi g_arm = 1, rut ST-Link; moi lan bam-nha SW2 (hoac SW3) -> 1 s sau
   chay PROFILE trong g_p. Ket qua tung lan vao g_runs[], log cua lan cuoi vao g_log[] (RAM con khi con pin).
   Lenh SAVE ghi g_p + g_arm vao flash (sector 11): khoi dong lai van tu nap va tu hen gio. */
typedef struct {
  int32_t status;          /* DC_OK / DC_STOP_* */
  int32_t d_enc_l, d_enc_r;
  uint32_t ms;
} DcRun;
#define DC_RUNS_MAX 16u

extern volatile float g_p[P_COUNT];
extern volatile int32_t g_cmd, g_status, g_arm;
extern volatile uint32_t g_seq, g_log_n, g_run_n;
extern volatile int32_t g_enc_l, g_enc_r;
extern volatile float g_ir_mm[IR_NUM];
extern volatile uint32_t g_imu_who;   /* WHO_AM_I: 0x12 = ICM-20602, 0x70/0x71 = MPU-6500/9250, 0x47 = ICM-42688 */
extern volatile int16_t g_gz;         /* gyro Z tho, 16.4 LSB = 1 do/s */
extern volatile uint32_t g_imu_fix;   /* so lan phat hien chip IMU tu reset va da danh thuc lai */
extern DcLog g_log[DC_LOG_MAX];
extern DcRun g_runs[DC_RUNS_MAX];

void DC_TestInit(void);
void DC_TestLoop(void);
void DC_Tick1kHz(void);   /* goi trong SysTick_Handler */

#endif
