/**
 * dc_test.c - Ban thu dong co + chinh PID + bam tuong (xem dc_test.h).
 *
 * Mo hinh moi banh (giong logic_code/core/motion.cpp, o day tinh bang DUTY thay vi volt):
 *   duty = KS*sgn(v) + KV*v + KA*a
 * Bo dieu khien, tinh tien u_f va xoay u_r (trai = u_f - u_r, phai = u_f + u_r):
 *   u_f = KV*v_ref + KA*a_ref + KP*e_s + KD*e_v + KI*Int(e_s)
 *   u_r = KV_R*w_ref + KA_R*al_ref + KP_R*e_d + KD_R*e_w + KI_R*Int(e_d)
 *   + KS*sgn(van toc tham chieu cua tung banh)
 * s = (s_trai + s_phai)/2, d = (s_phai - s_trai)/2 (= goc * nua truc banh).
 * Bam tuong (giong motion.cpp): lech ngang e -> bu goc huong = clamp(k_w*e), k_w = BW/v
 *   -> robot ve giua hanh lang voi toc do hoi tu BW (1/s) khong phu thuoc van toc.
 * Banh trai: TIM3 encoder + TIM8 PWM (CH1/CH2). Banh phai: TIM2 encoder + TIM4 PWM (CH1/CH2).
 */
#include "dc_test.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#ifndef DC_HOST_TEST
#include "adc.h"
#include "spi.h"
#include "sensor_calib.h"
#endif

#define ENC_SIGN_L   (+1)     /* da do 2026-11-09: banh trai dem tang khi tien */
#define ENC_SIGN_R   (-1)     /* banh phai dem giam khi tien */
#define PWM_TOP      4200.0f  /* ARR + 1 cua TIM8 va TIM4 (20 kHz) */
#define DT           0.001f
#define SETTLE_MS    300u     /* profile xong van giu vi tri 300 ms de do vot lo */
#define ERR_FAULT_MS 50u
#define BTN_STOP_MS  5u
#define I_LIMIT      0.2f     /* khau tich phan toi da 0.2 duty */
#define ARM_DELAY_MS 1000u    /* nha nut -> 1 s sau moi chay (rut tay ra) */
#define BTN_DEB_MS   20u

volatile float g_p[P_COUNT];
volatile int32_t g_cmd, g_status, g_arm;
volatile uint32_t g_seq, g_log_n, g_run_n;
volatile int32_t g_enc_l, g_enc_r;
volatile float g_ir_mm[IR_NUM];
DcLog g_log[DC_LOG_MAX];
DcRun g_runs[DC_RUNS_MAX];
volatile uint32_t g_imu_who;
volatile int16_t g_gz;

/* ---------------------------------------------------------------- gyro (SPI2, CS = PB12)
   ICM-20602 (0x12) / ICM-2060x / MPU-65xx: cung bang thanh ghi (GYRO_ZOUT_H = 0x47); ICM-42688 (0x47): 0x29.
   Robot lap ICM-20602 o DUOI day, up nguoc -> truc Z nguoc -> GYRO_SIGN = -1 (bai test tu xac dinh). */
#define GYRO_LSB_DPS 16.4f   /* +-2000 do/s */
static uint8_t s_imu;        /* 0 khong co, 1 MPU-65xx / ICM-2060x, 2 ICM-42688 */
static float s_gbias, s_gbias_sum, s_gang;
static uint32_t s_gbias_n;
static int16_t s_gmin, s_gmax;      /* gyro trong luc do bias: max == min -> gyro chet */
static float s_chk_d0, s_chk_g0;    /* moc kiem tra encoder vs gyro khi xoay theo gyro */
static uint32_t s_chk_bad;          /* so lan 10 ms lien tiep gyro dung im trong khi banh quay */
volatile uint32_t g_imu_fix;
#ifndef DC_HOST_TEST
static void imu_cs(int on) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

static void imu_write(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), val};
  imu_cs(1);
  HAL_SPI_Transmit(&hspi2, tx, 2, 5);
  imu_cs(0);
}

static int imu_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
  uint8_t tx[4] = {(uint8_t)(reg | 0x80u), 0, 0, 0}, rx[4] = {0};
  imu_cs(1);
  const int ok = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(n + 1u), 2) == HAL_OK;
  imu_cs(0);
  for (uint16_t i = 0; i < n; i++) buf[i] = rx[i + 1];
  return ok;
}

static void imu_init(void)
{
  /* cau hinh o SPI cham (~650 kHz: ghi thanh ghi toi da 1 MHz), doc o 5.25 MHz (toi da 10 MHz) */
  const uint32_t pres = hspi2.Init.BaudRatePrescaler;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  HAL_SPI_Init(&hspi2);
  imu_cs(0);
  HAL_Delay(150);   /* ICM-206xx: toi 100 ms sau khi cap nguon moi doc/ghi thanh ghi duoc */
  uint8_t who = 0;
  for (int k = 0; k < 5; k++) {   /* chip clone doi luc tra 0xFF: doc lai vai lan */
    imu_read(0x75, &who, 1);
    if (who != 0xFF && who != 0x00) break;
    HAL_Delay(20);
  }
  const int mpu65 = who == 0x70 || who == 0x71 || who == 0x73 || who == 0x74 || who == 0x68;
  const int icm206 = who == 0x12 || who == 0x11 || who == 0xAC || who == 0xAF || who == 0x98;
  /* chip la / WHO_AM_I khong on dinh (chip tren robot: luc 0x2E luc 0xFF, cac thanh ghi khac van dung)
     -> van khoi tao kieu MPU-6500, roi kiem chung bang gia toc (|a| ~ 1 g) o duoi */
  const int unknown = !mpu65 && !icm206 && who != 0x47;
  if (mpu65 || icm206 || unknown) {
    imu_write(0x6B, 0x80); HAL_Delay(100);   /* reset */
    imu_write(0x6B, 0x01); HAL_Delay(10);    /* dong ho PLL */
    if (mpu65) imu_write(0x6A, 0x10);        /* MPU-65xx: tat I2C */
    else if (icm206) imu_write(0x70, 0x40);  /* ICM-2060x: I2C_IF_DIS */
    imu_write(0x19, 0x00);                   /* 1 kHz */
    imu_write(0x1A, 0x01);                   /* DLPF ~176 Hz */
    imu_write(0x1B, 0x18);                   /* +-2000 do/s */
    HAL_Delay(50);
    uint8_t kind = unknown ? 3 : 1;   /* gan s_imu o cuoi: ngat khong doc SPI giua chung */
    if (unknown) {   /* kiem chung: doc gia toc, phai ~1 g (16384 LSB o +-2 g) */
      uint8_t a[6] = {0};
      uint8_t tx[7] = {0x3B | 0x80}, rx[7] = {0};
      imu_cs(1);
      HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, 5);
      imu_cs(0);
      for (int i = 0; i < 6; i++) a[i] = rx[i + 1];
      const float ax = (int16_t)((a[0] << 8) | a[1]), ay = (int16_t)((a[2] << 8) | a[3]), az = (int16_t)((a[4] << 8) | a[5]);
      const float g = sqrtf(ax * ax + ay * ay + az * az) / 16384.0f;
      if (g < 0.7f || g > 1.3f) kind = 0;   /* khong phai IMU kieu MPU -> khong dung */
    }
    s_imu = kind;
  } else if (who == 0x47) {
    imu_write(0x11, 0x01); HAL_Delay(5);
    imu_write(0x4E, 0x0F); HAL_Delay(1);     /* gyro + accel low noise */
    imu_write(0x4F, 0x06);                   /* +-2000 do/s, ODR 1 kHz */
    HAL_Delay(50);
    s_imu = 2;
  }
  g_imu_who = who;
  hspi2.Init.BaudRatePrescaler = pres;
  HAL_SPI_Init(&hspi2);
}

/* do chip: 2 kieu SPI x 2 toc do x 4 thanh ghi x 3 byte = 48 byte vao g_probe (chay o vong lap chinh) */
volatile uint8_t g_probe[64];   /* 48 byte do chip + 14 byte tu 0x3B (ax ay az temp gx gy gz) */
static void imu_probe(void)
{
  const uint32_t pol[2] = {SPI_POLARITY_LOW, SPI_POLARITY_HIGH}, ph[2] = {SPI_PHASE_1EDGE, SPI_PHASE_2EDGE};
  const uint32_t pres[2] = {SPI_BAUDRATEPRESCALER_64, SPI_BAUDRATEPRESCALER_256};
  const uint8_t regs[4] = {0x75, 0x00, 0x0F, 0x7F};
  const SPI_InitTypeDef keep = hspi2.Init;
  const uint8_t imu_keep = s_imu;
  s_imu = 0;   /* ngat khong dung SPI trong luc do */
  int k = 0;
  for (int m = 0; m < 2; m++) {
    for (int f = 0; f < 2; f++) {
      hspi2.Init.CLKPolarity = pol[m];
      hspi2.Init.CLKPhase = ph[m];
      hspi2.Init.BaudRatePrescaler = pres[f];
      HAL_SPI_Init(&hspi2);
      imu_cs(0);
      HAL_Delay(2);
      for (int r = 0; r < 4; r++) {
        uint8_t tx[4] = {(uint8_t)(regs[r] | 0x80u), 0, 0, 0}, rx[4] = {0};
        imu_cs(1);
        HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 5);
        imu_cs(0);
        g_probe[k++] = rx[1];
        g_probe[k++] = rx[2];
        g_probe[k++] = rx[3];
        HAL_Delay(1);
      }
    }
  }
  /* 14 byte du lieu tu 0x3B o SPI cham, mode 0 */
  hspi2.Init = keep;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  HAL_SPI_Init(&hspi2);
  {
    uint8_t tx[15] = {0x3B | 0x80}, rx[15] = {0};
    imu_cs(1);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 15, 5);
    imu_cs(0);
    for (int i = 0; i < 14; i++) g_probe[48 + i] = rx[i + 1];
  }
  hspi2.Init = keep;
  HAL_SPI_Init(&hspi2);
  s_imu = imu_keep;
}

/* go loi: op 1 = doc reg, 2 = ghi val roi doc lai; ket qua o g_dbg[3] */
volatile uint32_t g_dbg[4];
static void imu_dbg(void)
{
  const SPI_InitTypeDef keep = hspi2.Init;
  const uint8_t imu_keep = s_imu;
  s_imu = 0;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  HAL_SPI_Init(&hspi2);
  imu_cs(0);
  HAL_Delay(1);
  if (g_dbg[0] == 2) {
    imu_write((uint8_t)g_dbg[1], (uint8_t)g_dbg[2]);
    HAL_Delay(2);
  }
  uint8_t v = 0;
  imu_read((uint8_t)g_dbg[1], &v, 1);
  g_dbg[3] = v;
  hspi2.Init = keep;
  HAL_SPI_Init(&hspi2);
  s_imu = imu_keep;
}

/* chip IMU tu reset (nguon chap chon): PWR_MGMT_1 khong con 0x01 -> danh thuc + cau hinh lai (khong reset, nhanh) */
static void imu_watchdog(void)
{
  static uint32_t t_last;
  if (s_imu != 1 && s_imu != 3) return;
  if (HAL_GetTick() - t_last < 200u) return;
  t_last = HAL_GetTick();
  const SPI_InitTypeDef keep = hspi2.Init;
  const uint8_t imu_keep = s_imu;
  s_imu = 0;   /* ngat tam khong doc SPI */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  HAL_SPI_Init(&hspi2);
  uint8_t pm = 0, gc = 0;
  imu_read(0x6B, &pm, 1);
  imu_read(0x1B, &gc, 1);
  if (pm != 0x01 || gc != 0x18) {
    imu_write(0x6B, 0x01);
    imu_write(0x19, 0x00);
    imu_write(0x1A, 0x01);
    imu_write(0x1B, 0x18);
    g_imu_fix++;
  }
  hspi2.Init = keep;
  HAL_SPI_Init(&hspi2);
  s_imu = imu_keep;
}

static int16_t imu_gz(void)
{
  uint8_t b[2] = {0, 0};
  if (s_imu == 1 || s_imu == 3) imu_read(0x47, b, 2);
  else if (s_imu == 2) imu_read(0x29, b, 2);
  else return 0;
  return (int16_t)((b[0] << 8) | b[1]);
}
#else
int16_t host_gyro_raw(void);   /* mo phong tren may tinh */
static void imu_probe(void) { }
static void imu_dbg(void) { }
static void imu_watchdog(void) { }
static void imu_init(void) { s_imu = 1; g_imu_who = 0x12; }
static int16_t imu_gz(void) { return host_gyro_raw(); }
#endif

static volatile uint8_t s_ready;
static volatile uint8_t s_save_req;   /* ngat yeu cau, vong lap chinh ghi flash */
static volatile uint8_t s_probe_req;  /* lenh 6: do chip IMU o vong lap chinh */
static volatile uint8_t s_logsave_req;
static int rec_load(void);
static void runlog_load(void);
/* hop den: su kien -> hang doi RAM (ngat goi duoc) -> vong lap chinh ghi vao flash sector 9 */
enum { EV_BOOT = 1, EV_START = 2, EV_END = 3, EV_ARM = 4, EV_ALIVE = 5, EV_BIAS = 6 };
static void ev_push(uint32_t type, uint32_t data);
static float p[P_COUNT];           /* ban sao tham so luc bat dau lenh */
static int s_mode;                 /* 0 nghi, DC_CMD_OPEN, DC_CMD_PROFILE */
static uint32_t s_t, s_done_t, s_err_ms, s_btn_ms, s_rel_ms, s_arm_t;
static int s_last_btn;
/* beep: 2 motor dao chieu moi 1 ms (500 Hz, duty nho) -> keu ma robot khong di; n tieng 120 ms cach 120 ms */
#define BEEP_DUTY 0.18f
static void motors(float l, float r);
static int s_decay;   /* khai bao truoc (dinh nghia o duoi, cung la tentative definition) */
static uint32_t s_beep_n, s_beep_t;
static void beep(uint32_t n) { s_beep_n = n; s_beep_t = 0; }
static int beep_tick(void)   /* 1 = dang keu (da dat motor) */
{
  if (s_beep_n == 0) return 0;
  const uint32_t ph = s_beep_t % 240u;
  s_beep_t++;
  if (s_beep_t % 240u == 0) s_beep_n--;
  if (ph >= 120u) return 0;
  const float d = (s_beep_t & 1u) ? BEEP_DUTY : -BEEP_DUTY;
  const int keep = s_decay;
  s_decay = 0;   /* giong tieng beep luc cho (fast decay) */
  motors(d, d);
  s_decay = keep;
  return 1;
}
static int s_arm_pressed, s_from_btn;
static int32_t s_l0, s_r0;
static uint16_t s_last_l;
static uint32_t s_last_r;
static float s_ref, v_ref, a_ref, d_ref, w_ref, al_ref;
static int s_step, s_trans_done, s_rot_done, s_front, s_rstep;
static float s_rheld;
static uint32_t s_log_div;
#define FRONT_DECEL 4000.0f   /* mm/s^2 khi phanh vi tuong truoc */
static float s_held, s_vm, s_wm, s_prev, d_prev, i_s, i_d, s_wall;
#define WH_STEP 5.0f
#define WH_WIN  8           /* 8 x 5 mm = 40 mm */
#define WH_N    16
static float s_wh_s[WH_N], s_wh_e[WH_N];
static int s_wh_n;
static float s_d_off;
#define WALL_FILT_MM 10.0f
static float s_irf[2];                                   /* 0 = trai (S5), 1 = phai (S1) */
static const float IR_SAT_MM[2] = {41.0f, 31.0f};        /* gan hon -> mat bao hoa (S5 ~40 mm, S1 ~30 mm) */

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float sgnf(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

/* ---------------------------------------------------------------- IR */
#ifndef DC_HOST_TEST
/* moi tick do 1 nhom den: tat ~2 ms nen chup "off" ngay, bat den, cho 150 us, chup "on" */
#define IR_ON_US     150u
#define IR_MIN_VALUE 6.0f
static GPIO_TypeDef *const EMIT_PORT[3] = {IR_front_GPIO_Port, IR_side_GPIO_Port, IR_diag_GPIO_Port};
static const uint16_t EMIT_PIN[3] = {IR_front_Pin, IR_side_Pin, IR_diag_Pin};
static const uint8_t IR_GROUP[IR_NUM] = {1, 2, 0, 2, 1};   /* 0 = truoc, 1 = ben, 2 = cheo (nhu test_sensor) */
static volatile uint16_t s_adc[IR_NUM];
static uint8_t s_group;

static void delay_us(uint32_t us)
{
  const uint32_t start = DWT->CYCCNT, ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks) { }
}

static float to_mm(int i, float value)
{
  if (!IR_CALIB_VALID || value < IR_MIN_VALUE || IR_K[i] <= 0.0f) return IR_NONE_MM;
  const float d = IR_K[i] * powf(value, -IR_N[i]);
  return d > 300.0f ? IR_NONE_MM : d;
}

static void ir_tick(void)
{
  const uint8_t g = s_group;
  uint16_t off[IR_NUM], on[IR_NUM];
  for (int i = 0; i < IR_NUM; i++) off[i] = s_adc[i];
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_SET);
  delay_us(IR_ON_US);
  for (int i = 0; i < IR_NUM; i++) on[i] = s_adc[i];
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
  for (int i = 0; i < IR_NUM; i++)
    if (IR_GROUP[i] == g) g_ir_mm[i] = to_mm(i, fabsf((float)on[i] - (float)off[i]));
  s_group = (uint8_t)((g + 1) % 3);
}
#else
void host_ir_tick(volatile float *mm);   /* mo phong tren may tinh */
static void ir_tick(void) { host_ir_tick(g_ir_mm); }
#endif

/* ---------------------------------------------------------------- motor, encoder */
static int s_decay;   /* chi bat trong luc chay lenh; luc nghi luon tha troi de day tay duoc */

/* fast decay: chan tien bam xung, chan kia 0 -> luc tat thi motor THA TROI (khong phanh, duty nho quay qua nhanh).
   slow decay: chan tien luon 1, chan kia bam xung (1 - duty) -> luc tat thi 2 dau motor noi nhau = PHANH,
   toc do ti le thang voi duty va giam duty la ham duoc. Hieu (CH1 - CH2) van = duty * TOP o ca 2 kieu. */
static void motor_set(TIM_HandleTypeDef *h, float duty)
{
  const uint32_t ccr = (uint32_t)(fminf(fabsf(duty), 1.0f) * PWM_TOP);
  const uint32_t on = duty >= 0.0f ? TIM_CHANNEL_1 : TIM_CHANNEL_2, other = duty >= 0.0f ? TIM_CHANNEL_2 : TIM_CHANNEL_1;
  if (duty == 0.0f) {   /* tat han: tha troi (xoay tay / do bias / nghi), khong phanh */
    __HAL_TIM_SET_COMPARE(h, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(h, TIM_CHANNEL_2, 0);
    return;
  }
  if (s_decay) {
    __HAL_TIM_SET_COMPARE(h, on, (uint32_t)PWM_TOP);
    __HAL_TIM_SET_COMPARE(h, other, (uint32_t)PWM_TOP - ccr);
  } else {
    __HAL_TIM_SET_COMPARE(h, on, ccr);
    __HAL_TIM_SET_COMPARE(h, other, 0);
  }
}

static void motors(float l, float r)
{
  motor_set(&htim8, l);
  motor_set(&htim4, r);
}

static void update_counts(void)
{
  const uint16_t cl = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_enc_l += (int16_t)(cl - s_last_l);
  s_last_l = cl;
  const uint32_t cr = __HAL_TIM_GET_COUNTER(&htim2);
  g_enc_r += (int32_t)(cr - s_last_r);
  s_last_r = cr;
}

/* ---------------------------------------------------------------- lenh */
static void finish(int32_t status)
{
  s_decay = 0;
  motors(0.0f, 0.0f);
  s_mode = 0;
  g_status = status;
  if (s_from_btn && g_run_n < DC_RUNS_MAX) {
    DcRun *r = &g_runs[g_run_n];
    r->status = status;
    r->d_enc_l = g_enc_l - s_l0;
    r->d_enc_r = g_enc_r - s_r0;
    r->ms = s_t;
    g_run_n++;
    s_logsave_req = 1;   /* vong lap chinh ghi ket qua + log vao flash */
    beep(3);
  }
  ev_push(EV_END, (uint32_t)status | ((g_run_n & 0xFFu) << 4) | (((s_t / 10u) & 0xFFFu) << 12));
  s_from_btn = 0;
  s_arm_pressed = 0;
  s_arm_t = 0;
  g_seq++;
}

static void start_cmd(int32_t cmd)
{
  for (int i = 0; i < P_COUNT; i++) p[i] = g_p[i];
  s_l0 = g_enc_l;
  s_r0 = g_enc_r;
  s_t = s_done_t = s_err_ms = 0;
  s_ref = v_ref = a_ref = d_ref = w_ref = al_ref = 0.0f;
  s_step = 0;
  s_held = s_vm = s_wm = s_prev = d_prev = i_s = i_d = s_wall = 0.0f;
  s_wh_n = 0;
  s_d_off = 0.0f;
  s_gbias = s_gbias_sum = s_gang = 0.0f;
  s_gbias_n = 0;
  s_gmin = 32767;
  s_gmax = -32768;
  s_chk_d0 = s_chk_g0 = 0.0f;
  s_chk_bad = 0;
  s_irf[0] = s_irf[1] = IR_NONE_MM;
  s_trans_done = p[P_NSTEP] < 0.5f;
  s_front = 0;
  s_rot_done = fabsf(p[P_ROT]) < 1e-3f && p[P_RNSTEP] < 0.5f;
  s_rstep = 0;
  s_rheld = 0.0f;
  s_log_div = p[P_LOG_DIV] >= 1.0f ? (uint32_t)(p[P_LOG_DIV] + 0.5f) : 2u;
  if (cmd == DC_CMD_HAND) s_log_div = 20u;   /* xoay tay co the lau toi 40 s */
  g_log_n = 0;
  g_status = DC_RUN;
  s_decay = p[P_DECAY] > 0.5f;
  s_mode = cmd;
  ev_push(EV_START, (uint32_t)cmd | ((g_run_n & 0xFFu) << 4));
}

/* che do hen gio: bam (>= 20 ms) roi nha (>= 20 ms) -> dem 1 s -> chay. Bam lai khi dang dem -> huy. */
static int arm_wait(int btn)
{
  if (btn && s_btn_ms >= BTN_DEB_MS) { s_arm_pressed = 1; s_arm_t = 0; }
  if (s_arm_pressed && !btn && s_rel_ms >= BTN_DEB_MS) { s_arm_pressed = 0; s_arm_t = ARM_DELAY_MS; beep(1); }
  if (s_arm_t > 0 && --s_arm_t == 0) return 1;
  return 0;
}

/* tinh tien: tang/giam toc lan luot toi tung nac, giu moi nac P_H* mm, het nac thi ve 0 */
static void prof_trans(void)
{
  const int n = (int)(p[P_NSTEP] + 0.5f);
  const float vt = s_step < n ? p[P_V0 + s_step] : 0.0f;
  const float acc = s_front ? fmaxf(p[P_ACC], FRONT_DECEL) : p[P_ACC];
  float vn = v_ref;
  if (vn < vt) vn = fminf(vn + acc * DT, vt);
  else if (vn > vt) vn = fmaxf(vn - acc * DT, vt);
  a_ref = (vn - v_ref) / DT;
  v_ref = vn;
  s_ref += v_ref * DT;
  if (s_step < n) {
    if (v_ref == vt) {
      s_held += fabsf(v_ref) * DT;
      if (s_held >= p[P_H0 + s_step]) { s_step++; s_held = 0.0f; }
    }
  } else if (v_ref == 0.0f) {
    a_ref = 0.0f;
    s_trans_done = 1;
  }
}

/* xoay: hinh thang toi dung P_ROT mm tren moi banh */
/* xoay theo nac (giong prof_trans nhung cho d): dung cho bai do mo hinh 1 lan bam nut */
static void prof_rot_steps(void)
{
  const int n = (int)(p[P_RNSTEP] + 0.5f);
  const float wt = s_rstep < n ? p[P_RV0 + s_rstep] : 0.0f;
  const float acc = p[P_RACC];
  float wn = w_ref;
  if (wn < wt) wn = fminf(wn + acc * DT, wt);
  else if (wn > wt) wn = fmaxf(wn - acc * DT, wt);
  al_ref = (wn - w_ref) / DT;
  w_ref = wn;
  d_ref += w_ref * DT;
  if (s_rstep < n) {
    if (w_ref == wt) {
      s_rheld += fabsf(w_ref) * DT;
      if (s_rheld >= p[P_RH0 + s_rstep]) { s_rstep++; s_rheld = 0.0f; }
    }
  } else if (w_ref == 0.0f) {
    al_ref = 0.0f;
    s_rot_done = 1;
  }
}

static void prof_rot(void)
{
  const float rem = p[P_ROT] - d_ref;
  const float acc = p[P_ROT_ACC];
  const float wt = sgnf(rem) * fminf(p[P_ROT_V], sqrtf(2.0f * acc * fabsf(rem)));
  float wn = w_ref;
  if (wn < wt) wn = fminf(wn + acc * DT, wt);
  else if (wn > wt) wn = fmaxf(wn - acc * DT, wt);
  if (fabsf(wn * DT) >= fabsf(rem)) {   /* buoc nay da toi dich */
    d_ref = p[P_ROT];
    w_ref = al_ref = 0.0f;
    s_rot_done = 1;
    return;
  }
  al_ref = (wn - w_ref) / DT;
  w_ref = wn;
  d_ref += w_ref * DT;
}

/* lech ngang (mm, + = robot dang lech PHAI so voi giua -> can sang trai); 0 neu khong co tuong */
/* mat ben da loc theo quang duong (~10 mm): tuong that co van / ranh / khe ghep lap lai vai mm lam so do
   nhay +-15 mm (do tren hanh lang 2026-11-10: chu ky 12 mm) -> loc con ~1/5, tre 10 mm (25 ms @ 400 mm/s). */

static void side_filter(float ds)
{
  const float raw[2] = {g_ir_mm[IR_LS], g_ir_mm[IR_RS]};
  const float a = fminf(1.0f, fabsf(ds) / WALL_FILT_MM + 0.002f);
  for (int k = 0; k < 2; k++) {
    if (raw[k] >= 900.0f || s_irf[k] >= 900.0f) s_irf[k] = raw[k];   /* mat / thay lai tuong: nhan ngay */
    else s_irf[k] += a * (raw[k] - s_irf[k]);
  }
}

static int wall_error(float *e)
{
  const float dl = s_irf[0], dr = s_irf[1];
  /* mat dang bao hoa thi bo, dung mat ben kia */
  const int okl = dl < p[P_WALL_TH] && g_ir_mm[IR_LS] >= IR_SAT_MM[0];
  const int okr = dr < p[P_WALL_TH] && g_ir_mm[IR_RS] >= IR_SAT_MM[1];
  if (okl && okr) *e = 0.5f * ((dl - p[P_REF_L]) - (dr - p[P_REF_R]));
  else if (okl) *e = dl - p[P_REF_L];
  else if (okr) *e = p[P_REF_R] - dr;
  else return 0;
  return 1;
}

/* Sua huong bang tuong (giong Motion::heading_from_wall): do doc cua lech ngang theo quang duong
   = -(goc that cua robot so voi hanh lang). Neu goc encoder noi khac -> keo lech hieu chinh s_d_off ve.
   Nho vay khau P bam tuong khong con lech du khi robot xuat phat cheo. */

static void heading_from_wall(float s, float d, float e)
{
  if (s_wh_n > 0 && s - s_wh_s[(s_wh_n - 1) % WH_N] < WH_STEP) return;
  s_wh_s[s_wh_n % WH_N] = s;
  s_wh_e[s_wh_n % WH_N] = e;
  s_wh_n++;
  if (s_wh_n <= WH_WIN) return;
  const int a = (s_wh_n - 1 - WH_WIN) % WH_N, b = (s_wh_n - 1) % WH_N;
  const float ds = s_wh_s[b] - s_wh_s[a];
  if (ds < 0.8f * WH_WIN * WH_STEP) return;
  const float slope = (s_wh_e[b] - s_wh_e[a]) / ds;
  const float off = (d - s_d_off - d_ref) / p[P_HALF_TRACK] + slope;   /* rad: goc encoder - goc that */
  if (fabsf(off) < 0.1f) s_d_off += p[P_WALL_HEAD] * off * WH_STEP / fmaxf(v_ref, 50.0f) * p[P_HALF_TRACK];
}

static void control(void)
{
  const float sl = ENC_SIGN_L * (float)(g_enc_l - s_l0) / p[P_CPM];
  const float sr = ENC_SIGN_R * (float)(g_enc_r - s_r0) / p[P_CPM];
  const float s = 0.5f * (sl + sr);
  /* vong xoay: encoder (mac dinh) hoac goc gyro doi ra mm tren banh (do GYRO_SCALE: xoay dung 10 vong theo gyro) */
  const float d_raw = p[P_GYRO_ROT] > 0.5f ? p[P_GSIGN] * s_gang * (3.14159265f / 180.0f) * p[P_HALF_TRACK]
                                           : 0.5f * (sr - sl);
  side_filter(s - s_prev);
  s_vm = 0.6f * s_vm + 0.4f * (s - s_prev) / DT;
  s_wm = 0.6f * s_wm + 0.4f * (d_raw - d_prev) / DT;
  s_prev = s;
  d_prev = d_raw;

  if (!s_trans_done) prof_trans();
  if (!s_rot_done) {
    if (p[P_RNSTEP] > 0.5f) {
      if (s_trans_done) prof_rot_steps();   /* tinh tien xong moi xoay */
    } else {
      prof_rot();
    }
  }

  /* PID bam tuong: P = lech ngang -> bu goc (k_w = BW/v), D = chinh goc huong (y' = v*goc, vong goc giu),
     "I" = sua huong theo do doc tuong (xoa lech du). Chi khi dang chay toi; cham lai thi giu nguyen. */
  float e_lat;
  if (p[P_WALL_BW] > 0.0f && v_ref > 50.0f) {
    if (wall_error(&e_lat)) {
      const float kw = fminf(p[P_WALL_KMAX], p[P_WALL_BW] / fmaxf(v_ref, 150.0f));
      s_wall = clampf(kw * e_lat, -p[P_WALL_LIM], p[P_WALL_LIM]) * p[P_HALF_TRACK];
      if (p[P_WALL_HEAD] > 0.0f) heading_from_wall(s, d_raw, e_lat);
    } else {
      s_wall = 0.0f;   /* mat tuong: giu huong theo encoder */
      s_wh_n = 0;
    }
  }
  const float d = d_raw - s_d_off;

  const float e_s = s_ref - s, e_d = d_ref + s_wall - d;
  if (p[P_KI] > 0.0f) i_s = clampf(i_s + e_s * DT, -I_LIMIT / p[P_KI], I_LIMIT / p[P_KI]);
  if (p[P_KI_R] > 0.0f) i_d = clampf(i_d + e_d * DT, -I_LIMIT / p[P_KI_R], I_LIMIT / p[P_KI_R]);
  const float uf = p[P_KV] * v_ref + p[P_KA] * a_ref + p[P_KP] * e_s + p[P_KD] * (v_ref - s_vm) + p[P_KI] * i_s;
  const float ur = p[P_KV_R] * w_ref + p[P_KA_R] * al_ref + p[P_KP_R] * e_d + p[P_KD_R] * (w_ref - s_wm) +
                   p[P_KI_R] * i_d;
  float dl = uf - ur, dr = uf + ur;
  const float wl = v_ref - w_ref, wr = v_ref + w_ref;
  if (fabsf(wl) > 5.0f) dl += p[P_KS] * sgnf(wl);
  if (fabsf(wr) > 5.0f) dr += p[P_KS] * sgnf(wr);
  /* bu ma sat tinh luc dung: profile xong ma con lech > 0.3 mm thi KP*lech nho hon KS -> robot ket lai vai do.
     Them 0.8*KS theo chieu can sua de thang ma sat (do tren robot: xoay 10 vong ket o +5.5 do) */
  if (s_rot_done && w_ref == 0.0f && fabsf(e_d) > 0.3f) {
    const float k = 0.8f * p[P_KS] * sgnf(e_d);
    dl -= k;
    dr += k;
  }
  if (s_trans_done && v_ref == 0.0f && fabsf(e_s) > 0.3f) {
    const float k = 0.8f * p[P_KS] * sgnf(e_s);
    dl += k;
    dr += k;
  }
  dl = clampf(dl, -p[P_DUTY_MAX], p[P_DUTY_MAX]);
  dr = clampf(dr, -p[P_DUTY_MAX], p[P_DUTY_MAX]);
  motors(p[P_MSIGN_L] * dl, p[P_MSIGN_R] * dr);

  /* sai so xoay khong tinh phan bam tuong (do la lenh co chu dich) */
  s_err_ms = (fabsf(e_s) > p[P_ERR_MAX] || fabsf(d_ref - d) > p[P_ERR_MAX] + fabsf(s_wall)) ? s_err_ms + 1 : 0;
}

static int16_t mm10(float mm) { return (int16_t)lrintf(fminf(mm, 999.0f) * 10.0f); }

void DC_Tick1kHz(void)
{
  if (!s_ready) return;
  update_counts();
  ir_tick();
  g_gz = imu_gz();

  const int b2 = HAL_GPIO_ReadPin(BTN_Start_GPIO_Port, BTN_Start_Pin) == GPIO_PIN_RESET;   /* SW2 */
  const int b3 = HAL_GPIO_ReadPin(BTN_Mode_GPIO_Port, BTN_Mode_Pin) == GPIO_PIN_RESET;     /* SW3 */
  const int btn = b2 || b3;
  s_btn_ms = btn ? s_btn_ms + 1 : 0;
  s_rel_ms = btn ? 0 : s_rel_ms + 1;
  if (s_mode == 0 && s_btn_ms >= BTN_DEB_MS) s_last_btn = b2 ? 2 : 3;   /* nut nao vua bam (chon bai gyro) */

  if (s_mode == 0) {
    s_decay = 0;
    if (!beep_tick()) motors(0.0f, 0.0f);
    const int32_t cmd = g_cmd;
    if (cmd == DC_CMD_SAVE) {
      g_cmd = 0;
      s_save_req = 1;   /* vong lap chinh ghi flash */
      return;
    }
    if (cmd == 6 || cmd == 7 || cmd == 8) {
      g_cmd = 0;
      s_probe_req = (uint8_t)(cmd - 5);   /* 1 do, 2 khoi tao lai + do, 3 doc/ghi 1 thanh ghi */
      return;
    }
    if (s_save_req || s_logsave_req) return;   /* dang ghi flash: chua nhan lenh / nut */
    if (cmd == DC_CMD_OPEN || cmd == DC_CMD_PROFILE) {
      g_cmd = 0;
      s_from_btn = 0;
      start_cmd(cmd);
    } else if ((g_arm == 1 || g_arm == 2) && arm_wait(btn)) {
      s_from_btn = 1;
      s_btn_ms = 0;
      start_cmd(g_arm == 2 && s_last_btn == 2 ? DC_CMD_HAND : DC_CMD_PROFILE);
    } else {
      return;
    }
  }

  /* gyro: dung yen P_BIAS_MS dau de do bias, sau do tich phan goc o 1 kHz */
  const uint32_t bias_ms = p[P_BIAS_MS] > 0.0f ? (uint32_t)p[P_BIAS_MS] : 0u;
  if (s_t < bias_ms) {
    s_gbias_sum += (float)g_gz;
    s_gbias_n++;
    if (g_gz < s_gmin) s_gmin = g_gz;
    if (g_gz > s_gmax) s_gmax = g_gz;
  } else {
    if (s_t == bias_ms && s_gbias_n) {
      s_gbias = s_gbias_sum / (float)s_gbias_n;
      if (s_mode == DC_CMD_HAND) beep(2);   /* bao: bat dau xoay tay duoc */
      ev_push(EV_BIAS, (uint32_t)lrintf(fabsf(s_gbias) * 10.0f) & 0xFFFFFu);
    }
    s_gang += ((float)g_gz - s_gbias) * (DT / GYRO_LSB_DPS);
  }

  if (s_mode == DC_CMD_OPEN) {
    motors(clampf(p[P_DUTY_L], -p[P_DUTY_MAX], p[P_DUTY_MAX]), clampf(p[P_DUTY_R], -p[P_DUTY_MAX], p[P_DUTY_MAX]));
  } else if (s_mode == DC_CMD_HAND || s_t < bias_ms) {
    if (!(s_mode == DC_CMD_HAND && beep_tick())) motors(0.0f, 0.0f);   /* xoay tay / do bias: motor tha troi */
  } else {
    control();
  }

  if (s_t % (s_log_div ? s_log_div : 2u) == 0 && g_log_n < DC_LOG_MAX) {
    DcLog *e = &g_log[g_log_n];
    e->enc_l = g_enc_l;
    e->enc_r = g_enc_r;
    const float l = (float)__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1) - (float)__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2);
    const float r = (float)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) - (float)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
    e->duty_l = (int16_t)lrintf(l / PWM_TOP * 10000.0f);
    e->duty_r = (int16_t)lrintf(r / PWM_TOP * 10000.0f);
    e->s_ref = s_ref;
    e->d_ref = d_ref;
    e->ir_l = mm10(g_ir_mm[IR_LS]);
    e->ir_r = mm10(g_ir_mm[IR_RS]);
    e->ir_f = mm10(g_ir_mm[IR_F]);
    e->wall = (int16_t)lrintf(clampf(s_wall * 100.0f, -32000.0f, 32000.0f));
    e->gyro_deg = s_gang;
    g_log_n++;
  }
  s_t++;
  if (s_t % 1000u == 0) ev_push(EV_ALIVE, (s_t / 1000u) & 0xFFFFu);   /* con song: giay thu may cua lan chay */

  if (s_mode == DC_CMD_HAND && s_t > bias_ms + 500u && s_btn_ms >= BTN_DEB_MS) {
    const float ang = s_gang, bias = s_gbias;
    finish(DC_GYRO_HAND);
    if (g_run_n > 0) {   /* ban ghi: goc (mili do) va bias (mili LSB) thay cho so xung */
      g_runs[g_run_n - 1].d_enc_l = (int32_t)lrintf(ang * 1000.0f);
      g_runs[g_run_n - 1].d_enc_r = (int32_t)lrintf(bias * 1000.0f);
    }
    return;
  }
  /* gyro chet: do bias xong ma gyro khong nhuc nhich (that luon co nhieu vai LSB) hoac khong co IMU -> khong chay */
  if (bias_ms > 0u && s_t == bias_ms + 1u && (s_imu == 0 || s_gmax <= s_gmin)) { finish(DC_STOP_GYRO); return; }
  /* xoay theo gyro: moi 10 ms so goc theo encoder voi goc gyro. Banh quay > 1 mm (>= 100 mm/s) ma gyro doi
     chua toi 1/4 goc encoder trong 3 lan lien tiep (30 ms) -> gyro chet giua chung -> dung ngay */
  if (s_mode == DC_CMD_PROFILE && p[P_GYRO_ROT] > 0.5f && s_t > bias_ms && s_t % 10u == 0u) {
    const float d_enc = 0.5f * (ENC_SIGN_R * (float)(g_enc_r - s_r0) - ENC_SIGN_L * (float)(g_enc_l - s_l0)) / p[P_CPM];
    const float dd = d_enc - s_chk_d0, dg = s_gang - s_chk_g0;
    const float expect_deg = dd / fmaxf(p[P_HALF_TRACK], 1.0f) * 57.29578f;
    if (fabsf(dd) > 1.0f && fabsf(dg) < 0.25f * fabsf(expect_deg)) s_chk_bad++;
    else s_chk_bad = 0;
    s_chk_d0 = d_enc;
    s_chk_g0 = s_gang;
    if (s_chk_bad >= 3u) { finish(DC_STOP_GYRO); return; }
  }
  /* bam nut = dung ngay (xoay tay thi nut la de KET THUC do, xu ly o tren) */
  if (s_mode != DC_CMD_HAND && s_btn_ms >= BTN_STOP_MS) { finish(DC_STOP_BTN); return; }
  if (s_mode == DC_CMD_PROFILE && s_err_ms > ERR_FAULT_MS) { finish(DC_STOP_ERR); return; }
  /* tuong truoc qua gan: phanh chu dong (bo cac nac con lai, giam toc FRONT_DECEL) thay vi cat motor roi troi */
  if (s_mode == DC_CMD_PROFILE && !s_front && p[P_FRONT_STOP] > 0.0f && v_ref > 0.0f &&
      g_ir_mm[IR_F] < p[P_FRONT_STOP]) {
    s_front = 1;
    s_step = (int)(p[P_NSTEP] + 0.5f);
  }
  if ((float)s_t >= p[P_TIME_MS]) { finish(s_mode == DC_CMD_OPEN ? DC_OK : DC_STOP_TIME); return; }
  if (s_mode == DC_CMD_PROFILE && s_trans_done && s_rot_done) {
    if (++s_done_t >= SETTLE_MS) finish(s_front ? DC_STOP_FRONT : DC_OK);
  }
}

void DC_TestInit(void)
{
  motors(0.0f, 0.0f);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  s_last_l = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  s_last_r = __HAL_TIM_GET_COUNTER(&htim2);
  g_enc_l = g_enc_r = 0;
  for (int i = 0; i < IR_NUM; i++) g_ir_mm[i] = IR_NONE_MM;
#ifndef DC_HOST_TEST
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  for (int g = 0; g < 3; g++) HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc, IR_NUM);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT);   /* chi doc buffer, khong can ngat */
#endif
  g_cmd = 0;
  g_arm = 0;
  imu_init();
  rec_load();   /* co lenh da luu trong flash -> tu nap va tu hen gio (chiu duoc reset) */
  runlog_load();
#ifndef DC_HOST_TEST
  ev_push(EV_BOOT, ((RCC->CSR >> 24) & 0xFFu) | ((uint32_t)g_arm << 8));   /* co reset: POR, PIN, SOFT, BOR, IWDG... */
  RCC->CSR |= RCC_CSR_RMVF;
#endif
  g_status = DC_IDLE;
  s_ready = 1;
  beep(1);   /* bip khi khoi dong: lac giac pin / cong tac ma nghe bip = robot vua mat nguon */
}

/* ---------------------------------------------------------------- luu tham so vao flash */
#define REC_ADDR  0x080E0000u      /* sector 11 (128 KB), chuong trinh chi ~20 KB nen khong dung toi */
#define REC_MAGIC 0x44435031u      /* "DCP1" */
typedef struct {
  uint32_t magic, arm, n;
  float p[P_COUNT];
  uint32_t crc;
} DcRec;

static uint32_t rec_crc(const DcRec *r)
{
  const uint32_t *w = (const uint32_t *)r;
  uint32_t c = 2166136261u;
  for (uint32_t i = 0; i < (sizeof(DcRec) - 4) / 4; i++) c = (c ^ w[i]) * 16777619u;
  return c;
}

static int rec_load(void)
{
#ifdef DC_HOST_TEST
  return 0;   /* tren may tinh khong co flash o dia chi nay */
#endif
  const DcRec *r = (const DcRec *)REC_ADDR;
  if (r->magic != REC_MAGIC || r->n != P_COUNT || r->crc != rec_crc(r)) return 0;
  for (int i = 0; i < P_COUNT; i++) g_p[i] = r->p[i];
  g_arm = (int32_t)r->arm;
  return 1;
}

/* Ket qua cac lan chay bang nut + log lan cuoi, ghi vao sector 10 sau moi lan chay
   (cam/rut ST-Link lam robot reset nen RAM khong giu duoc). Bo cuc theo tu 32 bit:
   [0] magic, [1] so lan, [2..65] g_runs, [66] so mau log, [67..] g_log. Magic ghi CUOI CUNG. */
#define RUNLOG_ADDR  0x080C0000u
#define RUNLOG_MAGIC 0x44434C31u   /* "DCL1" */
#define RUNLOG_HDR   (2u + DC_RUNS_MAX * 4u + 1u)

static void runlog_load(void)
{
#ifndef DC_HOST_TEST
  const uint32_t *w = (const uint32_t *)RUNLOG_ADDR;
  if (w[0] != RUNLOG_MAGIC || w[1] > DC_RUNS_MAX) return;
  g_run_n = w[1];
  const uint32_t *src = &w[2];
  uint32_t *dst = (uint32_t *)g_runs;
  for (uint32_t i = 0; i < DC_RUNS_MAX * 4u; i++) dst[i] = src[i];
#endif
}

#ifndef DC_HOST_TEST
static int flash_erase(uint32_t sector)
{
  FLASH_EraseInitTypeDef e = {0};
  uint32_t bad = 0;
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Sector = sector;
  e.NbSectors = 1;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  return HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
}

static int flash_words(uint32_t addr, const uint32_t *w, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, w[i]) != HAL_OK) return 0;
  return 1;
}

static int runlog_save(void)
{
  const uint32_t rn = g_run_n, ln = g_log_n, magic = RUNLOG_MAGIC;
  HAL_FLASH_Unlock();
  int ok = flash_erase(FLASH_SECTOR_10);
  ok = ok && flash_words(RUNLOG_ADDR + 4, &rn, 1);
  ok = ok && flash_words(RUNLOG_ADDR + 8, (const uint32_t *)g_runs, DC_RUNS_MAX * 4u);
  ok = ok && flash_words(RUNLOG_ADDR + 4 * (RUNLOG_HDR - 1), &ln, 1);
  ok = ok && flash_words(RUNLOG_ADDR + 4 * RUNLOG_HDR, (const uint32_t *)g_log, ln * (sizeof(DcLog) / 4));
  ok = ok && flash_words(RUNLOG_ADDR, &magic, 1);
  HAL_FLASH_Lock();
  return ok;
}

static int rec_save(void)
{
  DcRec r;
  r.magic = REC_MAGIC;
  r.arm = (uint32_t)g_arm;
  r.n = P_COUNT;
  for (int i = 0; i < P_COUNT; i++) r.p[i] = g_p[i];
  r.crc = rec_crc(&r);
  HAL_FLASH_Unlock();
  int ok = flash_erase(FLASH_SECTOR_11) && flash_words(REC_ADDR, (const uint32_t *)&r, sizeof(DcRec) / 4);
  ok = ok && flash_erase(FLASH_SECTOR_10);   /* lenh moi -> xoa ket qua cu */
  HAL_FLASH_Lock();
  g_run_n = 0;
  return ok && rec_load();
}
#else
static int rec_save(void) { return 1; }
static int runlog_save(void) { return 1; }
#endif

/* ---------------------------------------------------------------- hop den (sector 9, chi ghi them, khong xoa)
   moi su kien 2 tu: [0xE<<28 | loai<<24 | du lieu 24 bit], [thoi gian tu luc khoi dong, ms].
   Mat dien giua duong -> co START ma khong co END, roi BOOT voi co POR. */
#define EV_ADDR  0x080A0000u
#define EV_SIZE  0x20000u
#define EV_QN    16u
static volatile uint32_t s_evq[EV_QN][2];
static volatile uint32_t s_evh, s_evt;

static void ev_push(uint32_t type, uint32_t data)
{
  const uint32_t h = s_evh, nh = (h + 1u) % EV_QN;
  if (nh == s_evt) return;   /* day: bo */
  s_evq[h][0] = 0xE0000000u | ((type & 0xFu) << 24) | (data & 0xFFFFFFu);
  s_evq[h][1] = HAL_GetTick();
  s_evh = nh;
}

#ifndef DC_HOST_TEST
static void ev_flush(void)
{
  while (s_evt != s_evh) {
    uint32_t a = EV_ADDR;
    while (a < EV_ADDR + EV_SIZE && *(const volatile uint32_t *)a != 0xFFFFFFFFu) a += 8u;
    HAL_FLASH_Unlock();
    if (a >= EV_ADDR + EV_SIZE) { flash_erase(FLASH_SECTOR_9); a = EV_ADDR; }   /* day 16k su kien: lam lai */
    const uint32_t w[2] = {s_evq[s_evt][0], s_evq[s_evt][1]};
    flash_words(a, w, 2);
    HAL_FLASH_Lock();
    s_evt = (s_evt + 1u) % EV_QN;
  }
}
#else
static void ev_flush(void) { s_evt = s_evh; }
#endif

void DC_TestLoop(void)
{
  /* dieu khien nam trong ngat 1 kHz; o day chi ghi flash (lau ~1-2 s, motor da tat) */
  ev_flush();
  imu_watchdog();
  if (s_probe_req) {
    if (s_probe_req == 2) imu_init();
    if (s_probe_req == 3) imu_dbg();
    else imu_probe();
    s_probe_req = 0;
    g_status = 10;
    g_seq++;
  }
  if (s_save_req) {
    const int ok = rec_save();
    s_save_req = 0;
    g_status = ok ? DC_SAVED : DC_SAVE_FAIL;
    g_seq++;
  }
  if (s_logsave_req) {
    runlog_save();
    s_logsave_req = 0;
  }
}
