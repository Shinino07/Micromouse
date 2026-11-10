/**
 * ir_sensor.c - Test 5 mat IR (IR_1..IR_5) + 3 kenh ADC phu (IR_diag PA5, IR_side PC1, IR_front PC2)
 *
 * ADC1 chay lien tuc 8 kenh bang DMA circular vao s_adc[] (1 vong ~27 us).
 * IR_PULSED = 1: PA5/PC1/PC2 la chan GPIO bat/tat den phat (cau hinh cu), do |on - off|.
 * IR_PULSED = 0: (cau hinh chan hien tai: PA5/PC1/PC2 la ADC) den phat luon sang,
 *                value = ADC tho, KHONG khu duoc anh sang moi truong.
 * Ket qua: qua USART1 neu project co UART, neu khong thi xem bien g_ir_* bang Live Expressions.
 * Moi "frame": voi tung nhom den (front, side, diag):
 *   tat het den -> doi -> chup ADC (off) -> bat nhom den -> doi -> chup ADC (on) -> tat
 *   value = |on - off|  (khu anh sang moi truong; dung duoc ca khi mach thu lam ADC giam)
 *
 * Dieu khien:
 *   BTN_Start (PA11) hoac phim 'c' : CAP  - do trung binh 64 frame, in 1 dong de chep vao Excel
 *   BTN_Mode  (PA12) hoac phim 'm' : MAP  - in ma tran nhom den x sensor
 *   phim 's'                       : STEP - value theo thoi gian bat den, de chon IR_ON_SETTLE_US
 *   phim 'l'                       : bat/tat in LIVE moi 200 ms
 *
 * LUU Y: firmware chay that phai dung DUNG thoi gian cho (IR_*_SETTLE_US) nhu luc hieu chuan,
 * neu khong k, n se lech.
 */
#include "ir_sensor.h"
#include "sensor_calib.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#ifdef HAL_UART_MODULE_ENABLED
#include "usart.h"
#endif
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Cau hinh ---------------------------------------------------------- */
#ifndef IR_PULSED
#define IR_PULSED 1             /* 1: PA5/PC1/PC2 bat/tat den phat (do |on - off|, khu anh sang phong) */
#endif
#define IR_ADC_CH         5u    /* rank 0..4 = IR_1..IR_5 (PA3, PA4, PB0, PB1, PC0) */
#define IR_OFF_SETTLE_US  100u  /* cho den nhom truoc tat han (phototransistor xa cham) */
#define IR_ON_SETTLE_US   120u  /* cho phototransistor len on dinh - kiem tra bang lenh 's' */
#define IR_FRESH_US       30u   /* > 1 vong ADC 8 kenh (~27 us) de moi kenh co mau moi */
#define IR_FRAMES_LIVE    16u
#define IR_FRAMES_CAP     64u
#define IR_SAT_HIGH       3850u /* ADC on >= muc nay -> mach thu bao hoa (~3.1 V) */
#define IR_SAT_LOW        150u  /* ADC on <= muc nay -> bao hoa (mach thu keo xuong) */
#define IR_AMBIENT_WARN   2000u /* anh sang moi truong qua manh (mach thu keo len) */
#define LIVE_PERIOD_MS    200u
#define BTN_DEBOUNCE_MS   30u

/* Nhom den cua tung sensor IR_1..IR_5. De IR_AUTO_GROUP cho den khi chay MAP
 * xong, roi dien 0 (front), 1 (side) hoac 2 (diag) cho dung phan cung. */
/* Do tren robot 09/11/2026: IR_1, IR_5 = side (PC1); IR_2, IR_4 = diag (PA5); IR_3 = front (PC2) */
static const uint8_t IR_SENSOR_GROUP[IR_NUM] = {1, 2, 0, 2, 1};

static const char *const GROUP_NAME[IR_GROUPS] = {"front", "side", "diag"};
static GPIO_TypeDef *const EMIT_PORT[IR_GROUPS] = {IR_front_GPIO_Port, IR_side_GPIO_Port, IR_diag_GPIO_Port};
static const uint16_t EMIT_PIN[IR_GROUPS] = {IR_front_Pin, IR_side_Pin, IR_diag_Pin};

static volatile uint16_t s_adc[IR_ADC_CH];

/* Xem bang CubeIDE Live Expressions khi khong co UART */
volatile uint16_t g_ir_live[8];           /* ADC tho moi 200 ms (5 o dau) */
volatile uint16_t g_ir_cap[IR_NUM];       /* value lan CAP gan nhat (chep vao Excel) */
volatile uint16_t g_ir_value[IR_NUM];     /* value (|on - off|) cap nhat moi 200 ms */
volatile int16_t g_ir_mm[IR_NUM];         /* khoang cach (mm) theo sensor_calib.h, -1 = chua hieu chuan */
volatile uint32_t g_ir_cap_count;
char g_ir_log[1024];                      /* nhat ky van ban (vong tron) */
static uint32_t s_log_pos;

/* ---- Tien ich ---------------------------------------------------------- */
static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks) { }
}

static void uart_printf(const char *fmt, ...)
{
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (len <= 0) return;
  if (len >= (int)sizeof buf) len = sizeof buf - 1;
#ifdef HAL_UART_MODULE_ENABLED
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
#else
  for (int i = 0; i < len; i++) { g_ir_log[s_log_pos] = buf[i]; s_log_pos = (s_log_pos + 1) % sizeof g_ir_log; }
#endif
}

static void emitters_off(void)
{
  if (!IR_PULSED) return;
  for (int g = 0; g < IR_GROUPS; g++)
    HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
}

static void snapshot(uint16_t out[IR_NUM])
{
  for (int i = 0; i < IR_NUM; i++) out[i] = s_adc[i];
}

/* Do 1 nhom den: off[] va on[] cua ca 5 kenh */
static void measure_group(uint8_t g, uint32_t on_settle_us, uint16_t off[IR_NUM], uint16_t on[IR_NUM])
{
  if (!IR_PULSED) {
    /* den luon sang: chi doc ADC tho */
    (void)g;
    (void)on_settle_us;
    memset(off, 0, sizeof(uint16_t) * IR_NUM);
    delay_us(IR_FRESH_US);
    snapshot(on);
    return;
  }
  emitters_off();
  delay_us(IR_OFF_SETTLE_US);
  snapshot(off);
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_SET);
  delay_us(on_settle_us + IR_FRESH_US);
  snapshot(on);
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
}

typedef struct {
  int32_t  delta_sum[IR_GROUPS][IR_NUM]; /* tong (on - off), co dau */
  uint32_t off_sum[IR_NUM];
  uint16_t on_max[IR_NUM];
  uint16_t on_min[IR_NUM];
} IR_Accum;

/* Cong don 'frames' lan do cho ca 3 nhom */
static void accumulate(uint16_t frames, uint32_t on_settle_us, IR_Accum *a)
{
  uint16_t off[IR_NUM], on[IR_NUM];
  memset(a, 0, sizeof *a);
  for (int i = 0; i < IR_NUM; i++) a->on_min[i] = 0xFFFF;

  for (uint16_t f = 0; f < frames; f++) {
    for (uint8_t g = 0; g < (IR_PULSED ? IR_GROUPS : 1); g++) {
      measure_group(g, on_settle_us, off, on);
      for (int i = 0; i < IR_NUM; i++) {
        a->delta_sum[g][i] += (int32_t)on[i] - (int32_t)off[i];
        if (on[i] > a->on_max[i]) a->on_max[i] = on[i];
        if (on[i] < a->on_min[i]) a->on_min[i] = on[i];
        if (g == 0) a->off_sum[i] += off[i];
      }
    }
  }
  emitters_off();
}

/* Trung binh co dau -> lam tron */
static int32_t avg_signed(int32_t sum, uint16_t frames)
{
  return (sum >= 0) ? (sum + frames / 2) / frames : -((-sum + frames / 2) / frames);
}

/* Chon nhom den cho sensor i (co dinh hoac AUTO = |tin hieu| lon nhat) */
static uint8_t pick_group(const IR_Accum *a, int i)
{
  uint8_t g = IR_SENSOR_GROUP[i];
  if (!IR_PULSED) return 0;
  if (g < IR_GROUPS) return g;
  g = 0;
  for (uint8_t k = 1; k < IR_GROUPS; k++)
    if (labs(a->delta_sum[k][i]) > labs(a->delta_sum[g][i])) g = k;
  return g;
}

/* ---- API --------------------------------------------------------------- */
void IR_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  emitters_off();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc, IR_ADC_CH);
  /* Khong can ngat DMA (moi 8 us 1 lan) vi chi doc buffer khi can */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  HAL_Delay(2);
}

void IR_ReadMatrix(int16_t delta[IR_GROUPS][IR_NUM], uint16_t frames)
{
  IR_Accum a;
  if (frames == 0) frames = 1;
  accumulate(frames, IR_ON_SETTLE_US, &a);
  for (int g = 0; g < IR_GROUPS; g++)
    for (int i = 0; i < IR_NUM; i++)
      delta[g][i] = (int16_t)avg_signed(a.delta_sum[g][i], frames);
}

static void read_with_settle(IR_Reading *r, uint16_t frames, uint32_t on_settle_us)
{
  IR_Accum a;
  if (frames == 0) frames = 1;
  accumulate(frames, on_settle_us, &a);
  for (int i = 0; i < IR_NUM; i++) {
    uint8_t g = pick_group(&a, i);
    int32_t v = avg_signed(a.delta_sum[g][i], frames);
    r->group[i]   = g;
    r->value[i]   = (uint16_t)(v < 0 ? -v : v);
    r->ambient[i] = (uint16_t)((a.off_sum[i] + frames / 2) / frames);
    r->on_max[i]  = a.on_max[i];
    r->on_min[i]  = a.on_min[i];
    r->inverted[i] = (v < 0);
  }
}

void IR_Read(IR_Reading *r, uint16_t frames)
{
  read_with_settle(r, frames, IR_ON_SETTLE_US);
}

int32_t IR_ToMm(uint8_t sensor, uint16_t value)
{
  if (!IR_CALIB_VALID || sensor >= IR_NUM || value == 0 || IR_K[sensor] <= 0.0f) return -1;
  return (int32_t)(IR_K[sensor] * powf((float)value, -IR_N[sensor]) + 0.5f);
}

/* ---- Chuong trinh test -------------------------------------------------- */
static uint8_t  s_live = 1;
static uint32_t s_cap_count;
static uint32_t s_last_live;

typedef struct { GPIO_TypeDef *port; uint16_t pin; uint8_t stable; uint32_t t; } Button;
static Button s_btn_start = {BTN_Start_GPIO_Port, BTN_Start_Pin, 1, 0};
static Button s_btn_mode  = {BTN_Mode_GPIO_Port,  BTN_Mode_Pin,  1, 0};

/* Tra ve 1 khi vua nhan (muc thap, co chong doi) */
static uint8_t button_pressed(Button *b)
{
  uint8_t level = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_SET);
  uint32_t now = HAL_GetTick();
  if (level != b->stable && (now - b->t) >= BTN_DEBOUNCE_MS) {
    b->stable = level;
    b->t = now;
    return level == 0;
  }
  return 0;
}

static int uart_getc(void)
{
#ifdef HAL_UART_MODULE_ENABLED
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart1);
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) return (int)(huart1.Instance->DR & 0xFF);
#endif
  return -1;
}

static void print_values(const char *tag, const IR_Reading *r)
{
  uart_printf("%s", tag);
  for (int i = 0; i < IR_NUM; i++) uart_printf("\t%u", r->value[i]);
  if (IR_CALIB_VALID) {
    uart_printf("\t|mm");
    for (int i = 0; i < IR_NUM; i++) {
      int32_t mm = IR_ToMm((uint8_t)i, r->value[i]);
      if (mm < 0) uart_printf("\t-"); else uart_printf("\t%ld", (long)mm);
    }
  }
  uart_printf("\r\n");
}

static void print_warnings(const IR_Reading *r)
{
  for (int i = 0; i < IR_NUM; i++) {
    if (r->value[i] < 20) continue;   /* sensor khong nhin thay gi, bo qua */
    if (!r->inverted[i] && r->on_max[i] >= IR_SAT_HIGH)
      uart_printf("# CANH BAO S%d bao hoa (ADC on max = %u): diem nay qua gan, bo khoi Excel\r\n", i + 1, r->on_max[i]);
    if (r->inverted[i] && r->on_min[i] <= IR_SAT_LOW)
      uart_printf("# CANH BAO S%d bao hoa (ADC on min = %u): diem nay qua gan, bo khoi Excel\r\n", i + 1, r->on_min[i]);
    if (IR_PULSED && !r->inverted[i] && r->ambient[i] >= IR_AMBIENT_WARN)
      uart_printf("# CANH BAO S%d anh sang moi truong manh (off = %u)\r\n", i + 1, r->ambient[i]);
  }
}

static void do_capture(void)
{
  IR_Reading r;
  IR_Read(&r, IR_FRAMES_CAP);
  char tag[16];
  snprintf(tag, sizeof tag, "CAP%lu", (unsigned long)++s_cap_count);
  print_values(tag, &r);
  for (int i = 0; i < IR_NUM; i++) g_ir_cap[i] = r.value[i];
  g_ir_cap_count = s_cap_count;
  uart_printf("#   ambient");
  for (int i = 0; i < IR_NUM; i++) uart_printf("\t%u", r.ambient[i]);
  uart_printf("\r\n#   group  ");
  for (int i = 0; i < IR_NUM; i++) uart_printf("\t%s", GROUP_NAME[r.group[i]]);
  uart_printf("\r\n");
  print_warnings(&r);
}

static void do_map(void)
{
  if (!IR_PULSED) {
    /* khong bat/tat den duoc: in ADC tho ca 8 kenh de biet kenh nao phan ung voi tuong */
    uart_printf("MAP (ADC tho)\tIR_1\tIR_2\tIR_3\tIR_4\tIR_5\tdiag\tside\tfront\r\n ");
    for (unsigned i = 0; i < IR_ADC_CH; i++) uart_printf("\t%u", s_adc[i]);
    uart_printf("\r\n=> dua tay/tuong lai gan tung mat, kenh nao doi thi la mat do\r\n");
    return;
  }
  int16_t d[IR_GROUPS][IR_NUM];
  IR_ReadMatrix(d, IR_FRAMES_CAP);
  uart_printf("MAP (on - off khi bat tung nhom den)\r\n group\tS1\tS2\tS3\tS4\tS5\r\n");
  for (int g = 0; g < IR_GROUPS; g++) {
    uart_printf(" %s", GROUP_NAME[g]);
    for (int i = 0; i < IR_NUM; i++) uart_printf("\t%d", d[g][i]);
    uart_printf("\r\n");
  }
  uart_printf("=> moi cot, nhom co |so| lon nhat la den cua sensor do (dien vao IR_SENSOR_GROUP)\r\n");
  uart_printf("   so am = mach thu lam ADC giam khi co anh sang (binh thuong, firmware lay tri tuyet doi)\r\n");
}

/* Value theo thoi gian bat den: chon IR_ON_SETTLE_US = muc nho nhat ma value
 * khong con tang (lech < 1% so voi hang cuoi) */
static void do_step(void)
{
  if (!IR_PULSED) { uart_printf("STEP chi dung khi bat/tat duoc den phat (IR_PULSED = 1)\r\n"); return; }
  static const uint16_t SETTLE[] = {10, 20, 40, 60, 80, 120, 160, 250, 400};
  uart_printf("STEP (dat tuong ~5 cm truoc cac sensor)\r\n settle_us\tS1\tS2\tS3\tS4\tS5\r\n");
  for (unsigned k = 0; k < sizeof SETTLE / sizeof SETTLE[0]; k++) {
    IR_Reading r;
    read_with_settle(&r, 32, SETTLE[k]);
    uart_printf(" %u", SETTLE[k]);
    for (int i = 0; i < IR_NUM; i++) uart_printf("\t%u", r.value[i]);
    uart_printf("\r\n");
  }
  uart_printf("=> dang dung IR_ON_SETTLE_US = %u\r\n", IR_ON_SETTLE_US);
}

static void motors_off(void)
{
  /* PWM = 0 tren ca 2 chan moi motor -> driver ngat, motor khong tu quay */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void IR_TestInit(void)
{
  motors_off();
  IR_Init();
  uart_printf("\r\n=== TEST SENSOR IR (5 cap) ===\r\n");
  uart_printf("BTN_Start/'c' = CAP | BTN_Mode/'m' = MAP | 's' = STEP | 'l' = bat/tat LIVE\r\n");
  uart_printf("Hieu chuan: %s | settle on/off = %u/%u us\r\n",
              IR_CALIB_VALID ? "CO (in them mm)" : "CHUA (chi in ADC)", IR_ON_SETTLE_US, IR_OFF_SETTLE_US);
  uart_printf("tag\tS1\tS2\tS3\tS4\tS5\r\n");
}

void IR_TestLoop(void)
{
  int c = uart_getc();
  if (c == 'l' || c == 'L') { s_live = !s_live; uart_printf("# LIVE %s\r\n", s_live ? "ON" : "OFF"); }
  if (c == 's' || c == 'S') do_step();
  if (button_pressed(&s_btn_start) || c == 'c' || c == 'C') do_capture();
  if (button_pressed(&s_btn_mode)  || c == 'm' || c == 'M') do_map();

  uint32_t now = HAL_GetTick();
  if (s_live && (now - s_last_live) >= LIVE_PERIOD_MS) {
    s_last_live = now;
    IR_Reading r;
    IR_Read(&r, IR_FRAMES_LIVE);
    for (unsigned i = 0; i < IR_ADC_CH; i++) g_ir_live[i] = s_adc[i];
    for (int i = 0; i < IR_NUM; i++) {
      g_ir_value[i] = r.value[i];
      g_ir_mm[i] = (int16_t)IR_ToMm((uint8_t)i, r.value[i]);
    }
    print_values("LIVE", &r);
  }
}
