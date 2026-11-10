// Lop phan cung STM32F405 cho core/: dung ten chan (label) trong micromouse_v2.ioc.
// Moi thong so phu thuoc mach that danh dau [DO].
#include "app.h"

#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#ifdef HAL_UART_MODULE_ENABLED
#include "usart.h"
#endif

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "../core/hw.h"
#include "../core/mouse.h"
#include "sensor_calib.h"

using namespace mm;

namespace {

// ---------------- IR ----------------
// ADC quet 8 kenh: rank 0..4 = IR_1..IR_5 (PA3, PA4, PB0, PB1, PC0), 5 = IR_diag PA5, 6 = IR_side PC1, 7 = IR_front PC2.
// He so k, n (sensor_calib.h) theo IR_1..IR_5 = S1..S5 cua test_sensor.
// IR_PULSED = 0 (cau hinh chan hien tai): PA5/PC1/PC2 la ADC nen khong bat/tat den phat duoc,
// den phai luon sang, value = ADC tho (giong test_sensor khi IR_PULSED = 0).
#ifndef IR_PULSED
#define IR_PULSED 1
#endif
constexpr int ADC_CH = 5;
// vai tro (cfg::IrId: truoc, cheo T, cheo P, ben T, ben P) -> ADC rank (S1..S5 = IR_1..IR_5) va nhom den phat
// S1 = ben phai, S2 = cheo phai, S3 = truoc, S4 = cheo trai, S5 = ben trai (da do 2026-11-09)
constexpr uint8_t ROLE_ADC[cfg::IR_COUNT]   = {2, 3, 1, 4, 0};
constexpr uint8_t ROLE_GROUP[cfg::IR_COUNT] = {0, 2, 2, 1, 1};  // 0 = den truoc (PC2), 1 = den ben (PC1), 2 = den cheo (PA5)
constexpr uint32_t IR_ON_US = 150;  // chi dung khi IR_PULSED = 1 (= IR_ON_SETTLE_US + IR_FRESH_US cua test_sensor)
constexpr float IR_MIN_VALUE = 6.0f;
static_assert(sizeof(IR_K) / sizeof(IR_K[0]) == 5, "sensor_calib.h phai co 5 he so");
constexpr bool role_adc_ok() {
  for (uint8_t a : ROLE_ADC) if (a >= 5) return false;
  return true;
}
static_assert(role_adc_ok(), "ROLE_ADC chi duoc tro toi IR_1..IR_5 (co he so k, n)");

#if IR_PULSED
GPIO_TypeDef* const EMIT_PORT[3] = {IR_front_GPIO_Port, IR_side_GPIO_Port, IR_diag_GPIO_Port};
const uint16_t EMIT_PIN[3] = {IR_front_Pin, IR_side_Pin, IR_diag_Pin};
#endif

volatile uint16_t g_adc[ADC_CH];
float g_ir[cfg::IR_COUNT];
#if IR_PULSED
uint8_t g_group = 0;

void delay_us(uint32_t us) {
  const uint32_t start = DWT->CYCCNT, ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks) {}
}
#endif

float to_mm(uint8_t adc_idx, float value) {
  if (!IR_CALIB_VALID || value < IR_MIN_VALUE || IR_K[adc_idx] <= 0.0f) return cfg::IR_NONE;
  const float d = IR_K[adc_idx] * powf(value, -IR_N[adc_idx]);
  return d > 300.0f ? cfg::IR_NONE : d;
}

void emitters_off() {
#if IR_PULSED
  for (int g = 0; g < 3; g++) HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
#endif
}

void ir_tick() {
#if IR_PULSED
  // moi tick do 1 nhom den: den da tat ~1 ms nen chup "off" ngay, bat den, cho, chup "on"
  const uint8_t g = g_group;
  uint16_t off[5], on[5];
  for (int i = 0; i < 5; i++) off[i] = g_adc[i];
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_SET);
  delay_us(IR_ON_US);
  for (int i = 0; i < 5; i++) on[i] = g_adc[i];
  HAL_GPIO_WritePin(EMIT_PORT[g], EMIT_PIN[g], GPIO_PIN_RESET);
  for (int r = 0; r < cfg::IR_COUNT; r++) {
    if (ROLE_GROUP[r] != g) continue;
    const uint8_t a = ROLE_ADC[r];
    g_ir[r] = to_mm(a, std::fabs(static_cast<float>(on[a]) - static_cast<float>(off[a])));
  }
  g_group = static_cast<uint8_t>((g + 1) % 3);
#else
  // den luon sang: DMA cap nhat lien tuc, doc thang (khong ton thoi gian cho)
  for (int r = 0; r < cfg::IR_COUNT; r++) {
    const uint8_t a = ROLE_ADC[r];
    g_ir[r] = to_mm(a, static_cast<float>(g_adc[a]));
  }
#endif
}

// ---------------- encoder ----------------
int32_t g_enc_l = 0, g_enc_r = 0;
uint16_t g_last_l = 0;
uint32_t g_last_r = 0;

void enc_tick() {
  const uint16_t cl = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&htim3));  // trai: TIM3 16 bit
  g_enc_l += static_cast<int16_t>(cl - g_last_l);
  g_last_l = cl;
  const uint32_t cr = __HAL_TIM_GET_COUNTER(&htim2);  // phai: TIM2 32 bit
  g_enc_r += static_cast<int32_t>(cr - g_last_r);
  g_last_r = cr;
}

// ---------------- motor (2 chan PWM / motor, vd DRV8833) ----------------
// SLOW DECAY (da kiem chung tren robot 2026-11-10 bang test_dc): chan tien luon 1, chan kia bam xung (1 - duty).
// Luc xung tat 2 dau motor noi nhau = phanh -> toc do ti le thang voi duty, giam duty la ham duoc.
// (Fast decay cu: duty nho quay qua nhanh, khong phanh -> banh chay vuot profile.)
// KV/KA/KS trong config.h do o che do nay. duty == 0 dung bang (motion tat) -> tha troi de day tay duoc.
void motor_set(TIM_HandleTypeDef* h, uint32_t ch_fwd, uint32_t ch_rev, float duty) {
  const uint32_t top = __HAL_TIM_GET_AUTORELOAD(h) + 1u;
  if (duty == 0.0f) {
    __HAL_TIM_SET_COMPARE(h, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(h, ch_rev, 0);
    return;
  }
  const float a = std::fabs(duty) > 1.0f ? 1.0f : std::fabs(duty);
  const uint32_t off = static_cast<uint32_t>(a * static_cast<float>(top));
  __HAL_TIM_SET_COMPARE(h, duty > 0 ? ch_fwd : ch_rev, top);
  __HAL_TIM_SET_COMPARE(h, duty > 0 ? ch_rev : ch_fwd, top - off);
}
// trai: PWM_L1 = TIM8_CH1 (PC6), PWM_L2 = TIM8_CH2 (PC7); phai: PWM_R1 = TIM4_CH1 (PB6), PWM_R2 = TIM4_CH2 (PB7)
void motors(float l, float r) {
  motor_set(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, l);
  motor_set(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, r);
}

// ---------------- gyro SPI2 (MPU-6500/9250 hoac ICM-42688) ----------------
enum class Imu : uint8_t { NONE, MPU6500, ICM42688 };
Imu g_imu = Imu::NONE;
constexpr float GYRO_LSB_RAD = (3.14159265f / 180.0f) / 16.4f;  // +-2000 dps

void imu_cs(bool on) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

void imu_write(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
  imu_cs(true);
  HAL_SPI_Transmit(&hspi2, tx, 2, 5);
  imu_cs(false);
}

bool imu_read(uint8_t reg, uint8_t* buf, uint16_t n) {
  uint8_t tx[8] = {static_cast<uint8_t>(reg | 0x80)}, rx[8] = {};
  imu_cs(true);
  const bool ok = HAL_SPI_TransmitReceive(&hspi2, tx, rx, static_cast<uint16_t>(n + 1), 2) == HAL_OK;
  imu_cs(false);
  std::memcpy(buf, rx + 1, n);
  return ok;
}

volatile bool g_imu_busy = false;   // vong lap chinh dang dung SPI (watchdog) -> ngat khong doc gyro
volatile uint32_t g_imu_fix = 0;    // so lan phat hien chip IMU tu reset va da danh thuc lai

bool imu_accel_ok() {   // kiem chung chip kieu MPU: |gia toc| ~ 1 g (16384 LSB o +-2 g sau reset)
  uint8_t a[6];
  if (!imu_read(0x3B, a, 6)) return false;
  const float ax = static_cast<int16_t>((a[0] << 8) | a[1]), ay = static_cast<int16_t>((a[2] << 8) | a[3]),
              az = static_cast<int16_t>((a[4] << 8) | a[5]);
  const float g = std::sqrt(ax * ax + ay * ay + az * az) / 16384.0f;
  return g > 0.7f && g < 1.3f;
}

void imu_init() {
  // cau hinh o toc do SPI thap (MPU-6500 gioi han 1 MHz khi ghi thanh ghi)
  const uint32_t pres = hspi2.Init.BaudRatePrescaler;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  HAL_SPI_Init(&hspi2);
  imu_cs(false);
  HAL_Delay(150);   // ICM-206xx: toi 100 ms sau cap nguon moi doc/ghi thanh ghi duoc
  uint8_t who = 0;
  for (int k = 0; k < 5; k++) {   // chip tren robot doi luc tra 0xFF / gia tri la: doc lai vai lan
    imu_read(0x75, &who, 1);
    if (who != 0xFF && who != 0x00) break;
    HAL_Delay(20);
  }
  // MPU-65xx; ICM-2060x (0x12 ICM-20602...). Chip tren robot: ban clone ghi ICM-20602, WHO_AM_I KHONG on dinh
  // (0x2E / 0xFF / 0x3F) nhung bang thanh ghi MPU-6500 -> chip la van khoi tao roi kiem chung bang gia toc.
  const bool mpu65 = who == 0x70 || who == 0x71 || who == 0x73 || who == 0x74 || who == 0x68;
  const bool icm206 = who == 0x12 || who == 0x11 || who == 0xAC || who == 0xAF || who == 0x98;
  if (who != 0x47) {
    imu_write(0x6B, 0x80); HAL_Delay(100);
    imu_write(0x6B, 0x01); HAL_Delay(10);
    if (mpu65) imu_write(0x6A, 0x10);          // MPU-65xx: tat I2C
    else if (icm206) imu_write(0x70, 0x40);    // ICM-2060x: I2C_IF_DIS
    imu_write(0x19, 0x00);
    imu_write(0x1A, 0x01);   // DLPF 184 Hz
    imu_write(0x1B, 0x18);   // +-2000 dps
    HAL_Delay(50);
    g_imu = (mpu65 || icm206 || imu_accel_ok()) ? Imu::MPU6500 : Imu::NONE;
  } else {
    imu_write(0x11, 0x01); HAL_Delay(5);
    imu_write(0x4E, 0x0F); HAL_Delay(1);  // gyro + accel low noise
    imu_write(0x4F, 0x06);                // +-2000 dps, ODR 1 kHz
    HAL_Delay(50);
    g_imu = Imu::ICM42688;
  }
  hspi2.Init.BaudRatePrescaler = pres;
  HAL_SPI_Init(&hspi2);
  hw::log("IMU who_am_i 0x%02x -> %s\r\n", who, g_imu == Imu::MPU6500 ? "MPU" : (g_imu == Imu::ICM42688 ? "ICM42688" : "KHONG CO (dung encoder)"));
}

// Watchdog (goi o vong lap chinh): chip IMU tu reset khi nguon chap chon (do tren robot 2026-11-10: thanh ghi ve
// mac dinh 0x41 = ngu, gyro doc 0) -> moi 200 ms kiem tra, sai thi danh thuc + cau hinh lai (khong reset, < 1 ms)
void imu_watchdog() {
  static uint32_t t_last = 0;
  if (g_imu != Imu::MPU6500 || HAL_GetTick() - t_last < 200) return;
  t_last = HAL_GetTick();
  g_imu_busy = true;
  const uint32_t pres = hspi2.Init.BaudRatePrescaler;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
    hw::log("IMU tu reset (PWR_MGMT_1 0x%02x) -> da danh thuc lai, lan %lu\r\n", pm, static_cast<unsigned long>(g_imu_fix));
  }
  hspi2.Init.BaudRatePrescaler = pres;
  HAL_SPI_Init(&hspi2);
  g_imu_busy = false;
}

float gyro_z(int32_t dl, int32_t dr) {
  static float last = 0.0f;
  if (g_imu_busy) return last;   // watchdog dang dung SPI: giu gia tri cu 1 ms
  uint8_t b[2];
  if (g_imu == Imu::MPU6500 && imu_read(0x47, b, 2)) return last = static_cast<int16_t>((b[0] << 8) | b[1]) * GYRO_LSB_RAD;
  if (g_imu == Imu::ICM42688 && imu_read(0x29, b, 2)) return static_cast<int16_t>((b[0] << 8) | b[1]) * GYRO_LSB_RAD;
  // khong co gyro: toc do quay tu 2 banh
  const float vl = cfg::ENC_SIGN_L * dl / cfg::COUNTS_PER_MM_L, vr = cfg::ENC_SIGN_R * dr / cfg::COUNTS_PER_MM_R;
  return cfg::GYRO_SIGN * (vr - vl) / cfg::TRACK_MM / cfg::DT_S;
}

// ---------------- luu ban do: ghi noi tiep vao sector 11 (128 KB) ----------------
constexpr uint32_t FL_BASE = 0x080E0000u, FL_SIZE = 0x20000u;
constexpr uint32_t MAGIC = 0x4D5A3136u;   // "MZ16"
constexpr uint32_t ERASED = 0x4D5A0000u;  // ban ghi danh dau "da xoa ban do"
struct Rec { uint32_t magic, seq, crc; uint8_t cells[cfg::N * cfg::N]; };
static_assert(sizeof(Rec) % 4 == 0, "Rec phai chia het 4");
uint32_t g_seq = 0, g_last_crc = 0;

const Rec* rec_at(uint32_t i) { return reinterpret_cast<const Rec*>(FL_BASE + i * sizeof(Rec)); }
constexpr uint32_t REC_MAX = FL_SIZE / sizeof(Rec);

void flash_append(const Rec& r) {
  uint32_t slot = 0;
  while (slot < REC_MAX && rec_at(slot)->magic != 0xFFFFFFFFu) slot++;
  HAL_FLASH_Unlock();
  if (slot >= REC_MAX) {
    FLASH_EraseInitTypeDef e = {};
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = FLASH_SECTOR_11;
    e.NbSectors = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t err = 0;
    HAL_FLASHEx_Erase(&e, &err);
    slot = 0;
  }
  const uint32_t* w = reinterpret_cast<const uint32_t*>(&r);
  const uint32_t addr = FL_BASE + slot * sizeof(Rec);
  for (uint32_t k = 0; k < sizeof(Rec) / 4; k++) HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * k, w[k]);
  HAL_FLASH_Lock();
}

// ---------------- log luu flash: sector 10 (128 KB), ghi noi tiep ----------------
// Cam/rut ST-Link lam robot reset kieu mat nguon (do bang hop den 2026-11-10) -> log RAM mat het.
// Moi dong log ghi them vao flash (chu ASCII, dem bang 0; het = tu 0xFFFFFFFF). Doc bang tools/readlog.py.
constexpr uint32_t LOG_BASE = 0x080C0000u, LOG_SIZE = 0x20000u, LOG_MAGIC = 0x31474F4Cu;   // "LOG1"
uint32_t g_flog = 0;   // dia chi trong tiep theo (0 = chua san sang)

void flog_init() {
  uint32_t a = LOG_BASE + 4;
  const bool fresh = *reinterpret_cast<const volatile uint32_t*>(LOG_BASE) != LOG_MAGIC;
  if (!fresh)
    while (a < LOG_BASE + LOG_SIZE && *reinterpret_cast<const volatile uint32_t*>(a) != 0xFFFFFFFFu) a += 4;
  if (fresh || a > LOG_BASE + LOG_SIZE - 4096) {   // chua co (vd sau test_dc) hoac gan day: xoa lam lai (~1-2 s)
    FLASH_EraseInitTypeDef e = {};
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = FLASH_SECTOR_10;
    e.NbSectors = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t err = 0;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, LOG_BASE, LOG_MAGIC);
    HAL_FLASH_Lock();
    a = LOG_BASE + 4;
  }
  g_flog = a;
}

void flog_write(const char* s, int len) {
  if (!g_flog || g_flog + static_cast<uint32_t>(len) + 8u > LOG_BASE + LOG_SIZE) return;   // day: bo
  HAL_FLASH_Unlock();
  for (int i = 0; i < len; i += 4) {
    uint32_t w = 0;
    for (int k = 0; k < 4; k++) w |= static_cast<uint32_t>(static_cast<uint8_t>(i + k < len ? s[i + k] : 0)) << (8 * k);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, g_flog, w);
    g_flog += 4;
  }
  HAL_FLASH_Lock();
}

volatile bool g_ready = false;
uint32_t g_log_pos = 0;
Mouse g_mouse;

}  // namespace

// bo dem log khi khong co UART (extern de debugger thay ten)
extern "C" { char g_log[2048]; }

// ---------------- dich vu cho core ----------------
namespace mm::hw {
uint32_t millis() { return HAL_GetTick(); }
bool btn_mode() { return HAL_GPIO_ReadPin(BTN_Mode_GPIO_Port, BTN_Mode_Pin) == GPIO_PIN_RESET; }
bool btn_start() { return HAL_GPIO_ReadPin(BTN_Start_GPIO_Port, BTN_Start_Pin) == GPIO_PIN_RESET; }
void system_reset() {
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  NVIC_SystemReset();
}

bool load_maze(Maze& m) {
  const Rec* best = nullptr;
  for (uint32_t i = 0; i < REC_MAX && rec_at(i)->magic != 0xFFFFFFFFu; i++) {
    const Rec* r = rec_at(i);
    if (r->magic == ERASED) { best = nullptr; g_seq = r->seq; continue; }
    if (r->magic != MAGIC) continue;
    Maze t;
    for (int c = 0; c < cfg::N * cfg::N; c++) t.set_raw(static_cast<uint8_t>(c), r->cells[c]);
    if (t.checksum() == r->crc) { best = r; g_seq = r->seq; }
  }
  if (!best) return false;
  for (int c = 0; c < cfg::N * cfg::N; c++) m.set_raw(static_cast<uint8_t>(c), best->cells[c]);
  g_last_crc = best->crc;
  return true;
}

void save_maze(const Maze& m) {
  const uint32_t crc = m.checksum();
  if (crc == g_last_crc) return;
  static Rec r;
  r.magic = MAGIC;
  r.seq = ++g_seq;
  r.crc = crc;
  for (int c = 0; c < cfg::N * cfg::N; c++) r.cells[c] = m.raw(static_cast<uint8_t>(c));
  flash_append(r);
  g_last_crc = crc;
}

void erase_maze() {
  static Rec r;
  std::memset(&r, 0, sizeof r);
  r.magic = ERASED;
  r.seq = ++g_seq;
  flash_append(r);
  g_last_crc = 0;
}

void log(const char* fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  const int len = n < 127 ? n : 127;
  {   // flash: "[giay.mili] dong log"
    const uint32_t t = HAL_GetTick();
    char line[150];
    const int m = snprintf(line, sizeof line, "[%5lu.%03lu] %.*s", static_cast<unsigned long>(t / 1000),
                           static_cast<unsigned long>(t % 1000), len, buf);
    if (m > 0) flog_write(line, m < static_cast<int>(sizeof line) ? m : static_cast<int>(sizeof line) - 1);
  }
#ifdef HAL_UART_MODULE_ENABLED
  HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(buf), static_cast<uint16_t>(len), 50);
#else
  // khong co UART: ghi vao bo dem vong, xem g_log bang Live Expressions / Memory khi debug
  for (int i = 0; i < len; i++) { g_log[g_log_pos] = buf[i]; g_log_pos = (g_log_pos + 1) % sizeof g_log; }
#endif
}
}  // namespace mm::hw

// ---------------- diem vao tu main.c ----------------
extern "C" void App_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  emitters_off();
  for (auto& v : g_ir) v = cfg::IR_NONE;
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  motors(0, 0);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  g_last_l = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&htim3));
  g_last_r = __HAL_TIM_GET_COUNTER(&htim2);
  HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(g_adc)), ADC_CH);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT);  // chi doc buffer, khong can ngat

  flog_init();
  {
    const uint32_t csr = RCC->CSR;
    hw::log("=== KHOI DONG: %s%s%s%s%s\r\n", (csr & RCC_CSR_PORRSTF) ? "MAT NGUON (POR) " : "",
            (csr & RCC_CSR_PINRSTF) ? "chan NRST/cam day " : "", (csr & RCC_CSR_SFTRSTF) ? "reset mem " : "",
            (csr & RCC_CSR_IWDGRSTF) ? "watchdog " : "", (csr & RCC_CSR_BORRSTF) ? "sut ap (BOR) " : "");
    RCC->CSR |= RCC_CSR_RMVF;
  }
  imu_init();
  if (!IR_CALIB_VALID) hw::log("CANH BAO: chua co he so k, n (stm32/sensor_calib.h) -> robot khong thay tuong\r\n");
  g_mouse.init();
  g_ready = true;
}

extern "C" void App_Loop(void) {
  g_mouse.loop();
  imu_watchdog();
}

extern "C" void App_Tick1kHz(void) {
  if (!g_ready) return;
  const int32_t l0 = g_enc_l, r0 = g_enc_r;
  enc_tick();
  ir_tick();
  TickIn in;
  in.enc_l = g_enc_l;
  in.enc_r = g_enc_r;
  in.gyro_z = gyro_z(g_enc_l - l0, g_enc_r - r0);
  for (int i = 0; i < cfg::IR_COUNT; i++) in.ir[i] = g_ir[i];
  in.vbat = cfg::VBAT_NOMINAL;  // chua co mach do pin
  TickOut out;
  g_mouse.tick(in, out);
  motors(out.duty_l, out.duty_r);
}
