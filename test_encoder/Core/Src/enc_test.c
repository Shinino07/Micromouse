/**
 * enc_test.c - Test 2 encoder: dem xung cong don, khong tran khi quay nhieu vong.
 * Banh trai: TIM3 (PA6 ENC_LA, PA7 ENC_LB, 16 bit). Banh phai: TIM2 (PA0, PA1, 32 bit).
 * Motor giu PWM = 0 suot bai test.
 */
#include "enc_test.h"
#include "main.h"
#include "tim.h"

#define BTN_DEBOUNCE_MS 30u

volatile int32_t g_enc_l, g_enc_r;
volatile int32_t g_cap_l, g_cap_r;
volatile uint32_t g_cap_count;
volatile uint32_t g_zero_count;

static uint16_t s_last_l;
static uint32_t s_last_r;

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

static void motors_off(void)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

static void update_counts(void)
{
  const uint16_t cl = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_enc_l += (int16_t)(cl - s_last_l);  /* 16 bit: hieu so co dau xu ly duoc tran */
  s_last_l = cl;
  const uint32_t cr = __HAL_TIM_GET_COUNTER(&htim2);
  g_enc_r += (int32_t)(cr - s_last_r);
  s_last_r = cr;
}

void ENC_TestInit(void)
{
  motors_off();
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  s_last_l = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  s_last_r = __HAL_TIM_GET_COUNTER(&htim2);
  g_enc_l = g_enc_r = 0;
}

void ENC_TestLoop(void)
{
  update_counts();
  if (button_pressed(&s_btn_start)) {
    g_enc_l = g_enc_r = 0;
    g_zero_count++;
  }
  if (button_pressed(&s_btn_mode)) {
    g_cap_l = g_enc_l;
    g_cap_r = g_enc_r;
    g_cap_count++;
  }
}
