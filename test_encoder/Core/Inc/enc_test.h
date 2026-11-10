/**
 * enc_test.h - Test 2 encoder (quay banh bang tay, motor tat)
 *
 * Xem bang CubeIDE Live Expressions (hoac doc qua ST-Link):
 *   g_enc_l, g_enc_r       : so xung cong don banh trai (TIM3) / phai (TIM2), dau theo chieu quay
 *   g_cap_l, g_cap_r       : so xung luc bam BTN_Mode lan gan nhat
 *   g_cap_count            : so lan da bam BTN_Mode
 * BTN_Start (PA11): dua g_enc_l, g_enc_r ve 0
 * BTN_Mode  (PA12): chot ket qua vao g_cap_*
 */
#ifndef ENC_TEST_H
#define ENC_TEST_H

#include <stdint.h>

extern volatile int32_t g_enc_l, g_enc_r;
extern volatile int32_t g_cap_l, g_cap_r;
extern volatile uint32_t g_cap_count;
extern volatile uint32_t g_zero_count;

void ENC_TestInit(void);
void ENC_TestLoop(void);

#endif
