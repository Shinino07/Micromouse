#ifndef MM_APP_H
#define MM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);       /* goi 1 lan sau cac MX_xxx_Init() trong main.c */
void App_Loop(void);       /* goi lien tuc trong while(1) */
void App_Tick1kHz(void);   /* goi trong SysTick_Handler (stm32f4xx_it.c), sau HAL_IncTick() */

#ifdef __cplusplus
}
#endif

#endif
