#ifndef MM_APP_H
#define MM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Loop(void);
void App_Tick1kHz(void);

#ifdef __cplusplus
}
#endif

#endif
