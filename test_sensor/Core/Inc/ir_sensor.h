/**
 * ir_sensor.h - Test 5 cap mat IR (thu = ADC, phat = 3 nhom GPIO qua MOSFET)
 *
 * Gia tri moi sensor = |ADC khi bat den phat - ADC khi tat| (tru anh sang moi truong).
 * Khoang cach: d[mm] = k * value^(-n)   (k, n lay tu sensor_calib.h do Python sinh ra)
 */
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>

#define IR_NUM        5     /* IR_1..IR_5 = PA3, PA4, PB0, PB1, PC0 (ADC rank 1..5) */
#define IR_GROUPS     3     /* 0 = front (PC2), 1 = side (PC1), 2 = diag (PA5) */
#define IR_AUTO_GROUP 0xFF  /* tu chon nhom den cho tin hieu manh nhat */

typedef struct {
  uint16_t value[IR_NUM];    /* |on - off| cua nhom den gan voi sensor */
  uint16_t ambient[IR_NUM];  /* ADC luc tat den (anh sang moi truong) */
  uint16_t on_max[IR_NUM];   /* ADC lon/nho nhat luc bat den, de bat bao hoa */
  uint16_t on_min[IR_NUM];
  uint8_t  group[IR_NUM];    /* nhom den da dung cho tung sensor */
  uint8_t  inverted[IR_NUM]; /* 1 = mach thu lam ADC giam khi co anh sang */
} IR_Reading;

void IR_Init(void);
/* Do trung binh 'frames' lan (moi lan bat lan luot 3 nhom den) */
void IR_Read(IR_Reading *r, uint16_t frames);
/* Ma tran (on - off)[nhom][sensor], co dau, de biet den nao chieu vao mat thu nao */
void IR_ReadMatrix(int16_t delta[IR_GROUPS][IR_NUM], uint16_t frames);
/* Doi gia tri ra mm; tra ve -1 neu chua hieu chuan hoac value = 0 */
int32_t IR_ToMm(uint8_t sensor, uint16_t value);

/* Khong co UART: xem cac bien nay bang CubeIDE Live Expressions khi debug */
extern volatile uint16_t g_ir_live[8];       /* ADC tho IR_1..IR_5 (chi dung 5 o dau) */
extern volatile uint16_t g_ir_value[IR_NUM]; /* |on - off| moi 200 ms: nhom den front = IR_3, side = IR_1/IR_5, diag = IR_2/IR_4 */
extern volatile int16_t g_ir_mm[IR_NUM];     /* khoang cach (mm), -1 = chua hieu chuan */
extern volatile uint16_t g_ir_cap[IR_NUM];   /* value lan CAP gan nhat */
extern volatile uint32_t g_ir_cap_count;

/* Vong lap test: goi IR_TestInit() 1 lan, IR_TestLoop() trong while(1) */
void IR_TestInit(void);
void IR_TestLoop(void);

#endif
