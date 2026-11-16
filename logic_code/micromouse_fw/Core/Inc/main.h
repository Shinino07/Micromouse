/* USER CODE BEGIN Header */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* USER CODE BEGIN EM */

/* USER CODE END EM */

void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#define IR_5_Pin GPIO_PIN_0
#define IR_5_GPIO_Port GPIOC
#define IR_side_Pin GPIO_PIN_1
#define IR_side_GPIO_Port GPIOC
#define IR_front_Pin GPIO_PIN_2
#define IR_front_GPIO_Port GPIOC
#define IR_1_Pin GPIO_PIN_3
#define IR_1_GPIO_Port GPIOA
#define IR_2_Pin GPIO_PIN_4
#define IR_2_GPIO_Port GPIOA
#define IR_diag_Pin GPIO_PIN_5
#define IR_diag_GPIO_Port GPIOA
#define ENC_LA_Pin GPIO_PIN_6
#define ENC_LA_GPIO_Port GPIOA
#define ENC_LB_Pin GPIO_PIN_7
#define ENC_LB_GPIO_Port GPIOA
#define IR_3_Pin GPIO_PIN_0
#define IR_3_GPIO_Port GPIOB
#define IR_4_Pin GPIO_PIN_1
#define IR_4_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define PWM_L1_Pin GPIO_PIN_6
#define PWM_L1_GPIO_Port GPIOC
#define PWM_L2_Pin GPIO_PIN_7
#define PWM_L2_GPIO_Port GPIOC
#define BTN_Start_Pin GPIO_PIN_11
#define BTN_Start_GPIO_Port GPIOA
#define BTN_Start_EXTI_IRQn EXTI15_10_IRQn
#define BTN_Mode_Pin GPIO_PIN_12
#define BTN_Mode_GPIO_Port GPIOA
#define BTN_Mode_EXTI_IRQn EXTI15_10_IRQn
#define PWM_R1_Pin GPIO_PIN_6
#define PWM_R1_GPIO_Port GPIOB
#define PWM_R2_Pin GPIO_PIN_7
#define PWM_R2_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
