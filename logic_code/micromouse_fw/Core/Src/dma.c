/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

void MX_DMA_Init(void)
{

  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
