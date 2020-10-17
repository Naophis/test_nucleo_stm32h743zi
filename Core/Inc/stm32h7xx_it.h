/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "vl53l0x/vl53l0x_class.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM3_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI4_IRQHandler(void);
void TIM15_IRQHandler(void);
/* USER CODE BEGIN EFP */
int getEncTIM1();

typedef struct {
    int16_t r;
    int16_t l;
} t_EncoderData;
#define GYRO_DATA_SIZE 5
typedef struct {
    int16_t rawdata[GYRO_DATA_SIZE];
} t_GyroData;

typedef struct {
    t_EncoderData encoder;
    t_GyroData gyro;
    int32_t dist;
} t_SensorRawData;

t_SensorRawData getSensorData();
void setSensorData(const t_SensorRawData *tmp);
//VL53L0X_Dev_t* getVL53L0Xinstance();
void VL53L0X_Start();
void get_tof();
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
