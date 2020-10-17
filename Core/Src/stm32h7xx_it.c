/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdio.h>
#include <string.h>
//#include "vl53l0x/vl53l0x_class.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi4;
/* USER CODE BEGIN EV */
int count=0;
int gyro_c=0;
int enc_tim1 = 0;
t_SensorRawData sensor_raw;
//VL53L0X_Dev_t sensor1;
int sensor1_flag = 1;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    printf("Error_HardFault");
    return ;
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    if ( LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1 ) {
        LL_TIM_ClearFlag_UPDATE(TIM3);
//        uint32_t distance;
//        int status=VL53L0X_GetDistance(&sensor1, &distance);
//        printf("%d %d\r\n",status, distance);
//        sensor_raw.dist = distance;
//        LL_TIM_EnableCounter(TIM3);
        LL_TIM_EnableIT_UPDATE(TIM3);
    }
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
    if ( LL_TIM_IsActiveFlag_UPDATE(TIM5) == 1 ) {
        LL_TIM_ClearFlag_UPDATE(TIM5);
        /* 割り込み処理を入れる */
        count++;
        if (count == 500) {
            LL_GPIO_SetOutputPin(GPIOB, GPIO_BSRR_BS14);
        } else if (count == 1000) {
            LL_GPIO_ResetOutputPin(GPIOB, GPIO_BSRR_BS14);
            count = 1;
        }

        if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)) {
            count = 0;
            LL_GPIO_SetOutputPin(GPIOB, GPIO_BSRR_BS0); //LD1
            LL_GPIO_ResetOutputPin(GPIOE, GPIO_BSRR_BS1); //LD2
        } else {
            LL_GPIO_ResetOutputPin(GPIOB, GPIO_BSRR_BS0); //LD1
            LL_GPIO_SetOutputPin(GPIOE, GPIO_BSRR_BS1); //LD2
        }
        uint32_t distance;
        sensor_raw.encoder.r = LL_TIM_GetCounter(TIM1) - 30000;
        enc_tim1 =  LL_TIM_GetCounter(TIM1) - 30000;
        LL_TIM_SetCounter(TIM1,30000);

//        printf("enc(tim1) = %d  %d  %d  %d  %d  %d\r\n", sensor_raw.encoder.r, sensor_raw.gyro.rawdata[0], sensor_raw.gyro.rawdata[1],
//               sensor_raw.gyro.rawdata[2], sensor_raw.gyro.rawdata[3], sensor_raw.gyro.rawdata[4]);


//        LL_TIM_EnableCounter(TIM5);
        LL_TIM_EnableIT_UPDATE(TIM5);
//        HAL_USART_Transmit(&huart3,(uint8_t *)msg,sizeof(msg),3000);
//        return (uint16_t)rx_data[1];
    }
  /* USER CODE END TIM5_IRQn 0 */
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI4 global interrupt.
  */
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */

  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */
    if ( LL_TIM_IsActiveFlag_UPDATE(TIM15) == 1 ) {

        LL_TIM_ClearFlag_UPDATE(TIM15);
        uint8_t tx_data[3];
        tx_data[0] = 0x47 | 0x80;
        tx_data[1] = 0;
        tx_data[2] = 0x00;  // dummy
        uint8_t rx_data[3];
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
        HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 3, 1);
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
        int tmp_gyro = (signed short) ((((unsigned int) (rx_data[1] & 0xff)) << 8)
                                       | ((unsigned int) (rx_data[2] & 0xff)));

        sensor_raw.gyro.rawdata[gyro_c] = tmp_gyro;
        gyro_c++;
        if (gyro_c == 5) {
            gyro_c = 0;
        }

//        LL_TIM_EnableCounter(TIM15);
        LL_TIM_EnableIT_UPDATE(TIM15);
    }
  /* USER CODE END TIM15_IRQn 0 */
  /* USER CODE BEGIN TIM15_IRQn 1 */

  /* USER CODE END TIM15_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int getEncTIM1(){
    return enc_tim1;
}

t_SensorRawData getSensorData(){
    return sensor_raw;
}

void setSensorData(const t_SensorRawData *tmp) {
    sensor_raw.gyro.rawdata[1] = tmp->gyro.rawdata[1];
}

//VL53L0X_Dev_t* getVL53L0Xinstance(){
//    return &sensor1;
//}

void VL53L0X_Start() {
//    sensor1.I2cDevAddr = 0x29; // ADDRESS_DEFAULT;
//    sensor1.comms_type = 1;    // VL53L0X_COMMS_I2C
//    sensor1.comms_speed_khz = 100;
//    if (!VL53L0X_InitSensor(&sensor1,
//                            VL53L0x_DEFAULT_DEVICE_ADDRESS)) {
//        //attempt to initialise it with the necessary settings for normal operation. Returns 0 if fail, 1 if success.
//        printf("Failed to initialize\r\n");
//    } else {
//        printf("Successfully initialized\r\n");
//    }
}
void get_tof(){
//    uint32_t distance;
//    int status=VL53L0X_GetDistance(&sensor1, &distance);
//    printf("%d %d\r\n",status, distance);
//    sensor_raw.dist = distance;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
