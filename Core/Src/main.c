/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32h7xx_it.h"
#include "../../Drivers/api/inc/vl53l0x_api.h"
#include "vl53l0x_calibration.h"
//#include "vl53l0x/vl53l0x_class.h"
#include "PhysicalBasement.h"
//#include "../../Drivers/VL53L0X/core/inc/vl53l0x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t char_buf[256];
uint8_t rx_data[2];
uint8_t tx_data[2];
uint8_t Message[64];
uint8_t MessageLen;
VL53L0X_Dev_t myDevStruct[3];
VL53L0X_DEV myDev;
void writeByte(uint8_t reg, uint8_t data)
{

    tx_data[0] = reg & 0x7F;
    tx_data[1] = data;  // write data

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 2, 1);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
}
uint8_t readByte(uint8_t reg)
{

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00;  // dummy

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 2, 1);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);

    return rx_data[1];
}

uint16_t read1Byte(uint8_t reg)
{

    uint8_t tx_data[2];
    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00;  // dummy
    uint8_t rx_data[2];
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 2, 1);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);

    return (uint16_t)rx_data[1];
}
uint16_t read2Byte(uint8_t reg)
{
    uint8_t tx_data[3];
    tx_data[0] = reg | 0x80;
    tx_data[1] = 0;
    tx_data[2] = 0x00;  // dummy
    uint8_t rx_data[3];
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 3, 1);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
    signed short tmp = (signed short) ((((unsigned int) (rx_data[1] & 0xff)) << 8)
                               | ((unsigned int) (rx_data[2] & 0xff)));
    printf("%d, %d, %d, %d\r\n", rx_data[0], rx_data[1], rx_data[2], tmp);
    return (uint16_t)rx_data[1];
}

uint8_t g_tx_data[3];
uint8_t g_rx_data[3];
uint16_t read2Byte2(uint8_t reg)
{
    g_tx_data[0] = reg | 0x80;
    g_tx_data[1] = 0;
    g_tx_data[2] = 0x00;  // dummy
    g_rx_data[0] = g_rx_data[1] = g_rx_data[2] =0;

    HAL_SPI_Transmit_IT(&hspi4, g_tx_data, 3);
    HAL_SPI_Receive_IT(&hspi4, g_rx_data, 3);

    auto tmp = (signed short) ((((unsigned int) (g_rx_data[1] & 0xff)) << 8)
                               | ((unsigned int) (g_rx_data[2] & 0xff)));
    printf("   %d, %d, %d, %d\r\n", g_rx_data[0], g_rx_data[1], g_rx_data[2], tmp);
    return (uint16_t)rx_data[1];
}
float mpu6500_read_gyro_z( void )
{
    int16_t gyro_z;
    float omega;

    return omega;
}
#define VL53L0X_MAX_STRING_LENGTH 32

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if (Status == VL53L0X_ERROR_NONE) {
        printf("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        printf("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
                                               &VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if (Status == VL53L0X_ERROR_NONE) // needed if a coverglass is used and no calibration has been performed
    {
        printf("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                                                  &refSpadCount, &isApertureSpads); // Device Initialization
        printf("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(Status);
    }

    if (Status == VL53L0X_ERROR_NONE) {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        printf("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                                             VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                            (FixPoint1616_t) (0.25 * 65536));
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                            (FixPoint1616_t) (18 * 65536));
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
                                                                200000);
    }
    /*
     *  Step  4 : Test ranging mode
     */

    if (Status == VL53L0X_ERROR_NONE) {
        for (i = 0; i < 10; i++) {
            printf("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
                                                             &RangingMeasurementData);

//            print_pal_error(Status);
//            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
                                         VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            printf("RANGE IGNORE THRESHOLD: %f\n\n", (float) LimitCheckCurrent / 65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

            printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM5_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
//    HAL_SPI_MspInit(&hspi4);

    auto res = HAL_SPI_Init(&hspi4);
    HAL_SPI_IRQHandler(&hspi4);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    setbuf(stdout, NULL);
    printf("who am i = %d\r\n", readByte(0x75));
//    writeByte(0x6B, 0x80);
//    VL53L0X_Start();
//    vl53l0x_calibration();

//    myDev = &myDevStruct[0];
//    VL53L0X_StartMeasurement(myDev);
//    VL53L0X_ClearInterruptMask(myDev, -1);
//    HAL_Delay(1);
    if (HAL_I2C_Init(&hi2c2) != HAL_OK){
        printf("NG\r\n");
    }else{
        printf("OK\r\n");
    }
    LL_mDelay(1000);
//    I2Cx_FORCE_RESET();
//    HAL_I2C_Mem_Write(&hi2c2, 0x29, 0x6b, I2C_MEMADD_SIZE_8BIT, (uint8_t *) ret, 0x01, 100);
    printf("%d\r\n",hi2c2.State);
    LL_mDelay(1000);
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t Version;
    VL53L0X_Version_t *pVersion = &Version;
    VL53L0X_DeviceInfo_t DeviceInfo;
    LL_mDelay(1000);

    writeByte(0x6B, 0x01);
    LL_mDelay(10);
    writeByte(0x1A, 0x00);
    LL_mDelay(10);
    writeByte(0x6A, 0x10);
    LL_mDelay(10);
    writeByte(0x1B, 0x18);
    LL_mDelay(10);

    LL_TIM_EnableCounter(TIM5);
    LL_TIM_EnableIT_UPDATE(TIM5);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM15);
    LL_TIM_EnableIT_UPDATE(TIM15);

    LL_TIM_EnableCounter(TIM1);
    RT_MODEL_PhysicalBasement_T PhysicalBasement_M;
    t_SensorData data;
    real_T out1, out2;
    data.light_sensor.right45 = 1000;
    PhysicalBasement_step(&PhysicalBasement_M, &data, &out1, &out2);
    printf("%f, %f \r\n",out1,out2);
    LL_mDelay(3000);
    // Enable/Disable Sigma and Signal check

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      setbuf(stdout, NULL);
      t_SensorRawData sen_data = getSensorData();
//      get_tof();
//      uint32_t distance;
//      int status=VL53L0X_GetDistance(&sensor1, &distance);
//      printf("%d %d\r\n",status, distance);
      printf("enc(tim1) = %d  %d  %d  %d  %d  %d  %d\r\n", sen_data.encoder.r, sen_data.gyro.rawdata[0], sen_data.gyro.rawdata[1],
             sen_data.gyro.rawdata[2], sen_data.gyro.rawdata[3], sen_data.gyro.rawdata[4],sen_data.dist);
//      MessageLen = sprintf((char *) Message, "Measured distance: %i\n\r", RangingData.RangeMilliMeter);
//      printf("%s\r\n", MessageLen);
//      HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
//      TofDataRead = 0;

      LL_mDelay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSI48_Enable();

   /* Wait till HSI48 is ready */
  while(LL_RCC_HSI48_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(2);
  LL_RCC_PLL1_SetN(240);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(4);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);
  LL_SetSystemCoreClock(480000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C123_CLKSOURCE_PCLK1);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PCLK2);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
}

/* USER CODE BEGIN 4 */
void USART_TransmitByte(uint8_t ch){
    LL_USART_TransmitData8(USART3,ch);
    while(LL_USART_IsActiveFlag_TXE(USART3)==0);
}

void __io_putchar(uint8_t ch){
    USART_TransmitByte(ch);
}
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi){
  printf("tx callback\r\n");

  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
}
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi){
  printf("rx callback\r\n");

  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);

    auto tmp = (signed short) ((((unsigned int) (g_rx_data[1] & 0xff)) << 8)
                               | ((unsigned int) (g_rx_data[2] & 0xff)));
    printf("   %d, %d, %d, %d\r\n", g_rx_data[0], g_rx_data[1], g_rx_data[2], tmp);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
