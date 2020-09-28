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
    auto tmp = (signed short) ((((unsigned int) (rx_data[1] & 0xff)) << 8)
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
#define LEN 3

void read_gyro() {

//    uint8_t rx_data[LEN];
//    uint8_t tx_data[LEN];
//    // H:8bit shift, Link h and l
//    tx_data[0] = 0x47 | 0x80;
//    tx_data[1] = 0x00;  // dummy
//    tx_data[2] = 0x00;  // dummy
//    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
//    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, LEN, 1);
//    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
//    printf("%d, %d, %d\r\n", tx_data[0], tx_data[1], tx_data[2]);
}

void read_gyro2() {

//    unsigned char rx_data[LEN];
//    unsigned char tx_data[LEN];
//    // H:8bit shift, Link h and l
//    tx_data[0] = 0x47 | 0x80;
//    tx_data[1] = 0x00;  // dummy
//    tx_data[2] = 0x00;  // dummy
//
//    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
////    HAL_SPI_Transmit(&hspi4, &tx_data, 3, 100);//Select reg
////    HAL_SPI_Receive(&hspi4, &rx_data, 3, 100);//Read data
//    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
//    printf("%d, %d, %d\r\n", tx_data[0], tx_data[1], tx_data[2]);
//    printf("%d, %d, %d\r\n", rx_data[0], rx_data[1], rx_data[2]);
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
  /* USER CODE BEGIN 2 */
//    HAL_SPI_MspInit(&hspi4);
    LL_TIM_EnableCounter(TIM5);
    LL_TIM_EnableIT_UPDATE(TIM5);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM15);
    LL_TIM_EnableIT_UPDATE(TIM15);
    auto res = HAL_SPI_Init(&hspi4);
    HAL_SPI_IRQHandler(&hspi4);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
    setbuf(stdout, NULL);
    printf("who am i = %d\r\n", readByte(0x75));
//    writeByte(0x6B, 0x80);
    writeByte(0x6B, 0x01);
    LL_mDelay(10);
    writeByte(0x1A, 0x00);
    LL_mDelay(10);
    writeByte(0x6A, 0x10);
    LL_mDelay(10);
    writeByte(0x1B, 0x18);
    LL_mDelay(10);
//    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    LL_TIM_EnableCounter(TIM1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      setbuf(stdout, NULL);
      uint32_t state = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13);
      auto high = readByte(0x47) & 0xff;
      auto rx0_h = rx_data[0];
      auto rx1_h = rx_data[1];
      auto low = readByte(0x48) & 0xff;
      auto rx0_l = rx_data[0];
      auto rx1_l = rx_data[1];
//      snprintf(char_buf, 256, "hello world btn state = %d, %d, %f, %d, %d, %d\r\n", (int)state, res, mpu6500_read_gyro_z(), high, low, TIM1->CNT);
//      snprintf(char_buf, 256, "hello world btn state = %d, %d, %f\r\n", (int)state, res, readByte(0x47));
//      printf("%s",char_buf);
      t_SensorRawData sen_data = getSensorData();

//      printf("hello\r\n");
      printf("enc(tim1) = %d  %d  %d  %d  %d  %d\r\n", sen_data.encoder.r, sen_data.gyro.rawdata[0], sen_data.gyro.rawdata[1],
             sen_data.gyro.rawdata[2], sen_data.gyro.rawdata[3], sen_data.gyro.rawdata[4]);

//      sen_data2.gyro.rawdata[0] = 1000;
//      sen_data2.gyro.rawdata[1] = 1000;
//      setSensorData(&sen_data2);
//      sen_data = getSensorData();
//
//      printf("enc(tim2) = %d  %d\r\n", sen_data.encoder.r, sen_data.gyro.rawdata[1]);

//      float gyro_z = read2Byte(0x47);

      LL_mDelay(1);
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
