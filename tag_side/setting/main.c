/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <example_selection.h>
#include "rb.h"
#include "port.h"
#include "shared_defines.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_interface.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
volatile int gTimerCnt=0, B1_count=0, _B1_count;
volatile int _sec=0, sec=0, min=0, hour=0, Timer_on_Flag=0, B1_Flag=0;
uint8_t time_buffer[256], Tx_buffer[256];

host_using_spi_e host_spi = SPI_1;

void dummy_wakeup(void);
volatile uint8_t rx_ready_flag = 0;
/* 3. drive list (probe chip) */
extern struct dwt_driver_s dw3000_driver; // SDK driver info
//extern volatile uint8_t rx_ready_flag;
struct dwt_driver_s *dw_drivers[] = { &dw3000_driver };
extern USBD_HandleTypeDef hUsbDeviceFS;
extern int read_dev_id(void);
extern int simple_tx(void);
extern int simple_tx_pdoa(void);
extern int simple_tx_automotive(void);
extern int simple_rx(void);
extern int simple_rx_nlos(void);
extern int rx_diagnostics(void);
extern int rx_sniff(void);
extern int double_buffer_rx(void);
extern int rx_with_xtal_trim(void);
extern int simple_rx_pdoa(void);
extern int simple_rx_cir(void);
extern int tx_sleep(void);
extern int tx_sleep_idleRC(void);
extern int tx_sleep_auto(void);
extern int tx_timed_sleep(void);
extern int tx_with_cca(void);
extern int tx_wait_resp(void);
extern int tx_wait_resp_int(void);
extern int rx_send_resp(void);
extern int rx_adc_capture(void);
extern int ds_twr_initiator(void);
extern int ds_twr_responder(void);
extern int ss_twr_initiator(void);
extern int ss_twr_responder(void);
extern int simple_rx_sts_sdc(void);
extern int simple_tx_aes(void);
extern int simple_rx_aes(void);
extern int ds_twr_initiator_sts(void);
extern int ds_twr_responder_sts(void);
extern int ds_twr_sts_sdc_initiator(void);
extern int ds_twr_sts_sdc_responder(void);
extern int ss_twr_initiator_sts(void);
extern int ss_twr_responder_sts(void);
extern int ss_twr_initiator_sts_no_data(void);
extern int ss_twr_responder_sts_no_data(void);
extern int ss_aes_twr_initiator(void);
extern int ss_aes_twr_responder(void);
extern int ack_data_tx(void);
extern int ack_data_rx(void);
extern int continuous_wave_example(void);
extern int continuous_frame_example(void);
extern int bw_cal(void);
extern int pll_cal(void);
extern int tx_power_adjustment_example(void);
extern int simple_uart_upcount(void);
extern int simple_uart_io(void);
extern int simple_uart_rb(void);
extern int test_barometer(void);
extern int ds_twr_barometer_responder(void);
extern int ds_twr_barometer_initiator(void);
extern void usb_test_com(void);


RingFifo_t gtUart4Fifo;
uint8_t rb_ch;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  uint8_t ctrl1 = 0x20;
  HAL_I2C_Mem_Write(&hi2c1, 0xB8, 0x10, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 10);

#if defined(TEST_READING_DEV_ID)
    read_dev_id();
#endif

#if defined(TEST_SIMPLE_TX)
    simple_tx();
#endif

#if defined(TEST_SIMPLE_TX_PDOA)
    simple_tx_pdoa();
#endif

#if defined(TEST_SIMPLE_TX_AUTOMOTIVE)
    simple_tx_automotive();
#endif

#if defined(TEST_SIMPLE_RX)
    simple_rx();
#endif

#if defined(TEST_SIMPLE_RX_NLOS)
    simple_rx_nlos();
#endif

#if defined(TEST_RX_DIAG)
    rx_diagnostics();
#endif

#if defined(TEST_RX_SNIFF)
    rx_sniff();
#endif

#if defined(TEST_DOUBLE_BUFFER_RX)
    double_buffer_rx();
#endif

#if defined(TEST_RX_TRIM)
    rx_with_xtal_trim();
#endif

#if defined(TEST_SIMPLE_RX_PDOA)
    simple_rx_pdoa();
#endif

#if defined(TEST_SIMPLE_RX_CIR)
    simple_rx_cir();
#endif

#if defined(TEST_TX_SLEEP)
    tx_sleep();
#endif

#if defined(TEST_TX_SLEEP_IDLE_RC)
    tx_sleep_idleRC();
#endif

#if defined(TEST_TX_SLEEP_AUTO)
    tx_sleep_auto();
#endif

#if defined(TEST_TX_SLEEP_TIMED)
    tx_timed_sleep();
#endif

#if defined(TEST_TX_WITH_CCA)
    tx_with_cca();
#endif

#if defined(TEST_TX_WAIT_RESP)
    tx_wait_resp();
#endif

#if defined(TEST_TX_WAIT_RESP_INT)
    tx_wait_resp_int();
#endif

#if defined(TEST_RX_SEND_RESP)
    rx_send_resp();
#endif

#if defined(TEST_RX_ADC_CAPTURE)
    rx_adc_capture();
#endif

#if defined(TEST_DS_TWR_INITIATOR)
    ds_twr_initiator();
#endif

#if defined(TEST_DS_TWR_RESPONDER)
    ds_twr_responder();
#endif

#if defined(TEST_SS_TWR_INITIATOR)
    ss_twr_initiator();
#endif

#if defined(TEST_SS_TWR_RESPONDER)
    ss_twr_responder();
#endif

#if defined(TEST_SIMPLE_RX_STS_SDC)
    simple_rx_sts_sdc();
#endif

#if defined(TEST_SIMPLE_TX_AES)
    simple_tx_aes();
#endif

#if defined(TEST_SIMPLE_RX_AES)
    simple_rx_aes();
#endif

#if defined(TEST_DS_TWR_INITIATOR_STS)
    ds_twr_initiator_sts();
#endif

#if defined(TEST_DS_TWR_RESPONDER_STS)
    ds_twr_responder_sts();
#endif

#if defined(TEST_DS_TWR_STS_SDC_INITIATOR)
    ds_twr_sts_sdc_initiator();
#endif

#if defined(TEST_DS_TWR_STS_SDC_RESPONDER)
    ds_twr_sts_sdc_responder();
#endif

#if defined(TEST_SS_TWR_INITIATOR_STS)
    ss_twr_initiator_sts();
#endif

#if defined(TEST_SS_TWR_RESPONDER_STS)
    ss_twr_responder_sts();
#endif

#if defined(TEST_AES_SS_TWR_INITIATOR)
    ss_aes_twr_initiator();
#endif

#if defined(TEST_AES_SS_TWR_RESPONDER)
    ss_aes_twr_responder();
#endif

#if defined(TEST_ACK_DATA_TX)
    ack_data_tx();
#endif

#if defined(TEST_ACK_DATA_RX)
    ack_data_rx();
#endif

#if defined(TEST_CONTINUOUS_WAVE)
    continuous_wave_example();
#endif

#if defined(TEST_CONTINUOUS_FRAME)
    continuous_frame_example();
#endif

#if defined(TEST_BW_CAL)
    bw_cal();
#endif

#if defined(TEST_PLL_CAL)
    pll_cal();
#endif

#if defined(TEST_TX_POWER_ADJUSTMENT)
    tx_power_adjustment_example();
#endif

#if defined(TEST_SIMPLE_UART_UPCOUNT)
    simple_uart_upcount();
#endif

#if defined(TEST_SIMPLE_UART_IO)
    simple_uart_io();
#endif

#if defined(TEST_SIMPLE_UART_RB)
    simple_uart_rb();
#endif

#if defined(TEST_BAROMETER)
    test_barometer();
#endif

#if defined(TEST_SS_TWR_INITIATOR_STS_NO_DATA)
    ss_twr_initiator_sts_no_data();
#endif

#if defined(TEST_SS_TWR_RESPONDER_STS_NO_DATA)
    ss_twr_responder_sts_no_data();
#endif

#if defined(TEST_BAROMETER_DS_TWR_RESPONDER)
    ds_twr_barometer_responder();
    //ds_twr_responder();
#endif

#if defined(TEST_BAROMETER_DS_TWR_INITIATOR)
    ds_twr_barometer_initiator();
#endif

#if defined(TEST_USB_COM)
    usb_test_com();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t msg[] = "UART4 Test\r\n";
	  HAL_UART_Transmit(&huart4, msg, sizeof(msg), 100);
	  HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DW_NSS_Pin|BARO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_RED1_Pin|LED_RED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_2_GPIO_Port, STATUS_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DW_NSS_Pin */
  GPIO_InitStruct.Pin = DW_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_WAKEUP_Pin */
  GPIO_InitStruct.Pin = DW_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_Pin GPIO1_Pin GPIO4_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin|GPIO1_Pin|GPIO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED1_Pin LED_RED2_Pin */
  GPIO_InitStruct.Pin = LED_RED1_Pin|LED_RED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_2_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BARO_CS_Pin */
  GPIO_InitStruct.Pin = BARO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BARO_CS_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOD, LED_RED1_Pin, GPIO_PIN_SET);
}

/* USER CODE BEGIN 4 */
#if 1
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *UartHandle)
{

#if defined(TEST_SIMPLE_UART_RB) | defined(TEST_BAROMETER)
	uint8_t rx;
#endif

	if (UartHandle->Instance == UART4)
	{

#if defined(TEST_SIMPLE_UART_RB) | defined(TEST_BAROMETER)
		rx = (uint8_t) (UartHandle->Instance->DR & (uint8_t) 0x00FF);
		RB_write (&gtUart4Fifo, rx);
		HAL_UART_Receive_IT(UartHandle, &rb_ch, 1);
#endif

#if defined(TEST_SIMPLE_UART_IO)
		rx_ready_flag = 1;
#endif

	}
}
#endif
void test_run_info(unsigned char *data) {
    uint32_t len = strlen((char*)data);

    // USB 연결 상태를 확인하는 가드 코드 추가
    if (hUsbDeviceFS.pClassData != NULL) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
        if (hcdc->TxState == 0) {
            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, len);
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
    }
    // UART로도 전송
    HAL_UART_Transmit(&huart4, (uint8_t*)data, len, 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 10);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
