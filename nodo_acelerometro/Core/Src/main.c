/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "stm32l475e_iot01.h"
#include <math.h>
#include <string.h>
#include "stm32l475e_iot01_accelero.h"

#include "es_wifi.h"
#include "wifi.h"

#include "core_mqtt.h"
#include "mqtt_priv.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MODO_NORMAL 0x0001U
#define MODO_CONTINUO 0x0002U
#define TIMER_MQTT 0x0004U

#define MUESTRAS_NORMAL 64
#define MUESTRAS_CONTINUO 1024

#define MAX_LEN_SSID 10
#define MAX_LEN_PSSWRD 20

#define WIFISECURITY WIFI_ECN_WPA2_PSK
//#define WIFISECURITY WIFI_ECN_OPEN
#define SOCKET 0
#define SOCKETSUBS 1
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for conf_inicial */
osThreadId_t conf_inicialHandle;
const osThreadAttr_t conf_inicial_attributes = {
  .name = "conf_inicial",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readAccel */
osThreadId_t readAccelHandle;
const osThreadAttr_t readAccel_attributes = {
  .name = "readAccel",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tarea_UART */
osThreadId_t tarea_UARTHandle;
const osThreadAttr_t tarea_UART_attributes = {
  .name = "tarea_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for temporizador */
osThreadId_t temporizadorHandle;
const osThreadAttr_t temporizador_attributes = {
  .name = "temporizador",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for clientMQTT */
osThreadId_t clientMQTTHandle;
const osThreadAttr_t clientMQTT_attributes = {
  .name = "clientMQTT",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_queue */
osMessageQueueId_t print_queueHandle;
const osMessageQueueAttr_t print_queue_attributes = {
  .name = "print_queue"
};
/* Definitions for receive_queue */
osMessageQueueId_t receive_queueHandle;
const osMessageQueueAttr_t receive_queue_attributes = {
  .name = "receive_queue"
};
/* Definitions for publish_queue */
osMessageQueueId_t publish_queueHandle;
const osMessageQueueAttr_t publish_queue_attributes = {
  .name = "publish_queue"
};
/* USER CODE BEGIN PV */
RTC_DateTypeDef GetDate; //Estructura para fijar/leer fecha
RTC_TimeTypeDef GetTime; //Estructura para fijar/leer hora

ACCELERO_StatusTypeDef status_acc;

volatile bool modo_continuo;
volatile frec_muestreo_actual = 52;

extern  SPI_HandleTypeDef hspi;
static  uint8_t  IP_Addr[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM7_Init(void);
static void MX_RTC_Init(void);
void conf_inicial_func(void *argument);
void readAccel_func(void *argument);
void printTask_func(void *argument);
void tarea_UART_func(void *argument);
void temporizador_func(void *argument);
void clientMQTT_func(void *argument);

/* USER CODE BEGIN PFP */
volatile unsigned long ulHighFrequencyTimerTicks;
void configureTimerForRunTimeStats(void) {
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim7);
}
unsigned long getRunTimeCounterValue(void) {
	return ulHighFrequencyTimerTicks;

}

ACCELERO_StatusTypeDef BSP_ACCELERO_Init_INT(uint8_t frec, uint8_t fs);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rec_data;
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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM7_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  status_acc = BSP_ACCELERO_Init_INT(LSM6DSL_ODR_52Hz, LSM6DSL_ACC_FULLSCALE_2G);
  if (status_acc == ACCELERO_OK){
	  printf("Acelerometro inicializado\r\n");
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of print_queue */
  print_queueHandle = osMessageQueueNew (2, sizeof(uintptr_t), &print_queue_attributes);

  /* creation of receive_queue */
  receive_queueHandle = osMessageQueueNew (1, sizeof(uint8_t), &receive_queue_attributes);

  /* creation of publish_queue */
  publish_queueHandle = osMessageQueueNew (1024, sizeof(uintptr_t), &publish_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of conf_inicial */
  conf_inicialHandle = osThreadNew(conf_inicial_func, NULL, &conf_inicial_attributes);

  /* creation of readAccel */
  readAccelHandle = osThreadNew(readAccel_func, NULL, &readAccel_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(printTask_func, NULL, &printTask_attributes);

  /* creation of tarea_UART */
  tarea_UARTHandle = osThreadNew(tarea_UART_func, NULL, &tarea_UART_attributes);

  /* creation of temporizador */
  temporizadorHandle = osThreadNew(temporizador_func, NULL, &temporizador_attributes);

  /* creation of clientMQTT */
  clientMQTTHandle = osThreadNew(clientMQTT_func, NULL, &clientMQTT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  HAL_UART_Receive_IT(&huart1,&rec_data,1);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 999;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 13;
  sTime.Minutes = 1;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 799;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|LED1_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BOTON_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = BOTON_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin LED1_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|LED1_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx=0; DataIdx<len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

ACCELERO_StatusTypeDef BSP_ACCELERO_Init_INT(uint8_t frec, uint8_t fs)
{
	ACCELERO_StatusTypeDef ret;
	ret = BSP_ACCELERO_Init(frec, fs);
	if (ret == ACCELERO_OK)
	{
		/* Initialize interruption*/
		uint8_t tmp;
		tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G);
		tmp |=0b10000000;
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G, tmp);

		tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);
		tmp |=0b00000001;
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, tmp);

		tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG);
		tmp |=0b10000000;
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG, tmp);
	}
	return ret;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	static osStatus_t estado;
	if (huart == &huart1)
	{
		HAL_UART_Receive_IT(&huart1,&rec_data,1);
		printf("Recibido: %d\r\n",rec_data);
		osThreadFlagsSet(tarea_UARTHandle,0x0002U);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case (LSM6DSL_INT1_EXTI11_Pin):
		{
			osThreadFlagsSet(readAccelHandle,0x0001U);
			break;
		}
		case(BOTON_Pin):
		{
			printf("Ha pulsado el boton\r\n");
			osThreadFlagsSet(readAccelHandle,0x0002U);
			break;
		}
		case (GPIO_PIN_1):
		{
			SPI_WIFI_ISR();
			break;
		}
		default:
		{
		  break;
		}
	}
}

static int wifi_start(void)
{
  uint8_t  MAC_Addr[6];
  printf("wifistart\r\n");
 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    printf(("ES-WIFI Initialized.\r\n"));
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      printf("MAC asignada\r\n");
      printf("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);
    }
    else
    {
      printf("> ERROR : CANNOT get MAC address\r\n");
      return -1;
    }
  }
  else
  {
	printf("Errorfifi\r\n");
    return -1;
  }
  return 0;
}

int wifi_connect(char* SSID, char* PASSWORD)
{

  wifi_start();

  printf("Connecting to %s\r\n",SSID);
  if( WIFI_Connect(SSID, PASSWORD, WIFISECURITY) == WIFI_STATUS_OK)
  {
    if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
    {
      printf("> es-wifi module connected: got IP Address : %d.%d.%d.%d\r\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]);
    }
    else
    {
		  printf(" ERROR : es-wifi module CANNOT get IP address\r\n");
      return -1;
    }
  }
  else
  {
		 printf("ERROR : es-wifi module NOT connected\n");
     return -1;
  }
  return 0;
}


/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

void prvMQTTProcessIncomingPublish( MQTTPublishInfo_t *pxPublishInfo )
{
	char buffer1[128];
	char buffer2[128];
    const char * pTopicName;
    uint8_t i;
    uint8_t limit[6][2] = {{0,23},{0,59},{0,59},{1,31},{1,12},{0,99}};
    uint16_t numero_usuario,frec_muestreo, frec_muestreo_str;
    uint8_t fondo_escala, fondo_escala_str;
    uint8_t to_change[6];
    bool dato_erroneo;
	const char* msg_error = "\r\nERROR: Valor de configuracion no v??lido\r\n";
	const char* msg_hora_ok = "\r\nHora cambiada correctamente\r\n";
	const char* msg_fecha_ok = "Fecha cambiada correctamente\r\n";

	uint8_t valores_frec_accel[3] = {52,104,208};
	uint8_t valores_frec_accel_str[3] = {LSM6DSL_ODR_52Hz, LSM6DSL_ODR_104Hz, LSM6DSL_ODR_208Hz};
	uint8_t valores_fs_accel[3] = {2,4,8};
	uint8_t valores_fs_accel_str[3] = {LSM6DSL_ACC_SENSITIVITY_2G, LSM6DSL_ACC_SENSITIVITY_4G, LSM6DSL_ACC_SENSITIVITY_8G};
	const char* msg_accelero_reconf = "ACELEROMETRO RECONFIGURADO\r\n";

	// pPayload no termina en \0, hay que copiarlo en un buffer para imprimirlo. Lo mismo con pTopicName
	memcpy(buffer1,pxPublishInfo->pPayload,min(127,pxPublishInfo->payloadLength));
	buffer1[min(1023,pxPublishInfo->payloadLength)]='\0';
	memcpy(buffer2,pxPublishInfo->pTopicName,min(127,pxPublishInfo->topicNameLength));
	buffer2[min(1023,pxPublishInfo->topicNameLength)]='\0';

	printf("Topic \"%s\": publicado \"%s\"\n",buffer2,buffer1);

  // Actuar localmente sobre los LEDs o alguna otra cosa

	for(i=0;i<strlen(tempSupTopic);i++){
		if(buffer2[i] != tempSupTopic[i]){
			break;
		}
	}
	if (i == strlen(tempSupTopic)){
		if(buffer1[0]=='1'){
			modo_continuo = true;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}else if(buffer1[0]=='0'){
			modo_continuo = false;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}

	}

	for(i=0;i<strlen(rtcConfTopic);i++){
		if(buffer2[i] != rtcConfTopic[i]){
			break;
		}
	}
	if (i == strlen(rtcConfTopic)){
		dato_erroneo = false;
		for (i=0;i<6 && !dato_erroneo;){
			numero_usuario = 10*(buffer1[3*i]-48) + buffer1[3*i+1]-48;
			printf("Dato: %d\r\n",numero_usuario);
			printf("Rango: %d-%d\r\n",limit[i][0],limit[i][1]);
			if (numero_usuario<limit[i][0] || numero_usuario>limit[i][1]){
				dato_erroneo = true;
				osMessageQueuePut(print_queueHandle, &msg_error, 0, pdMS_TO_TICKS(500));

			}else{
				to_change[i]=numero_usuario;
				i++;
			}

		}
		if(i == 6){
			RTC_TimeTypeDef sTime = {0};
			RTC_DateTypeDef sDate = {0};

			sTime.Hours = to_change[0];
			sTime.Minutes = to_change[1];
			sTime.Seconds = to_change[2];

			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
			  {
				Error_Handler();
			  }

			osMessageQueuePut(print_queueHandle, &msg_hora_ok, 0, pdMS_TO_TICKS(500));

			sDate.Date = to_change[3];
			sDate.Month = to_change[4];
			sDate.Year = to_change[5];
			printf("Anio: %d\r\n",to_change[5]);
			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
			{
				Error_Handler();
			}

			osMessageQueuePut(print_queueHandle, &msg_fecha_ok, 0, pdMS_TO_TICKS(500));
		}
	}

	for(i=0;i<strlen(accelConfTopic);i++){
		if(buffer2[i] != accelConfTopic[i]){
			break;
		}
	}
	if (i == strlen(accelConfTopic)){
		frec_muestreo = 100*(buffer1[0]-48) + 10*(buffer1[1]-48) + buffer1[2]-48;
		fondo_escala = buffer1[4]-48;
		for(i=0 ; i<strlen(valores_frec_accel) ; i++){
			if (frec_muestreo == valores_frec_accel[i]){
				frec_muestreo_str = valores_frec_accel_str[i];
				break;
			}
		}
		if (i == strlen(valores_frec_accel)){
			osMessageQueuePut(print_queueHandle, &msg_error, 0, pdMS_TO_TICKS(500));
		}else{
			for(i=0 ; i<strlen(valores_fs_accel) ; i++){
				if (fondo_escala == valores_fs_accel[i]){
					fondo_escala_str = valores_fs_accel_str[i];
					break;
				}
			}
			if (i == strlen(valores_fs_accel)){
				osMessageQueuePut(print_queueHandle, &msg_error, 0, pdMS_TO_TICKS(500));
			}else{
				//Intentamos configurar el acelerometro
				status_acc = BSP_ACCELERO_Init_INT(frec_muestreo_str, fondo_escala_str);
				if (status_acc == ACCELERO_OK){
					frec_muestreo_actual = frec_muestreo;
					osMessageQueuePut(print_queueHandle, &msg_accelero_reconf, 0, pdMS_TO_TICKS(500));
				}
			}
		}


	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_conf_inicial_func */
/**
  * @brief  Function implementing the conf_inicial thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_conf_inicial_func */
void conf_inicial_func(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t recibido[20];
	//uint32_t flag_rec;
	osStatus_t estado;
	uint32_t return_wait = 0U;

	uint16_t num_usuario;
	uint8_t to_change[6];
	const char* msg_hora_ok = "\r\nHora cambiada correctamente\r\n";
	const char* msg_fecha_ok = "Fecha cambiada correctamente\r\n";
	const char* msg_error = "\r\nERROR: Valor no v??lido\r\n";
	const char* msg_rtc1 = "\r\n\r\n========================\r\n"
	"| Configurar rtc |\r\n"
	"========================\r\n\r\n";
	const char* msg[6] = {
	"Hora (0-23): ", "\r\nMinuto (0-59): ","\r\nSegundo (0-59): ","\r\nD??a (1-31): ","\r\nMes (1-12): ",
	"\r\nA??o (0-99): "};
	uint8_t limit[6][2] = {{0,23},{0,59},{0,59},{1,31},{1,12},{0,99}};


	printf("Empieza el bucle\r\n");
	estado = osMessageQueuePut(print_queueHandle, &msg_rtc1, 0, pdMS_TO_TICKS(500));
	int i,j,m = 0;
	for (i=0;i<6;){
		estado = osMessageQueuePut(print_queueHandle, &msg[i], 0, pdMS_TO_TICKS(500));
		printf("Esperando a que ser reciba el dato\r\n");

		for (j=0;j<3;){
			estado = osMessageQueueGet(receive_queueHandle, &recibido[j], NULL, osWaitForever);
			printf("De la cola: %c\r\n",recibido[j]);
			if(recibido[j]==13){
				printf("Ha pulsado intro\r\n");
				break;
			}
			if (recibido[j]==127){
				printf("Ha pulsado borrar\r\n");
				if (j>0) j--;
			}else{
				j++;
			}
		}
		printf("%d\r\n",j);
		switch(j){
		case 0:
			num_usuario=0;
			break;
		case 1:
			num_usuario = recibido[0]-48;
			//i++;
			break;
		case 2:
			num_usuario = 10*(recibido[0]-48)+recibido[1]-48;
			//i++;
			break;
		case 3:
			num_usuario = 100*(recibido[0]-48)+10*(recibido[1]-48)+recibido[2]-48;
			break;
		}
		printf("Numero: %d\r\n",num_usuario);
		printf("Rango: %d-%d\r\n",limit[i][0],limit[i][1]);
		if (num_usuario<limit[i][0] || num_usuario>limit[i][1]){
			estado = osMessageQueuePut(print_queueHandle, &msg_error, 0, pdMS_TO_TICKS(500));
			if (estado == osOK)
				printf("Enviado valor erroneo\r\n");
			else
				printf("Algo no va bien\r\n");
		}else{
			to_change[i]=num_usuario;
			i++;
		}

	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	sTime.Hours = to_change[0];
	sTime.Minutes = to_change[1];
	sTime.Seconds = to_change[2];

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }

	osMessageQueuePut(print_queueHandle, &msg_hora_ok, 0, pdMS_TO_TICKS(500));

	sDate.Date = to_change[3];
	sDate.Month = to_change[4];
	sDate.Year = to_change[5];
	printf("Anio: %d\r\n",to_change[5]);
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	osMessageQueuePut(print_queueHandle, &msg_fecha_ok, 0, pdMS_TO_TICKS(500));

	const char* msg_wifi_conf_init = "\r\nInicio de configuraci??n del WiFi\r\n";
	const char* msg_wifi_connect_init = "\r\nConectando al WiFi\r\n";
	const char* msg_wifi_connect_error = "No se ha podido conectar, vuelva a introducir los datos\r\n";
	const char* msg_wifi_connect_success = "\r\nCONECTADO\r\n";
	const char* msg_too_many_characters = "\r\nHas introducido demasiados caracteres, prueba de nuevo\r\n";
	const char* msg_introduce_ssid = "Introduce el ssid: ";
	const char* msg_introduce_psswrd = "\r\nIntroduce la contrase??a: ";


	osMessageQueuePut(print_queueHandle, &msg_wifi_conf_init, 0, pdMS_TO_TICKS(500));

	//bucle de conexi??n
	while (1){

		//configuracion ssid
		osMessageQueuePut(print_queueHandle, &msg_introduce_ssid, 0, pdMS_TO_TICKS(500));
		for (j=0; j<MAX_LEN_SSID ; ){
			estado = osMessageQueueGet(receive_queueHandle, &recibido[j], NULL, osWaitForever);
			printf("De la cola: %c\r\n",recibido[j]);
			if(recibido[j]==13){
				printf("Ha pulsado intro\r\n");
				break;
			}
			if (recibido[j]==127){
				printf("Ha pulsado borrar\r\n");
				if (j>0) j--;
			}else{
				j++;
			}
		}
		if (j==MAX_LEN_SSID){
			osMessageQueuePut(print_queueHandle, &msg_too_many_characters, 0, pdMS_TO_TICKS(500));
		}else{
//			printf("Guardamos el ssid\r\n");
			char ssid[j];
//			printf("j: %d\r\n",j);
			for (m=0 ; m<j ; m++){
//				printf("m: %d\r\n",m);
//				printf("caracter: %c\r\n",recibido[m]);
				ssid[m] = recibido[m];
			}

			//configuracion contrase??a
			osMessageQueuePut(print_queueHandle, &msg_introduce_psswrd, 0, pdMS_TO_TICKS(500));

			for (j=0; j<MAX_LEN_PSSWRD ; ){
				estado = osMessageQueueGet(receive_queueHandle, &recibido[j], NULL, osWaitForever);
//				printf("De la cola: %c\r\n",recibido[j]);
				if(recibido[j]==13){
//					printf("Ha pulsado intro\r\n");
					break;
				}
				if (recibido[j]==127){
//					printf("Ha pulsado borrar\r\n");
					if (j>0) j--;
				}else{
					j++;
				}
			}
			if (j==MAX_LEN_PSSWRD){
				osMessageQueuePut(print_queueHandle, &msg_too_many_characters, 0, pdMS_TO_TICKS(500));
			}else{
//				printf("Guardamos el psswrd\r\n");
				char psswrd[j];
//				printf("j: %d\r\n",j);
				for (m=0 ; m<j ; m++){
//					printf("m: %d\r\n",m);
//					printf("caracter: %c\r\n",recibido[m]);
					psswrd[m] = recibido[m];
				}

				osMessageQueuePut(print_queueHandle, &msg_wifi_connect_init, 0, pdMS_TO_TICKS(500));

				if (wifi_connect(ssid,psswrd) != 0){
					osMessageQueuePut(print_queueHandle, &msg_wifi_connect_error, 0, pdMS_TO_TICKS(500));
				}
				else{
					break;
				}

			}
		}


	}

	osMessageQueuePut(print_queueHandle, &msg_wifi_connect_success, 0, pdMS_TO_TICKS(500));


	osThreadFlagsSet(clientMQTTHandle,0x0001U);


  /* Infinite loop */
  for(;;)
  {

	  osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_readAccel_func */
/**
* @brief Function implementing the readAccel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readAccel_func */
void readAccel_func(void *argument)
{
  /* USER CODE BEGIN readAccel_func */
	osStatus_t estado;

	char mensaje[100];
	char *p_mensaje = mensaje;

	char mensaje_params[100];
	char* p_mensaje_params = mensaje_params;

	int16_t DataXYZ[3];
	int16_t *pDataXYZ = DataXYZ;


	uint8_t horas,minutos,segundos,dia,mes,anio = 0;
	uint16_t milisegundos = 0;
	uint32_t return_wait = 0U;

	uint16_t iter; // Se usa para iterar en 64 o 1024 aceleraciones
	uint16_t max_iter;

	printf("ReadAccel task esperando\r\n");
	// Esperamos que el usuario configure el RTC y que el acelerometro este activo
	return_wait = osThreadFlagsWait(0x0008U, osFlagsWaitAll, osWaitForever);

	//Activamos el temporizador
	osThreadFlagsSet(temporizadorHandle,0x0001U);


	printf("ReadAccel task se inicia\r\n");

	const char* msg_read_normal = "\r\nLectura en modo normal\r\n";
	const char* msg_read_continuous = "\r\nLectura en modo continuo\r\n";


	/* Infinite loop */
	for(;;)
	{
		return_wait = osThreadFlagsWait(0x0006U, osFlagsWaitAny, osWaitForever); //espera media hora o que alguien pulse el boton
		if(return_wait == osFlagsErrorTimeout){
			printf("Ha pasado media hora\r\n");
		}
		else {
			printf("El usuario quiere enviar aceleraciones, modo continuo = %d\r\n",modo_continuo);
		}

		if (modo_continuo){
			max_iter = MUESTRAS_CONTINUO;
			osMessageQueuePut(print_queueHandle, &msg_read_continuous, 0, pdMS_TO_TICKS(500));
		}else{
			max_iter = MUESTRAS_NORMAL;
			osMessageQueuePut(print_queueHandle, &msg_read_normal, 0, pdMS_TO_TICKS(500));
		}

		snprintf(mensaje_params, 100, "Publicadas: %d muestras, frec_muestreo: %dHz",max_iter,frec_muestreo_actual);

		for (iter=0 ; iter<max_iter ; iter++){
			osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
			BSP_ACCELERO_AccGetXYZ(pDataXYZ);

			HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
			horas = GetTime.Hours;
			minutos = GetTime.Minutes;
			segundos = GetTime.Seconds;
			milisegundos = (uint16_t)1000*((float)(GetTime.SecondFraction-GetTime.SubSeconds)) / ((float)(GetTime.SecondFraction+1));

			HAL_RTC_GetDate(&hrtc, &GetDate, RTC_FORMAT_BIN);
			anio = GetDate.Year;
			dia = GetDate.Date;
			mes = GetDate.Month;


			snprintf(mensaje,100,"%d/%d/%d %d:%d:%d:%d %d,%d,%d",dia,mes,anio+2000,horas,minutos,segundos,milisegundos,DataXYZ[0],DataXYZ[1],DataXYZ[2]);

			printf("iter: %d\r\n",iter);


			osMessageQueuePut(publish_queueHandle, &p_mensaje, 0, pdMS_TO_TICKS(500));

		}


		osMessageQueuePut(publish_queueHandle, &p_mensaje_params, 0, pdMS_TO_TICKS(500));
		printf("Se han leido todas las aceleraciones, esperamos media hora o hasta que alguien pulse el boton\r\n");


	}
  /* USER CODE END readAccel_func */
}

/* USER CODE BEGIN Header_printTask_func */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printTask_func */
void printTask_func(void *argument)
{
  /* USER CODE BEGIN printTask_func */
	uintptr_t mensaje;
	osStatus_t estado;
  /* Infinite loop */
  for(;;)
  {
	  estado = osMessageQueueGet(print_queueHandle, &mensaje, NULL, osWaitForever);

	  if (estado == osOK)
	  {
		  //printf("%s",(char*)mensaje);
		  HAL_UART_Transmit(&huart1, (uint8_t*)mensaje, strlen(mensaje),10);
	  }
	  else if (estado == osErrorTimeout)
	  {
		  printf("Timeout printTask\r\n");
	  }
	  else
	  {
		  printf("Error en la tarea print\r\n");
	  }

  }
  /* USER CODE END printTask_func */
}

/* USER CODE BEGIN Header_tarea_UART_func */
/**
* @brief Function implementing the tarea_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tarea_UART_func */
void tarea_UART_func(void *argument)
{
  /* USER CODE BEGIN tarea_UART_func */
	osStatus_t estado;
	uint32_t return_wait = 0U;
  /* Infinite loop */
  for(;;)
  {
	  return_wait = osThreadFlagsWait(0x0002U, osFlagsWaitAny, osWaitForever);
	  estado = osMessageQueuePut(receive_queueHandle, &rec_data,0,pdMS_TO_TICKS(200));
	  if (estado == osOK)
		  printf("Estado: ok\r\n");
	  else
		  printf("Algo no va bien\r\n");

  }
  /* USER CODE END tarea_UART_func */
}

/* USER CODE BEGIN Header_temporizador_func */
/**
* @brief Function implementing the temporizador thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_temporizador_func */
void temporizador_func(void *argument)
{
  /* USER CODE BEGIN temporizador_func */
	osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
	printf("Temporizador activado\r\n");
  /* Infinite loop */
  for(;;)
  {
    osDelay(pdMS_TO_TICKS(1800000)); //Periodo en ms con el que se mandan las aceleraciones
    osThreadFlagsSet(readAccelHandle,0x0004U);
  }
  /* USER CODE END temporizador_func */
}

/* USER CODE BEGIN Header_clientMQTT_func */
/**
* @brief Function implementing the clientMQTT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_clientMQTT_func */
void clientMQTT_func(void *argument)
{
  /* USER CODE BEGIN clientMQTT_func */
	uint32_t return_wait = 0U;
	osStatus_t estado;
	uintptr_t mensaje;

	uint16_t iter;
	uint16_t max_iter;

	char payLoad[16];

	const char* msg_mqtt_initialized = "\r\nMQTT INICIALIZADO\r\n\r\n";

	osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);

	const uint32_t ulMaxPublishCount = 5UL;
	NetworkContext_t xNetworkContext = { 0 };
	MQTTContext_t xMQTTContext;
	MQTTStatus_t xMQTTStatus;
	TransportStatus_t xNetworkStatus;

	/* Attempt to connect to the MQTT broker. The socket is returned in
	* the network context structure. */
	xNetworkStatus = prvConnectToServer( &xNetworkContext, SOCKET );
	printf("Mitad de la definicion mqtt\r\n");
	configASSERT( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS );
	//LOG(("Trying to create an MQTT connection\n"));
	prvCreateMQTTConnectionWithBroker( &xMQTTContext, &xNetworkContext );
	prvMQTTSubscribeToTopic(&xMQTTContext,tempSupTopic);
	prvMQTTSubscribeToTopic(&xMQTTContext,rtcConfTopic);
	prvMQTTSubscribeToTopic(&xMQTTContext,accelConfTopic);

	osMessageQueuePut(print_queueHandle, &msg_mqtt_initialized, 0, pdMS_TO_TICKS(500));
	printf("Contexto mqtt inicializado\r\n");

	osThreadFlagsSet(readAccelHandle,0x0008U);
	//osThreadFlagsSet(temp_subHandle, 0x0001U);

	const char* msg_modo_continuo = "Modo continuo\r\n";
	const char* msg_modo_normal = "Modo normal\r\n";


  /* Infinite loop */
	for(;;)
	{
		estado = osMessageQueueGet(publish_queueHandle, &mensaje, NULL, pdMS_TO_TICKS(3000)); //Minimo la mitad de tiempo de lo que tarda en actualizar el valor el otro nodo

		 if (estado == osOK)
		 {
		  //printf("Publicamos: %s",(char*)mensaje);
			 sprintf(payLoad,"%s",mensaje);
			 prvMQTTPublishToTopic(&xMQTTContext,accelTopic,payLoad);
		 }
		 else if (estado == osErrorTimeout)
		 {
			 printf("Procesamos subscripcion\r\n");
			 MQTT_ProcessLoop(&xMQTTContext);
			 if (modo_continuo) osMessageQueuePut(print_queueHandle, &msg_modo_continuo, 0, pdMS_TO_TICKS(500));
			 else osMessageQueuePut(print_queueHandle, &msg_modo_normal, 0, pdMS_TO_TICKS(500));


		 }
		 else
		 {
			 printf("Error en la tarea sendMQTT\r\n");
		 }

	}
  /* USER CODE END clientMQTT_func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
