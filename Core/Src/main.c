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
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"

#include "ds3231/ds3231.h"

#include "bme280/bme280.h"

#include "task.h"

#include "stdlib.h"
#include "queue.h"

/////// For task management
volatile unsigned long ulHighFreqebcyTimerTicks;		// This variable using for calculate how many time all tasks was running.
char str_management_memory_str[1000] = {0};
int freemem = 0;
uint32_t tim_val = 0;

typedef struct 							// Queue for UARD
{
	char Buf[1024];
}QUEUE_t;
////////////////////////////
//BMP280_HandleTypedef bmp280;
//float pressure, temperature, humidity;
//uint16_t size;
//uint8_t Data[256];
typedef struct
{
	float pressure;
	float temperature;
	float humidity;
}QUEUE_BME280;


// For button debounce
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

// RTC
typedef struct
{
  uint8_t Year;
  uint8_t Month;
  uint8_t Date;
  uint8_t DaysOfWeek;
  uint8_t Hour;
  uint8_t Min;
  uint8_t Sec;
} QUEUE_RTC;

typedef struct
{
  uint8_t Year;
  uint8_t Month;
  uint8_t Date;
  uint8_t DaysOfWeek;
  uint8_t Hour;
  uint8_t Min;
  uint8_t Sec;
} QUEUE_NEW_RTC;

bool print_first_time_on_lcd_flag = true;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTC_DS3231_Task */
osThreadId_t RTC_DS3231_TaskHandle;
const osThreadAttr_t RTC_DS3231_Task_attributes = {
  .name = "RTC_DS3231_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BPE280_Task */
osThreadId_t BPE280_TaskHandle;
const osThreadAttr_t BPE280_Task_attributes = {
  .name = "BPE280_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SET_RTS_TASK */
osThreadId_t SET_RTS_TASKHandle;
const osThreadAttr_t SET_RTS_TASK_attributes = {
  .name = "SET_RTS_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_USB_Task */
osThreadId_t UART_USB_TaskHandle;
const osThreadAttr_t UART_USB_Task_attributes = {
  .name = "UART_USB_Task",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 300 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTQueue */
osMessageQueueId_t UARTQueueHandle;
uint8_t UARTQueueBuffer[ 2 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t UARTQueueControlBlock;
const osMessageQueueAttr_t UARTQueue_attributes = {
  .name = "UARTQueue",
  .cb_mem = &UARTQueueControlBlock,
  .cb_size = sizeof(UARTQueueControlBlock),
  .mq_mem = &UARTQueueBuffer,
  .mq_size = sizeof(UARTQueueBuffer)
};
/* Definitions for buttonQueue */
osMessageQueueId_t buttonQueueHandle;
const osMessageQueueAttr_t buttonQueue_attributes = {
  .name = "buttonQueue"
};
/* Definitions for THPQueue */
osMessageQueueId_t THPQueueHandle;
uint8_t THPQueueBuffer[ 3 * sizeof( QUEUE_BME280 ) ];
osStaticMessageQDef_t THPQueueControlBlock;
const osMessageQueueAttr_t THPQueue_attributes = {
  .name = "THPQueue",
  .cb_mem = &THPQueueControlBlock,
  .cb_size = sizeof(THPQueueControlBlock),
  .mq_mem = &THPQueueBuffer,
  .mq_size = sizeof(THPQueueBuffer)
};
/* Definitions for rtc_queue */
osMessageQueueId_t rtc_queueHandle;
uint8_t rtc_queueBuffer[ 2 * sizeof( QUEUE_RTC ) ];
osStaticMessageQDef_t rtc_queueControlBlock;
const osMessageQueueAttr_t rtc_queue_attributes = {
  .name = "rtc_queue",
  .cb_mem = &rtc_queueControlBlock,
  .cb_size = sizeof(rtc_queueControlBlock),
  .mq_mem = &rtc_queueBuffer,
  .mq_size = sizeof(rtc_queueBuffer)
};
/* Definitions for new_rtc_queue */
osMessageQueueId_t new_rtc_queueHandle;
uint8_t new_rtc_queueBuffer[ 2 * sizeof( QUEUE_NEW_RTC ) ];
osStaticMessageQDef_t new_rtc_queueControlBlock;
const osMessageQueueAttr_t new_rtc_queue_attributes = {
  .name = "new_rtc_queue",
  .cb_mem = &new_rtc_queueControlBlock,
  .cb_size = sizeof(new_rtc_queueControlBlock),
  .mq_mem = &new_rtc_queueBuffer,
  .mq_size = sizeof(new_rtc_queueBuffer)
};
/* Definitions for nev_val_queue */
osMessageQueueId_t nev_val_queueHandle;
uint8_t nev_val_queueBuffer[ 2 * sizeof( uint8_t ) ];
osStaticMessageQDef_t nev_val_queueControlBlock;
const osMessageQueueAttr_t nev_val_queue_attributes = {
  .name = "nev_val_queue",
  .cb_mem = &nev_val_queueControlBlock,
  .cb_size = sizeof(nev_val_queueControlBlock),
  .mq_mem = &nev_val_queueBuffer,
  .mq_size = sizeof(nev_val_queueBuffer)
};
/* Definitions for I2C_Mutex */
osMutexId_t I2C_MutexHandle;
const osMutexAttr_t I2C_Mutex_attributes = {
  .name = "I2C_Mutex"
};
/* Definitions for LCD_Sem */
osSemaphoreId_t LCD_SemHandle;
const osSemaphoreAttr_t LCD_Sem_attributes = {
  .name = "LCD_Sem"
};
/* USER CODE BEGIN PV */


// -------------------------------------------------------------------------
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == KEY_1_Pin) || (GPIO_Pin == KEY_2_Pin) || (GPIO_Pin == KEY_3_Pin) || (GPIO_Pin == KEY_4_Pin))
	{
		BaseType_t xHigherPriorityTaskWoken;
		uint16_t key_1 = 1;
		uint16_t key_2 = 2;
		uint16_t key_3 = 3;
		uint16_t key_4 = 4;

		currentMillis = HAL_GetTick();
		if((currentMillis - previousMillis) > 100)
		{
			if(HAL_GPIO_ReadPin(GPIOA, KEY_1_Pin) == GPIO_PIN_SET)			// If first key was pressed
			{
				if((xQueueSendFromISR( buttonQueueHandle, &key_1, &xHigherPriorityTaskWoken )) != 1)		// Send queue to main task
				{
					// Error
				}
			}
			else if(HAL_GPIO_ReadPin(GPIOA, KEY_2_Pin) == GPIO_PIN_SET)
			{
				if((xQueueSendFromISR( buttonQueueHandle, &key_2, &xHigherPriorityTaskWoken )) != 1)
				{
					// Error
				}
			}
			else if(HAL_GPIO_ReadPin(GPIOA, KEY_3_Pin) == GPIO_PIN_SET)
			{
				if((xQueueSendFromISR( buttonQueueHandle, &key_2, &xHigherPriorityTaskWoken )) != 1)
				{
					// Error
				}
			}
			else if(HAL_GPIO_ReadPin(GPIOA, KEY_4_Pin) == GPIO_PIN_SET)
			{
				if((xQueueSendFromISR( buttonQueueHandle, &key_4, &xHigherPriorityTaskWoken )) != 1)
				{
					// Error
				}
			}
			previousMillis = currentMillis;
		}
	}
}
// -------------------------------------------------------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void start_RTC_DS3231_Task(void *argument);
void start_BPE280_Task(void *argument);
void start_SET_RTS_TASK(void *argument);
void start_UART_USB_Task(void *argument);
void start_LCD_Task(void *argument);

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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Init LCD
//  ILI9341_SPI_Init();
//  ILI9341_Init();
//  ILI9341_Enable();
//  ILI9341_Fill_Screen(BLACK);

  // DC3231 init
  DS3231_Init(&hi2c1);

//  // Init BME280
//  bmp280_init_default_params(&bmp280.params);
//  bmp280.addr = BMP280_I2C_ADDRESS_0;
//  bmp280.i2c = &hi2c1;
//  if(bmp280_init(&bmp280, &bmp280.params) != true)
//  {
//	  // error
//	  while(1){}
//  }
//  bool bme280p = bmp280.id == BME280_CHIP_ID;




//  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of I2C_Mutex */
  I2C_MutexHandle = osMutexNew(&I2C_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of LCD_Sem */
  LCD_SemHandle = osSemaphoreNew(1, 1, &LCD_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (2, sizeof(QUEUE_t), &UARTQueue_attributes);

  /* creation of buttonQueue */
  buttonQueueHandle = osMessageQueueNew (5, sizeof(uint16_t), &buttonQueue_attributes);

  /* creation of THPQueue */
  THPQueueHandle = osMessageQueueNew (3, sizeof(QUEUE_BME280), &THPQueue_attributes);

  /* creation of rtc_queue */
  rtc_queueHandle = osMessageQueueNew (2, sizeof(QUEUE_RTC), &rtc_queue_attributes);

  /* creation of new_rtc_queue */
  new_rtc_queueHandle = osMessageQueueNew (2, sizeof(QUEUE_NEW_RTC), &new_rtc_queue_attributes);

  /* creation of nev_val_queue */
  nev_val_queueHandle = osMessageQueueNew (2, sizeof(uint8_t), &nev_val_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RTC_DS3231_Task */
  RTC_DS3231_TaskHandle = osThreadNew(start_RTC_DS3231_Task, NULL, &RTC_DS3231_Task_attributes);

  /* creation of BPE280_Task */
  BPE280_TaskHandle = osThreadNew(start_BPE280_Task, NULL, &BPE280_Task_attributes);

  /* creation of SET_RTS_TASK */
  SET_RTS_TASKHandle = osThreadNew(start_SET_RTS_TASK, NULL, &SET_RTS_TASK_attributes);

  /* creation of UART_USB_Task */
  UART_USB_TaskHandle = osThreadNew(start_UART_USB_Task, NULL, &UART_USB_Task_attributes);

  /* creation of LCD_Task */
  LCD_TaskHandle = osThreadNew(start_LCD_Task, NULL, &LCD_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  HAL_TIM_Base_Start_IT(&htim2);		//  This TIM3 using for calculate how many time all tasks was running.
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_4_Pin KEY_3_Pin KEY_2_Pin KEY_1_Pin */
  GPIO_InitStruct.Pin = KEY_4_Pin|KEY_3_Pin|KEY_2_Pin|KEY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_RTC_DS3231_Task */
/**
* @brief Function implementing the RTC_DS3231_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_RTC_DS3231_Task */
void start_RTC_DS3231_Task(void *argument)
{
  /* USER CODE BEGIN start_RTC_DS3231_Task */
  /* Infinite loop */
	QUEUE_RTC QUEUE_RTC_t;
	QUEUE_NEW_RTC QUEUE_NEW_RTC_t;

	_RTC time; 	// rtc_queueHandle

	for(;;)
	{
		if(xQueueReceive(new_rtc_queueHandle , &QUEUE_NEW_RTC_t, 0) == pdTRUE)			// Waiting to new rts time and data
		{
			// Set new time and data
			time.Year = QUEUE_NEW_RTC_t.Year;
			time.Month = QUEUE_NEW_RTC_t.Month;
			time.Date = QUEUE_NEW_RTC_t.Date ;
			time.DaysOfWeek = QUEUE_NEW_RTC_t.DaysOfWeek;
			time.Hour = QUEUE_NEW_RTC_t.Hour;
			time.Min = QUEUE_NEW_RTC_t.Min;
			time.Sec = QUEUE_NEW_RTC_t.Sec;

			if(osMutexAcquire (I2C_MutexHandle, 1) == osOK)
			{
				DS3231_SetTime(&time);
			}
			osMutexRelease(I2C_MutexHandle);

		}
		else																			// If no new data - show current time
		{
			if(osMutexAcquire (I2C_MutexHandle, 1) == osOK)
			{
				DS3231_GetTime(&time);
			}
			osMutexRelease(I2C_MutexHandle);

			// Fill in structure of queue
			QUEUE_RTC_t.Year = time.Year;
			QUEUE_RTC_t.Month = time.Month;
			QUEUE_RTC_t.Date = time.Date;
			QUEUE_RTC_t.DaysOfWeek = time.DaysOfWeek;
			QUEUE_RTC_t.Hour = time.Hour;
			QUEUE_RTC_t.Min = time.Min;
			QUEUE_RTC_t.Sec = time.Sec;

			if(xQueueSend(rtc_queueHandle, &QUEUE_RTC_t, 0) != pdPASS)					// Send current time over queue
			{
				// ERROR
			}
			// Give semaphore
			osSemaphoreRelease(LCD_SemHandle);		// Let print time and date on start_LCD_Task
		}
		osDelay(1000);		// ПОГАНО !!!!! Використати переривання від модуля RTS як синхронізацію
		// Тобюто, якщо наідйшло переривання то зчитати дані і заповнити чергу і відіслати семафор
ііі
	}
  /* USER CODE END start_RTC_DS3231_Task */
}

/* USER CODE BEGIN Header_start_BPE280_Task */
/**
* @brief Function implementing the BPE280_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_BPE280_Task */
void start_BPE280_Task(void *argument)
{
  /* USER CODE BEGIN start_BPE280_Task */
  /* Infinite loop */
	//QUEUE_BME280 QUEUE_BME280_t;

	QUEUE_BME280 QUEUE_BME280_t;
	BMP280_HandleTypedef bmp280;
	float pressure, temperature, humidity;

	// Init BME280
	if(osMutexAcquire (I2C_MutexHandle, 1) == osOK)
	{
		bmp280_init_default_params(&bmp280.params);
		bmp280.addr = BMP280_I2C_ADDRESS_0;
		bmp280.i2c = &hi2c1;

		if(bmp280_init(&bmp280, &bmp280.params) != true)
		{
			while(1){}   								// ERROR
		}

		bool bme280p = bmp280.id == BME280_CHIP_ID;
	}
	osMutexRelease(I2C_MutexHandle);


	for(;;)
	{
		if(osMutexAcquire (I2C_MutexHandle, 1) == osOK)
		{
			if((bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) != true)
			{
				while(1){}								// ERROR
			}
			else										// IF all okay. Send data into QUEUE to the main task
			{
				// Fill in fields of struct
				QUEUE_BME280_t.temperature = temperature;
				QUEUE_BME280_t.humidity = humidity;
				QUEUE_BME280_t.pressure = pressure;

				int status_queue = xQueueSend(THPQueueHandle, &QUEUE_BME280_t, 0);		// Send data into queue
				if(status_queue != pdPASS)
				{
					// ERROR
				}
				// Give semaphore
				osSemaphoreRelease(LCD_SemHandle);		// Let print T, H and P on start_LCD_Task
			}
		}
		osMutexRelease(I2C_MutexHandle);

		osDelay(5000);
	}
  /* USER CODE END start_BPE280_Task */
}

/* USER CODE BEGIN Header_start_SET_RTS_TASK */
/**
* @brief Function implementing the SET_RTS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_SET_RTS_TASK */
void start_SET_RTS_TASK(void *argument)
{
  /* USER CODE BEGIN start_SET_RTS_TASK */
  /* Infinite loop */

	QUEUE_RTC QUEUE_RTC_t;
	QUEUE_NEW_RTC QUEUE_NEW_RTC_t;

  for(;;)
  {


	  uint16_t pressed_key, status_queue = 0;
	  status_queue = xQueueReceive( buttonQueueHandle , &pressed_key, 0 );
	  if (status_queue == pdTRUE)
	  {
		  if(pressed_key == 1)
		  {
				QUEUE_NEW_RTC_t.Year = 11;
				QUEUE_NEW_RTC_t.Month = 11;
				QUEUE_NEW_RTC_t.Date = 11;
				QUEUE_NEW_RTC_t.DaysOfWeek = 11;
				QUEUE_NEW_RTC_t.Hour = 11;
				QUEUE_NEW_RTC_t.Min = 11;
				QUEUE_NEW_RTC_t.Sec = 11;

				if(xQueueSend(new_rtc_queueHandle, &QUEUE_NEW_RTC_t, 0) != pdPASS)
				{
					// ERROR
				}

				print_first_time_on_lcd_flag = true;
		  }
	  }
	  osDelay(1000);


//		// Waiting press the any button
//			uint16_t pressed_key, status_queue = 0;
//			status_queue = xQueueReceive( buttonQueueHandle , &pressed_key, 0 );
//			if (status_queue == pdTRUE)
//			{
//				if(pressed_key == 1)
//				{
//					// 1. Stop RTC task
//					osThreadSuspend(RTC_DS3231_TaskHandle);
//					osThreadSuspend(BPE280_TaskHandle);
//
//		//			uint8_t seconds = 0;
//		//			uint8_t minutes = 0;
//		//			uint8_t hours = 0;
//
//
//					uint8_t new_year = 0;
//					uint8_t i = 0;
//
//					do{
//						if(pressed_key == 2)
//						{
//	//						if(i == 0)
//	//						{
//								new_year--;
//
//	//						}
//								pressed_key = 0;
//						}
//						if(pressed_key == 3)
//						{
//	//						if(i == 0)
//	//						{
//								new_year++;
//	//						}
//								pressed_key = 0;
//						}
//
//						// Sent queue into LCD task ()
//						if(xQueueSend(nev_val_queueHandle, &new_year, 0) != pdPASS)
//						{
//
//						}
//
//
//						if(pressed_key == 4)		// Save new data into struct
//						{
//							//UEUE_NEW_RTC_t.
//							i++;
//						}
//
//						xQueueReceive( buttonQueueHandle , &pressed_key, 0 );
//					}while(i <= 7);
//				}
//
//
//				// Send global new time and data into RTC tssk
//
//
//
//
//				osThreadResume(RTC_DS3231_TaskHandle);
//				osThreadResume(BPE280_TaskHandle);
//
//
//
//				if(pressed_key == 1)
//				{
//					HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//				}
//				else if(pressed_key == 2)
//				{
//					HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//				}
//				else if(pressed_key == 3)
//				{
//					HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//				}
//				else if(pressed_key == 4)
//				{
//					HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//				}
//				//HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//
//				// 2. Send new time qwer queue into rtc task
//
//
//			}






  }
  /* USER CODE END start_SET_RTS_TASK */
}

/* USER CODE BEGIN Header_start_UART_USB_Task */
/**
* @brief Function implementing the UART_USB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_UART_USB_Task */
void start_UART_USB_Task(void *argument)
{
  /* USER CODE BEGIN start_UART_USB_Task */
  /* Infinite loop */
  for(;;)
  {
	  char str_end_of_line[3] = {'\r','\n'};
	  char str_sig = '-';
	  char buff[10] = {0};

	  QUEUE_t msg;												// Make a queue
	  memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'
	  strcat(msg.Buf, ">>>>> Free heap memory: ");				// Add string to another (Total heap)

	  freemem = xPortGetFreeHeapSize();							// Function return how many free memory.
	  itoa(freemem, buff, 10);
	  strcat(msg.Buf, buff);
	  strcat(msg.Buf, str_end_of_line);

	  // add a hat
	  strcat(msg.Buf, "| TASK NAME  | STATUS | PRIOR | STACK | NUM |\n\r\0");

	  vTaskList(str_management_memory_str);						// Fill in str_management_memory_str array management task information

	  // Finding the  end of string
	  uint16_t buffer_size = 0;
	  while(msg.Buf[buffer_size] != '\0')
	  {
		  buffer_size ++;
	  }

	  // Add str_management_memory_str to queue string
	  int i = 0;
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
		  // add data to queue
		  msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }

	  // add a hat
	  char str_line[] = {"-----------------------\n\r"};
	  char str_head_2[] = {"| TASK NAME | ABS TIME | TASK TIME% |\n\r"};
	  strcat(msg.Buf, str_line);
	  strcat(msg.Buf, str_head_2);

	  memset(str_management_memory_str, 0, sizeof(str_management_memory_str));	// Clean buffer

	  vTaskGetRunTimeStats(str_management_memory_str);							// Function return how much time all functions running.

	  buffer_size = buffer_size + i + (sizeof(str_line)-1) + (sizeof(str_head_2)-1);
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
		  // add data to queue
		  msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }
	  strcat(msg.Buf, "#########################################\n\r");

	  buffer_size = 0;
	  while(msg.Buf[buffer_size] != '\0')
	  {
		  buffer_size ++;
	  }
	  // Transmit over virtual comport
	  HAL_UART_Transmit_IT( &huart1, msg.Buf, buffer_size);

	  osDelay(3000);
  }
  /* USER CODE END start_UART_USB_Task */
}

/* USER CODE BEGIN Header_start_LCD_Task */
/**
* @brief Function implementing the LCD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_LCD_Task */
void start_LCD_Task(void *argument)
{
  /* USER CODE BEGIN start_LCD_Task */
  /* Infinite loop */
	//QUEUE_ALL_DATA QUEUE_ALL_DATA_t; 		// Global data queue for send to LCD task (to print)


	QUEUE_BME280 QUEUE_BME280_t;
	QUEUE_RTC QUEUE_RTC_t;

	char str_hour[4] = {0};
	char str_minute[4] = {0};
	char str_msecond[4] = {0};
	char str_buf[6] = {0};
	char str_date[15] = {0};
	char str_time_buf[10] = {0};

	bool two_point = true;
//	static bool print_first_time_on_lcd_flag = true;

	ILI9341_Reset();
	ILI9341_Init();
	ILI9341_Fill_Screen(BLACK);

	osDelay(1000);

	// Draw static lines
	ILI9341_Draw_Hollow_Rectangle_Coord(0, 0, 319, 150, BLUE);

	for(;;)
	{

//		if(eet RTC mode)
//		{
//			1. Clear time and data LCD part.
//			2. show selected value.
//		}

//		uint8_t new_rtc_vvvv = 0;
//		if(xQueueReceive(nev_val_queueHandle, &new_rtc_vvvv, 0) == pdPASS)
//		{
//			// Clear LCD
//			ILI9341_Fill_Screen(BLACK);
//
//			// Show new value on LCD
//			char ssss[5] = {0};
//			sprintf(ssss, "%d", new_rtc_vvvv);
//			ILI9341_Draw_Text(ssss, 100, 10, YELLOW, 2, BLACK);
//
//
//		}


		osStatus_t status = osSemaphoreAcquire(LCD_SemHandle, 10);
		if(status == 0)
		{
		// If data from BME280 is ready print T, H and  P
		// Waiting semaphore from
//		osStatus_t status = osSemaphoreAcquire(print_THPSemHandle, 0);
//		if(status == 0)
//		{
			if((xQueueReceive(THPQueueHandle, &QUEUE_BME280_t, 0)) == pdTRUE)
			{
				// Print T, H and  P on LCD
				char str_temperature[5] = {0};
				char str_humidity[5] = {0};
				char str_preassure[10] = {0};

				//int preasure = QUEUE_BME280_t.pressure;
				sprintf(str_temperature, "%d", (int)QUEUE_BME280_t.temperature);
				char strthp_buf_t[1] = {0};
				strncat(strthp_buf_t, "T:", 2);
				strncat(strthp_buf_t, str_temperature, sizeof(str_temperature));
				strncat(strthp_buf_t, " C", 2);
				ILI9341_Draw_Text(strthp_buf_t, 10, 160, YELLOW, 2, BLACK);

				sprintf(str_humidity, "%d", (int)QUEUE_BME280_t.humidity);
				char strthp_buf_h[10] = {0};
				strncat(strthp_buf_h, "H:", 2);
				strncat(strthp_buf_h, str_humidity, sizeof(str_humidity));
				strncat(strthp_buf_h, " %", 2);
				ILI9341_Draw_Text(strthp_buf_h, 10, 180, YELLOW, 2, BLACK);

				sprintf(str_preassure, "%d", (int)QUEUE_BME280_t.pressure);
				char strthp_buf_p[17] = {0};
				strncat(strthp_buf_p, "P:", 2);
				strncat(strthp_buf_p, str_preassure, sizeof(str_preassure));
				strncat(strthp_buf_p, " mmRh", 4);
				ILI9341_Draw_Text(strthp_buf_p, 10, 200, YELLOW, 2, BLACK);
			}
//		}


		// If data from RTC  is ready print it
//		status = 99;
//		status = osSemaphoreAcquire(print_RTCSemHandle, 0);
//		if(status == 0)
//		{
			if((xQueueReceive(rtc_queueHandle, &QUEUE_RTC_t, 0) == pdPASS))
			{
				if((print_first_time_on_lcd_flag == true) )				// If print data firsttime
				{
					sprintf(str_hour, "%d", QUEUE_RTC_t.Hour);
					sprintf(str_minute, "%d", QUEUE_RTC_t.Min);
					sprintf(str_msecond, "%d", QUEUE_RTC_t.Sec);

					// Updating hours  on LCD
					if(QUEUE_RTC_t.Hour < 10)
					{
						char hour_buff[5] = {0};
						strncat(hour_buff, "0", 1);
						strncat(hour_buff, str_hour, sizeof(str_hour));
						ILI9341_Draw_Text(hour_buff, 10, 1, GREEN, 10, BLACK);
					}
					else
					{
						ILI9341_Draw_Text(str_hour, 10, 1, GREEN, 10, BLACK);
					}

					// Updating minutes on LCD
					if(QUEUE_RTC_t.Min < 10)
					{
						char min_buff[5] = {0};
						strncat(min_buff, "0", 1);
						strncat(min_buff, str_minute, sizeof(str_minute));
						ILI9341_Draw_Text(min_buff, 195, 1, GREEN, 10, BLACK);
					}
					else
					{
						ILI9341_Draw_Text(str_minute, 195, 1, GREEN, 10, BLACK);
					}

					// Updating seconds on LCD
					if(QUEUE_RTC_t.Sec == 0)
					{
						ILI9341_Draw_Text("  ", 215, 85, GREEN, 6, BLACK);
					}
					if(QUEUE_RTC_t.Sec < 10)
					{
						// add '0'
						char second_buff[5] = {0};
						strncat(second_buff, "0", 1);
						strncat(second_buff, str_msecond, sizeof(str_msecond));
						//strncat(str_time_buf, minute_buff, sizeof(minute_buff));
						ILI9341_Draw_Text(second_buff, 215, 85, GREEN, 6, BLACK);
					}
					else
					{
						ILI9341_Draw_Text(str_msecond, 215, 85, GREEN, 6, BLACK);
					}

					// Draw seconds line
					ILI9341_Draw_Rectangle(10, 81, (5*QUEUE_RTC_t.Sec), 4, GREEN);
					if(QUEUE_RTC_t.Sec == 0)
					{
						ILI9341_Draw_Rectangle(10, 81, 300, 4, BLACK);
					}

					// Updating blink two points on LCD
					if(two_point == true)
					{
						ILI9341_Draw_Text(":", 135, 1, GREEN, 10, BLACK);
						two_point = false;
					}
					else
					{
						ILI9341_Draw_Text(" ", 135, 1, GREEN, 10, BLACK);
						two_point = true;
					}

					// Date
					ILI9341_Draw_Text("        ", 10, 86, BLUE, 4, BLACK);
					memset(str_buf, 0 , sizeof(str_buf));
					sprintf(str_date, "%d", QUEUE_RTC_t.Date);

					strncat(str_date, ".", 1);
					sprintf(str_buf, "%d", QUEUE_RTC_t.Month);
					strncat(str_date, str_buf, sizeof(str_buf));
					memset(str_buf, 0 , sizeof(str_buf));
					sprintf(str_buf, "%d", QUEUE_RTC_t.Year);
					strncat(str_date, ".", 1);
					strncat(str_date, str_buf, sizeof(str_buf));
					ILI9341_Draw_Text(str_date, 10, 86, BLUE, 4, BLACK);

					ILI9341_Draw_Text("           ", 10, 120, BLUE, 2, BLACK);

					switch (QUEUE_RTC_t.DaysOfWeek)
					{
						case 1:
							ILI9341_Draw_Text("MONDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 2:
							ILI9341_Draw_Text("TUESDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 3:
							ILI9341_Draw_Text("WEDNESDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 4:
							ILI9341_Draw_Text("THURSDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 5:
							ILI9341_Draw_Text("FRIDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 6:
							ILI9341_Draw_Text("SATURDAY", 10, 120, BLUE, 2, BLACK);
							break;
						case 7:
							ILI9341_Draw_Text("SUNDAY", 10, 120, BLUE, 2, BLACK);
							break;
					}

					print_first_time_on_lcd_flag = false;

				}
				else
				{
					sprintf(str_hour, "%d", QUEUE_RTC_t.Hour);
					sprintf(str_minute, "%d", QUEUE_RTC_t.Min);
					sprintf(str_msecond, "%d", QUEUE_RTC_t.Sec);

					// Updating hours and minutes on LCD
					if(QUEUE_RTC_t.Sec == 0)
					{
						if(QUEUE_RTC_t.Min < 10)
						{
							char min_buff[5] = {0};
							strncat(min_buff, "0", 1);
							strncat(min_buff, str_minute, sizeof(str_minute));
							ILI9341_Draw_Text(min_buff, 195, 1, GREEN, 10, BLACK);
						}
						else
						{
							ILI9341_Draw_Text(str_minute, 195, 1, GREEN, 10, BLACK);
						}

						if(QUEUE_RTC_t.Hour < 10)
						{
							char hour_buff[5] = {0};
							strncat(hour_buff, "0", 1);
							strncat(hour_buff, str_hour, sizeof(str_hour));
							ILI9341_Draw_Text(hour_buff, 10, 1, GREEN, 10, BLACK);
						}
						else
						{
							ILI9341_Draw_Text(str_hour, 10, 1, GREEN, 10, BLACK);
						}
					}

					// Updating seconds on LCD
					if(QUEUE_RTC_t.Sec == 0)
					{
						ILI9341_Draw_Text("  ", 215, 85, GREEN, 6, BLACK);
					}
					if(QUEUE_RTC_t.Sec < 10)
					{
						// add '0'
						char second_buff[5] = {0};
						strncat(second_buff, "0", 1);
						strncat(second_buff, str_msecond, sizeof(str_msecond));
						ILI9341_Draw_Text(second_buff, 215, 85, GREEN, 6, BLACK);
					}
					else
					{
						ILI9341_Draw_Text(str_msecond, 215, 85, GREEN, 6, BLACK);
					}

					// Draw seconds line
					ILI9341_Draw_Rectangle(10, 81, (5*QUEUE_RTC_t.Sec), 4, GREEN);
					if(QUEUE_RTC_t.Sec == 0)
					{
						ILI9341_Draw_Rectangle(10, 81, 300, 4, BLACK);
					}

					// Updating blink two points on LCD
					if(two_point == true)
					{
						ILI9341_Draw_Text(":", 135, 1, GREEN, 10, BLACK);
						two_point = false;
					}
					else
					{
						ILI9341_Draw_Text(" ", 135, 1, GREEN, 10, BLACK);
						two_point = true;
					}

					if(QUEUE_RTC_t.Sec == 0)
					{
						// Date
						ILI9341_Draw_Text("        ", 10, 86, BLUE, 4, BLACK);
						memset(str_buf, 0 , sizeof(str_buf));
						sprintf(str_date, "%d", QUEUE_RTC_t.Date);

						strncat(str_date, ".", 1);
						sprintf(str_buf, "%d", QUEUE_RTC_t.Month);
						strncat(str_date, str_buf, sizeof(str_buf));
						memset(str_buf, 0 , sizeof(str_buf));
						sprintf(str_buf, "%d", QUEUE_RTC_t.Year);
						strncat(str_date, ".", 1);
						strncat(str_date, str_buf, sizeof(str_buf));
						ILI9341_Draw_Text(str_date, 10, 86, BLUE, 4, BLACK);

						ILI9341_Draw_Text("           ", 10, 120, BLUE, 2, BLACK);

						switch (QUEUE_RTC_t.DaysOfWeek)
						{
							case 1:
								ILI9341_Draw_Text("MONDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 2:
								ILI9341_Draw_Text("TUESDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 3:
								ILI9341_Draw_Text("WEDNESDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 4:
								ILI9341_Draw_Text("THURSDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 5:
								ILI9341_Draw_Text("FRIDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 6:
								ILI9341_Draw_Text("SATURDAY", 10, 120, BLUE, 2, BLACK);
								break;
							case 7:
								ILI9341_Draw_Text("SUNDAY", 10, 120, BLUE, 2, BLACK);
								break;
						}

					}

				}
			}
		}   // semaphore



	}
  /* USER CODE END start_LCD_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	// Timer for measure how many time task was running.
	if(htim->Instance == TIM2)
	{
		ulHighFreqebcyTimerTicks++;					// Update time tasks counter
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
