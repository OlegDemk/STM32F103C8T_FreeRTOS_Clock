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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MAIN_TASK */
osThreadId_t MAIN_TASKHandle;
const osThreadAttr_t MAIN_TASK_attributes = {
  .name = "MAIN_TASK",
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
/* Definitions for Buttons_Task */
osThreadId_t Buttons_TaskHandle;
const osThreadAttr_t Buttons_Task_attributes = {
  .name = "Buttons_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 256 * 4,
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
/* USER CODE BEGIN PV */


// -------------------------------------------------------------------------
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == KEY_1_Pin) || (GPIO_Pin == KEY_2_Pin) || (GPIO_Pin == KEY_3_Pin) || (GPIO_Pin == KEY_4_Pin))
	{

		//HAL_TIM_Base_Start_IT(&htim3);



//		HAL_TIM_Base_Start(&htim1);
		//osTimerStart (button_timerHandle, 1000);

		//xTaskNotify(Buttons_TaskHandle, 0, eNoAction);

		// Make delay

//		xTaskNotifyFromISR(Buttons_TaskHandle, 0, eNoAction, 0);
	}
}
// -------------------------------------------------------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void start_RTC_DS3231_Task(void *argument);
void start_BPE280_Task(void *argument);
void start_MAIN_TASK(void *argument);
void start_UART_USB_Task(void *argument);
void start_Buttons_Task(void *argument);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Init LCD
  ILI9341_SPI_Init();
  ILI9341_Init();
  ILI9341_Enable();
  ILI9341_Fill_Screen(BLACK);

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
  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (2, sizeof(QUEUE_t), &UARTQueue_attributes);

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

  /* creation of MAIN_TASK */
  MAIN_TASKHandle = osThreadNew(start_MAIN_TASK, NULL, &MAIN_TASK_attributes);

  /* creation of UART_USB_Task */
  UART_USB_TaskHandle = osThreadNew(start_UART_USB_Task, NULL, &UART_USB_Task_attributes);

  /* creation of Buttons_Task */
  Buttons_TaskHandle = osThreadNew(start_Buttons_Task, NULL, &Buttons_Task_attributes);

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
	 ILI9341_Fill_Screen(BLUE);
//	 HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	 	 osDelay(500);
	 	  ILI9341_Fill_Screen(YELLOW);
	 	 osDelay(500);

	 	  // Test Clock dc3231
	 	  _RTC time;
	 	  DS3231_GetTime(&time);




   // osDelay(1000);
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
  for(;;)
  {
    osDelay(1);
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
	BMP280_HandleTypedef bmp280;
	float pressure, temperature, humidity;

	// Init BME280
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
	if(bmp280_init(&bmp280, &bmp280.params) != true)
	{
		while(1){}   			// ERROR
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;

	for(;;)
	{
		if((bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) != true)
		{
			while(1) {}			// ERROR
		}
		else
		{
			// Send data into QUEU to the main task <<<<<<<<<<<<<<<<<<<<<<<<<<
		}
		osDelay(2000);
	}
  /* USER CODE END start_BPE280_Task */
}

/* USER CODE BEGIN Header_start_MAIN_TASK */
/**
* @brief Function implementing the MAIN_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_MAIN_TASK */
void start_MAIN_TASK(void *argument)
{
  /* USER CODE BEGIN start_MAIN_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_MAIN_TASK */
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

/* USER CODE BEGIN Header_start_Buttons_Task */
/**
* @brief Function implementing the Buttons_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_Buttons_Task */
void start_Buttons_Task(void *argument)
{
  /* USER CODE BEGIN start_Buttons_Task */
  /* Infinite loop */

	BaseType_t status;
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END start_Buttons_Task */
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
  for(;;)
  {
    osDelay(1);
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
