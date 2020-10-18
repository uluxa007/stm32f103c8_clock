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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myLCD.h"
#include "myALARM.h"
#include "song.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	UCLOCK_NO_COMMAND=0,
	UCLOCK_SET_TIME,
	UCLOCK_SET_DATE,
	UCLOCK_SET_ALARM,
	UCLOCK_SET_TEXT
}myClockCommand;
int gggg=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int song1[][2]={	{E2,400},{F2,400},{G2,400},{B2,400},{A2,400},{G2,400},{F2,400},{E2,400},{D2D,400},{E2,400},{F2,400},{B1,400},{G2,400},{F2,400},{E2,800},
									{E2,400},{F2,400},{G2,400},{B2,400},{A2,400},{G2,400},{F2,400},{E2,400},{D2D,400},{E2,400},{F2,400},{B1,400},{G2,400},{F2,400},{E2,800},
									{0,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{0,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,400},
									{0,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{F2,200},{E2,400},
									{0,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{0,200},{E2,200},{E2,200},{E2,200},{E2,200},{C2,200},{C2,400},
									{0,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{D2D,200},{E2,200},{E2,200},{E2,200},{E2,200},{E2,200},{F2,200},{E2,400}};
										
	
int song2[][2]={{E1,L8},{E1,L8},{E1,L4},{E1,L8},{B1,L4},{B1,L8},{G1,L4*4},
								{E1,L8},{E1,L8},{E1,L4},{E1,L4},{F1D,L4},{G1,L4},{G1,L4},{F1D,L4*2},
								{E1,L8},{E1,L8},{E1,L4},{E1,L8},{B1,L4},{B1,L8},{G1,L4*4},
								{E1,L8},{E1,L8},{E1,L4},{E1,L4},{F1D,L4},{G1,L4},{G1,L4},{F1D,L4},
								{E1,L4},{F1D,L4},{G1,L4},{G1,L8},{A1,L4},{A1,L8},{G1,L4*3},{G1,L8},
								{G1,L8},{G1,L4},{G1,L4},{G1,L4},{A1,L4},{G1,L4},{F1D,L4},{E1,L4},
								{F1D,L4},{G1,L4},{G1,L4},{F1D,L4},{F1D,L4},{E1,L4*3},{E1,L8},{E1,L8},
								{E1,L4},{E1,L4},{E1,L4},{F1D,L4},{E1,L4},{D1D,L4},{C1D,L4},{D1D,L4},{E1,L2}};

int song3[][2]={	{G2,L4},{C2,L4},{D2D,L8},{F2,L8},{G2,L4},{C2,L4},{D2D,L8},{F2,L8},{G2,L4},{C2,L4},{D2D,L8},{F2,L8},{G2,L4},{C2,L4},{D2D,L8},{F2,L8},{G2,L4},
								{C2,L4},{E2,L8},{F2,L8},{G2,L4},{C2,L4},{E2,L8},{F2,L8},{G2,L4},{C2,L4},{E2,L8},{F2,L8},{G2,L4},{C2,L4},{E2,L8},{F2,L8},
								{G1,L4*3},{C1,L4*3},{D1D,L8},{F1,L8},{G1,L2},{C1,L2},{D1D,L8},{F1,L8},{D1,L4},
								{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},{F1,L4*3},
								{A0D,L4*3},{D1D,L8},{D1,L8},{F1,L2},{A0D,L2},{D1D,L8},{D1,L8},{C1,L4},
								{F0,L4},{G0D,L8},{A0D,L8},{C1,L4},{F0,L4},{G0D,L8},{A0D,L8},{C1,L4},{F0,L4},{G0D,L8},{A0D,L8},{C1,L4},{F0,L4},{G0D,L8},{A0D,L8},{G1,L4*3},
								{C1,L4*3},{D1D,L8},{F1,L8},{G1,L2},{C1,L2},{D1D,L8},{F1,L8},{D1,L4},
								{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},{D1,L4},{G0,L4},{A0D,L8},{C1,L8},
								{G2,L4*3},{C2,L4*3},{D2D,L8},{F2,L8},{G2,L2},{C2,L2},{D2D,L8},{F2,L8},{D2,L4},
								{G1,L4},{A1D,L8},{C2,L8},{D2,L4},{G1,L4},{A1D,L8},{C2,L8},{D2,L4},{G1,L4},{A1D,L8},{C2,L8},{D2,L4},{G1,L4},{A1D,L8},{C2,L8}};
									
int song4[][2]={	{G0,L2},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},
								{F0D,L4/3},{F0D,L4/3},{F0D,L4/3},{F0D,L4},
								{G0,L2},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},
								{F0D,L4/3},{F0D,L4/3},{F0D,L4/3},{F0D,L4},
								{G0,L2},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},
								{A0,L4/3},{A0,L4/3},{A0,L4/3},{A0,L4},
								{G0,L2},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},
								{A0,L4/3},{A0,L4/3},{A0,L4/3},{A0,L4},
								{G0,L2},{G0,L2},{G0,L2},{D0D,L2*6/8},{A0D,L8},{G0,L2},{D0D,L2*7/8},{A0D,L8},{G0,L4},{G0,L4/3},{G0,L4/3},{G0,L4/3},
								{A0,L4/3},{A0,L4/3},{A0,L4/3},{A0,L4},
								{D1,L2},{D1,L2},{D1,L2},{D1D,L2*6/8},{A0D,L8},{F0D,L4}};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myUART_task */
osThreadId_t myUART_taskHandle;
const osThreadAttr_t myUART_task_attributes = {
  .name = "myUART_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myLCD_task */
osThreadId_t myLCD_taskHandle;
const osThreadAttr_t myLCD_task_attributes = {
  .name = "myLCD_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for mySetTime_task */
osThreadId_t mySetTime_taskHandle;
const osThreadAttr_t mySetTime_task_attributes = {
  .name = "mySetTime_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for mySetDate_task */
osThreadId_t mySetDate_taskHandle;
const osThreadAttr_t mySetDate_task_attributes = {
  .name = "mySetDate_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myAlarm_task */
osThreadId_t myAlarm_taskHandle;
const osThreadAttr_t myAlarm_task_attributes = {
  .name = "myAlarm_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for mySetAlarm_task */
osThreadId_t mySetAlarm_taskHandle;
const osThreadAttr_t mySetAlarm_task_attributes = {
  .name = "mySetAlarm_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myRing_task */
osThreadId_t myRing_taskHandle;
const osThreadAttr_t myRing_task_attributes = {
  .name = "myRing_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myButtons_task */
osThreadId_t myButtons_taskHandle;
const osThreadAttr_t myButtons_task_attributes = {
  .name = "myButtons_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for setTimeQueue */
osMessageQueueId_t setTimeQueueHandle;
const osMessageQueueAttr_t setTimeQueue_attributes = {
  .name = "setTimeQueue"
};
/* Definitions for setDateQueue */
osMessageQueueId_t setDateQueueHandle;
const osMessageQueueAttr_t setDateQueue_attributes = {
  .name = "setDateQueue"
};
/* Definitions for setAlarmQueue */
osMessageQueueId_t setAlarmQueueHandle;
const osMessageQueueAttr_t setAlarmQueue_attributes = {
  .name = "setAlarmQueue"
};
/* Definitions for setRingQueue */
osMessageQueueId_t setRingQueueHandle;
const osMessageQueueAttr_t setRingQueue_attributes = {
  .name = "setRingQueue"
};
/* Definitions for myLCD_Mutex */
osMutexId_t myLCD_MutexHandle;
const osMutexAttr_t myLCD_Mutex_attributes = {
  .name = "myLCD_Mutex"
};
/* Definitions for myRTC_Mutex */
osMutexId_t myRTC_MutexHandle;
const osMutexAttr_t myRTC_Mutex_attributes = {
  .name = "myRTC_Mutex"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartUART_task(void *argument);
void StartLCD_task(void *argument);
void StartSetTime_task(void *argument);
void StartSetDate_task(void *argument);
void StartAlarm_task(void *argument);
void StartSetAlarm_task(void *argument);
void StartRing_task(void *argument);
void StartButtons_task(void *argument);

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
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//myLCD_Initialize();
	myLCD_Initialize();
	
	//ST7920_Clear();
	//ST7920_SendString(0,0, "HELLO WORLD");
  //ST7920_SendString(1,0, "FROM");
  //ST7920_SendString(2,0, "CONTROLLERSTECH");
  //ST7920_SendString(3,0, "1234567890!@#$%^");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myLCD_Mutex */
  myLCD_MutexHandle = osMutexNew(&myLCD_Mutex_attributes);

  /* creation of myRTC_Mutex */
  myRTC_MutexHandle = osMutexNew(&myRTC_Mutex_attributes);

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
  /* creation of setTimeQueue */
  setTimeQueueHandle = osMessageQueueNew (3, sizeof(RTC_TimeTypeDef), &setTimeQueue_attributes);

  /* creation of setDateQueue */
  setDateQueueHandle = osMessageQueueNew (3, sizeof(RTC_DateTypeDef), &setDateQueue_attributes);

  /* creation of setAlarmQueue */
  setAlarmQueueHandle = osMessageQueueNew (3, sizeof(ALARM_TypeDef), &setAlarmQueue_attributes);

  /* creation of setRingQueue */
  setRingQueueHandle = osMessageQueueNew (3, sizeof(uint8_t), &setRingQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myUART_task */
  myUART_taskHandle = osThreadNew(StartUART_task, NULL, &myUART_task_attributes);

  /* creation of myLCD_task */
  myLCD_taskHandle = osThreadNew(StartLCD_task, NULL, &myLCD_task_attributes);

  /* creation of mySetTime_task */
  mySetTime_taskHandle = osThreadNew(StartSetTime_task, NULL, &mySetTime_task_attributes);

  /* creation of mySetDate_task */
  mySetDate_taskHandle = osThreadNew(StartSetDate_task, NULL, &mySetDate_task_attributes);

  /* creation of myAlarm_task */
  myAlarm_taskHandle = osThreadNew(StartAlarm_task, NULL, &myAlarm_task_attributes);

  /* creation of mySetAlarm_task */
  mySetAlarm_taskHandle = osThreadNew(StartSetAlarm_task, NULL, &mySetAlarm_task_attributes);

  /* creation of myRing_task */
  myRing_taskHandle = osThreadNew(StartRing_task, NULL, &myRing_task_attributes);

  /* creation of myButtons_task */
  myButtons_taskHandle = osThreadNew(StartButtons_task, NULL, &myButtons_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  //RTC_TimeTypeDef sTime = {0};
  //RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  //sTime.Hours = 0x0;
  //sTime.Minutes = 0x0;
  //sTime.Seconds = 0x0;
	//
  //if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  //{
  //  Error_Handler();
  //}
  //DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  //DateToUpdate.Month = RTC_MONTH_JANUARY;
  //DateToUpdate.Date = 0x1;
  //DateToUpdate.Year = 0x0;
	//
  //if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  //{
  //  Error_Handler();
  //}
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3032;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartUART_task */
/**
* @brief Function implementing the myUART_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_task */
void StartUART_task(void *argument)
{
  /* USER CODE BEGIN StartUART_task */
	uint8_t command =0;
	uint8_t time_mass[3]={0};
	uint8_t date_mass[4]={0};
	uint8_t alarm_mass[6];
	
	HAL_StatusTypeDef status;
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	ALARM_TypeDef alarm;
	
	
	//time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  //time.StoreOperation = RTC_STOREOPERATION_RESET; 
	
	char* trans_date = "Transmit";
  /* Infinite loop */
  for(;;)
  {
		
		HAL_UART_Receive(&huart1, &command, 1,200);
		
		if(command)
		{
			switch(command)
			{
				case UCLOCK_SET_TEXT:
				{
					break;
				}
				case UCLOCK_SET_TIME:
				{
					status = HAL_UART_Receive(&huart1, time_mass, 3,1000);
					command=UCLOCK_NO_COMMAND;
					if(status !=HAL_OK)
					{
						osMutexWait(myLCD_MutexHandle,portMAX_DELAY);
						myLCD_ClearLCDScreen();
						myLCD_PrintStr("Error get TIME");
						myLCD_Cursor(1,0);
						myLCD_PrintStr("from uart");
						osDelay(1000);
						myLCD_ClearLCDScreen();
						osMutexRelease(myLCD_MutexHandle);
						break;
					}
					//-----------
					time.Hours=time_mass[0];
					time.Minutes = time_mass[1];
					time.Seconds = time_mass[2];
					
					osMessageQueuePut(setTimeQueueHandle,&time,0U,200);
					osDelay(200);
					
					HAL_UART_Transmit(&huart1,(uint8_t*)trans_date,sizeof(trans_date),2000);
					
					break;
				}
				case UCLOCK_SET_DATE:
				{
					status = HAL_UART_Receive(&huart1, date_mass, 4,1000);
					command=UCLOCK_NO_COMMAND;
					if(status !=HAL_OK)
					{
						osMutexWait(myLCD_MutexHandle,portMAX_DELAY);
						myLCD_ClearLCDScreen();
						myLCD_PrintStr("Error get DATE");
						myLCD_Cursor(1,0);
						myLCD_PrintStr("from uart");
						osDelay(1000);
						myLCD_ClearLCDScreen();
						osMutexRelease(myLCD_MutexHandle);
						break;
					}
					date.Date = date_mass[0];
					date.Month = date_mass[1];
					date.Year = date_mass[2];
					date.WeekDay = date_mass[3];
					
					osMessageQueuePut(setDateQueueHandle,&date,0U,200);
					osDelay(20);
					break;
					
				}
				case UCLOCK_SET_ALARM:
				{
					status = HAL_UART_Receive(&huart1, alarm_mass, 6,1000);
					command=UCLOCK_NO_COMMAND;
					if(status !=HAL_OK)
					{
						osMutexWait(myLCD_MutexHandle,portMAX_DELAY);
						myLCD_ClearLCDScreen();
						myLCD_PrintStr("Error get ALARM");
						myLCD_Cursor(1,0);
						myLCD_PrintStr("from uart");
						osDelay(1000);
						myLCD_ClearLCDScreen();
						osMutexRelease(myLCD_MutexHandle);
						break;
					}
					
					alarm.day = alarm_mass[0];
					alarm.month = alarm_mass[1];
					alarm.year = alarm_mass[2];
					alarm.hour = alarm_mass[3];
					alarm.minute = alarm_mass[4];
					alarm.alarm_type = alarm_mass[5];
					
					osMessageQueuePut(setAlarmQueueHandle,&alarm,0U,200);
					osDelay(20);
					
					break;
				}
			}
		}
		
  }
  /* USER CODE END StartUART_task */
}

/* USER CODE BEGIN Header_StartLCD_task */
/**
* @brief Function implementing the myLCD_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD_task */
void StartLCD_task(void *argument)
{
  /* USER CODE BEGIN StartLCD_task */
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	uint32_t tickcount = osKernelGetTickCount();
	myLCD_ClearLCDScreen();
	
  /* Infinite loop */
  for(;;)
  {
		osMutexWait(myLCD_MutexHandle,portMAX_DELAY);
		myLCD_Cursor(0,0);
		HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
		
		myLCD_PrintInt_clock(time.Hours);
		myLCD_PrintStr(":");
		myLCD_PrintInt_clock(time.Minutes);
		myLCD_PrintStr(":");
		myLCD_PrintInt_clock(time.Seconds);
		
		HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
		myLCD_Cursor(1,0);
		myLCD_PrintInt_clock(date.Date);
		myLCD_PrintStr(".");
		myLCD_PrintInt_clock(date.Month);
		myLCD_PrintStr(".");
		myLCD_PrintInt_clock(date.Year);
		myLCD_PrintStr(" ");
		myLCD_PrintWeekDay(date.WeekDay);
		osMutexRelease(myLCD_MutexHandle);
		
		tickcount+=1000;
    osDelayUntil(tickcount);
  }
  /* USER CODE END StartLCD_task */
}

/* USER CODE BEGIN Header_StartSetTime_task */
/**
* @brief Function implementing the mySetTime_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetTime_task */
void StartSetTime_task(void *argument)
{
  /* USER CODE BEGIN StartSetTime_task */
	RTC_TimeTypeDef time;
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
		status = osMessageQueueGet(setTimeQueueHandle,&time,NULL,osWaitForever);
		if(status != osOK)
		{
			myLCD_ClearLCDScreen();
			myLCD_PrintStr("Error time queue");
		}
		else{
		HAL_RTC_SetTime(&hrtc,&time,RTC_FORMAT_BIN);
		}
    osDelay(1000);

  }
  /* USER CODE END StartSetTime_task */
}

/* USER CODE BEGIN Header_StartSetDate_task */
/**
* @brief Function implementing the mySetDate_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetDate_task */
void StartSetDate_task(void *argument)
{
  /* USER CODE BEGIN StartSetDate_task */
	RTC_DateTypeDef date;
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(setDateQueueHandle,&date,NULL,osWaitForever);
		if(status != osOK)
		{
			myLCD_ClearLCDScreen();
			myLCD_PrintStr("Error date queue");
		}
		else{
		HAL_RTC_SetDate(&hrtc,&date,RTC_FORMAT_BIN);
		}
    osDelay(1000);
  }
  /* USER CODE END StartSetDate_task */
}

/* USER CODE BEGIN Header_StartAlarm_task */
/**
* @brief Function implementing the myAlarm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarm_task */
void StartAlarm_task(void *argument)
{
  /* USER CODE BEGIN StartAlarm_task */
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;
	uint8_t ring_command;
  /* Infinite loop */
  for(;;)
  {
		HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
		osDelay(10);
		HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
		osDelay(10); 
		for(int i=0;i<Alarms.alarms_count;i++)
		{
			if(	Alarms.alarms_data[i].year == date.Year
				& Alarms.alarms_data[i].month == date.Month
				& Alarms.alarms_data[i].day == date.Date
				||
				Alarms.alarms_data[i].week_day == date.WeekDay)
			{
				
				if(Alarms.alarms_data[i].hour == time.Hours
				&  Alarms.alarms_data[i].minute == time.Minutes)
				{
					switch(Alarms.alarms_data[i].alarm_type)
					{
						case ALARM_NO_REPEAT:
						{
							for(int j=i;j<Alarms.alarms_count-1;j++)
							{
								Alarms.alarms_data[j]=Alarms.alarms_data[j+1];
							}
							i--;
							Alarms.alarms_count--;
							break;
						}
						case ALARM_EVERY_DAY:
						{
							//30 days in month
							if(date.Month == 4
							|| date.Month == 6
							|| date.Month == 9
							|| date.Month == 11)
							{
								if(date.Date==30)
								{
									Alarms.alarms_data[i].month++;
									Alarms.alarms_data[i].day =1;
								}
								else
								{
									Alarms.alarms_data[i].day++;
								}
							}//31 days in month
							else if(date.Month == 1
									 || date.Month == 3
									 || date.Month == 5
									 || date.Month == 7
									 || date.Month == 8
									 || date.Month == 10)
							{
								if(date.Date==31)
								{
									Alarms.alarms_data[i].month++;
									Alarms.alarms_data[i].day =1;
								}
								else
								{
									Alarms.alarms_data[i].day++;
								}
							}
							else if(date.Month == 12)
							{
								if(date.Date==31)
								{
									Alarms.alarms_data[i].month = 1;
									Alarms.alarms_data[i].day =1;
									Alarms.alarms_data[i].year++;
								}
								else
								{
									Alarms.alarms_data[i].day++;
								}
							}
							else if(date.Month==2)
							{
								if(date.Year%4==0)
								{
									if(date.Date==29)
									{
										Alarms.alarms_data[i].month++;
										Alarms.alarms_data[i].day =1;
									}
									else
									{
										Alarms.alarms_data[i].day++;
									}
								}
								else
								{
									if(date.Date==28)
									{
										Alarms.alarms_data[i].month++;
										Alarms.alarms_data[i].day =1;
									}
									else
									{
										Alarms.alarms_data[i].day++;
									}
								}
							}
						}
						case ALARM_EVERY_WEEK:
						{
							
							break;
						}
						case ALARM_EVERY_MONTH:
						{
							if(Alarms.alarms_data[i].month==12) Alarms.alarms_data[i].month =1;
							else Alarms.alarms_data[i].month++;
							break;
						}
					}
					
					ring_command =1;
					osMessageQueuePut(setRingQueueHandle,&ring_command,0U,200);
					
					osMutexWait(myLCD_MutexHandle,portMAX_DELAY);
					myLCD_ClearLCDScreen();
					myLCD_Cursor(0,0);
					myLCD_PrintStr("ALARMA DETECTED");
					myLCD_Cursor(1,0);
					myLCD_PrintInt(Alarms.alarms_count);
					osDelay(3000);
					myLCD_ClearLCDScreen();
					osMutexRelease(myLCD_MutexHandle);
				}
			}
		}
    osDelay(3000);
  }
  /* USER CODE END StartAlarm_task */
}

/* USER CODE BEGIN Header_StartSetAlarm_task */
/**
* @brief Function implementing the mySetAlarm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetAlarm_task */
void StartSetAlarm_task(void *argument)
{
  /* USER CODE BEGIN StartSetAlarm_task */
	ALARM_TypeDef alarm;
  osStatus_t status;
	osMessageQueueReset(setAlarmQueueHandle);
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(setAlarmQueueHandle,&alarm,NULL,osWaitForever);
		if(status != osOK)
		{
			myLCD_ClearLCDScreen();
			myLCD_PrintStr("Error alarm queue");
		}
		else{
			
			if(Alarms.alarms_count < ALARM_ALARMS_MAX)
			{
				Alarms.alarms_data[Alarms.alarms_count]=alarm;
				Alarms.alarms_count++;
			}
			else
			{
				for(int i=0;i<Alarms.alarms_count;i++)
				{
					if(Alarms.alarms_data[i].alarm_type==ALARM_NOT_SET)
					{
						Alarms.alarms_data[i] = alarm;
						break;
					}
				}
			}
		}
    osDelay(1000);
  }
  /* USER CODE END StartSetAlarm_task */
}

/* USER CODE BEGIN Header_StartRing_task */
/**
* @brief Function implementing the myRing_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRing_task */
void StartRing_task(void *argument)
{
  /* USER CODE BEGIN StartRing_task */
	uint8_t ring_command=1;
	SystemCoreClockUpdate();
	uint8_t divider = TIM3->PSC+1;
	uint8_t randomik=0;
	
	TIM_OC_InitTypeDef sConfigOC = {0};
  /* Infinite loop */
  for(int j=0;j<1;j++)
	{
		for(int i=0;i<sizeof(song2)/8;i++)
			{
				osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,0U);
				if(ring_command==0)break;
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
				osDelay(30);
				htim3.Init.Period = SystemCoreClock/(song2[i][0]*divider);
				HAL_TIM_PWM_Init(&htim3);
				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = (SystemCoreClock)/(song2[i][0]*divider*256);
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				osDelay(song2[i][1]-30);
			}
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	}
	
  for(;;)
  {
		osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,osWaitForever);
		if(ring_command>0)
		{
			if(randomik>=3)randomik=0;
			else randomik++;
			
			switch(randomik)
			{
				case 0:
				{
					for(int i=0;i<sizeof(song1)/(8);i++)
					{
						osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,0U);
						if(ring_command==0)break;
						HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
						osDelay(10);
						htim3.Init.Period = SystemCoreClock/(song1[i][0]*divider);
						HAL_TIM_PWM_Init(&htim3);
						sConfigOC.OCMode = TIM_OCMODE_PWM1;
						sConfigOC.Pulse = (SystemCoreClock)/(song1[i][0]*divider*256);
						sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
						sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						osDelay(song1[i][1]-10);
					}
					break;
				}
				case 1:
				{
					for(int i=0;i<sizeof(song2)/(8);i++)
					{
						osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,0U);
						if(ring_command==0)break;
						HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
						osDelay(10);
						htim3.Init.Period = SystemCoreClock/(song2[i][0]*divider);
						HAL_TIM_PWM_Init(&htim3);
						sConfigOC.OCMode = TIM_OCMODE_PWM1;
						sConfigOC.Pulse = (SystemCoreClock)/(song2[i][0]*divider*256);
						sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
						sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						osDelay(song2[i][1]-10);
					}
					break;
				}
				case 2:
				{
					for(int i=0;i<sizeof(song3)/(8);i++)
					{
						osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,0U);
						if(ring_command==0)break;
						HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
						osDelay(10);
						htim3.Init.Period = SystemCoreClock/(song3[i][0]*divider);
						HAL_TIM_PWM_Init(&htim3);
						sConfigOC.OCMode = TIM_OCMODE_PWM1;
						sConfigOC.Pulse = (SystemCoreClock)/(song3[i][0]*divider*256);
						sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
						sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						osDelay(song3[i][1]-10);
					}
					break;
				}
				case 3:
				{
					for(int i=0;i<sizeof(song4)/(8);i++)
					{
						osMessageQueueGet(setRingQueueHandle,&ring_command,NULL,0U);
						if(ring_command==0)break;
						HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
						osDelay(10);
						htim3.Init.Period = SystemCoreClock/(song4[i][0]*divider);
						HAL_TIM_PWM_Init(&htim3);
						sConfigOC.OCMode = TIM_OCMODE_PWM1;
						sConfigOC.Pulse = (SystemCoreClock)/(song4[i][0]*divider*256);
						sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
						sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						osDelay(song4[i][1]-10);
					}
					break;
				}
			}
			
			

		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		}
    osDelay(1);
  }
  /* USER CODE END StartRing_task */
}

/* USER CODE BEGIN Header_StartButtons_task */
/**
* @brief Function implementing the myButtons_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtons_task */
void StartButtons_task(void *argument)
{
  /* USER CODE BEGIN StartButtons_task */
	uint8_t ring_command,hui;
  /* Infinite loop */
  for(;;)
  {
		hui = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
		if(hui==1)
		{
			ring_command =0;
			osMessageQueuePut(setRingQueueHandle,&ring_command,0U,200);
		}
    osDelay(10);
  }
  /* USER CODE END StartButtons_task */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
