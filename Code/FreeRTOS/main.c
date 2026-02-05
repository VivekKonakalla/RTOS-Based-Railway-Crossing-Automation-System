/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ULTRASONIC_MIN_CM 2.0f
#define ULTRASONIC_MAX_CM 15.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorMonitor1 */
osThreadId_t SensorMonitor1Handle;
const osThreadAttr_t SensorMonitor1_attributes = {
  .name = "SensorMonitor1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SensorMonitor2 */
osThreadId_t SensorMonitor2Handle;
const osThreadAttr_t SensorMonitor2_attributes = {
  .name = "SensorMonitor2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ServoControl1 */
osThreadId_t ServoControl1Handle;
const osThreadAttr_t ServoControl1_attributes = {
  .name = "ServoControl1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ServoControl2 */
osThreadId_t ServoControl2Handle;
const osThreadAttr_t ServoControl2_attributes = {
  .name = "ServoControl2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LEDBuzzer */
osThreadId_t LEDBuzzerHandle;
const osThreadAttr_t LEDBuzzer_attributes = {
  .name = "LEDBuzzer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTPrint */
osThreadId_t UARTPrintHandle;
const osThreadAttr_t UARTPrint_attributes = {
  .name = "UARTPrint",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for printQueue */
osMessageQueueId_t printQueueHandle;
const osMessageQueueAttr_t printQueue_attributes = {
  .name = "printQueue"
};
/* Definitions for stateMutex */
osMutexId_t stateMutexHandle;
const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex"
};
/* Definitions for printMutex */
osMutexId_t printMutexHandle;
const osMutexAttr_t printMutex_attributes = {
  .name = "printMutex"
};
/* Definitions for ultrasonicSemGate1 */
osSemaphoreId_t ultrasonicSemGate1Handle;
const osSemaphoreAttr_t ultrasonicSemGate1_attributes = {
  .name = "ultrasonicSemGate1"
};
/* Definitions for ultrasonicSemGate2 */
osSemaphoreId_t ultrasonicSemGate2Handle;
const osSemaphoreAttr_t ultrasonicSemGate2_attributes = {
  .name = "ultrasonicSemGate2"
};
/* USER CODE BEGIN PV */
osEventFlagsId_t closeEvent;
osEventFlagsId_t openEvent;

volatile uint32_t ic_val1_gate1 = 0, ic_val2_gate1 = 0;
volatile uint8_t is_first_capture_gate1 = 0;
volatile uint32_t echo_ticks_gate1 = 0;
float distance_cm_gate1 = 0.0f;

volatile uint32_t ic_val1_gate2 = 0, ic_val2_gate2 = 0;
volatile uint8_t is_first_capture_gate2 = 0;
volatile uint32_t echo_ticks_gate2 = 0;
float distance_cm_gate2 = 0.0f;

float tick_us_pwm;

volatile uint8_t sweeping_down = 0;
volatile uint8_t gates_closed = 0;
volatile uint8_t gates_closed_count = 0;
volatile uint8_t train_passing = 0;
volatile float gate1_current_angle = 180.0f;
volatile float gate2_current_angle = 180.0f;

volatile uint32_t gate1_pause_end = 0;
volatile uint32_t gate2_pause_end = 0;
volatile uint32_t passing_end = 0;
volatile uint32_t open_delay_end = 0;
volatile uint32_t yellow_blink_end = 0;
volatile uint8_t system_state = 0; // 0=idle, 1=warning, 2=closing, 3=passing, 4=opening

uint32_t gates_opened_count = 0;  // Add this in your globals section
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartSensorMonitor1Task(void *argument);
void StartSensorMonitor2Task(void *argument);
void StartServoControl1Task(void *argument);
void StartServoControl2Task(void *argument);
void StartLEDBuzzerTask(void *argument);
void StartUARTPrintTask(void *argument);

/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
void HCSR04_Trigger_Gate1(void);
void HCSR04_Trigger_Gate2(void);
uint32_t Angle_to_CCR(float angle_deg);
float Check_Ultrasonic_Gate1(void);
float Check_Ultrasonic_Gate2(void);
void BlinkRedAndBuzzBoth(void);
void TurnOffAllFeedback(void);
void SetServoGate1(float angle);
void SetServoGate2(float angle);
void Print(const char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

void HCSR04_Trigger_Gate1(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void HCSR04_Trigger_Gate2(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

uint32_t Angle_to_CCR(float angle_deg)
{
  float pulse_us = 1000.0f + (angle_deg / 180.0f) * 1000.0f;
  return (uint32_t)(pulse_us / tick_us_pwm);
}

float Get_Ultrasonic_Distance_Gate1(void)
{
  HCSR04_Trigger_Gate1();
  vTaskDelay(pdMS_TO_TICKS(60));
  return distance_cm_gate1;
}

float Get_Ultrasonic_Distance_Gate2(void)
{
  HCSR04_Trigger_Gate2();
  vTaskDelay(pdMS_TO_TICKS(60));
  return distance_cm_gate2;
}

void BlinkRedAndBuzzBoth(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

void TurnOffAllFeedback(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}

void SetServoGate1(float angle)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_to_CCR(angle));
}

void SetServoGate2(float angle)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Angle_to_CCR(angle));
}

void Print(const char *format, ...)
{
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  osMutexAcquire(printMutexHandle, osWaitForever);
  char *msg = pvPortMalloc(strlen(buffer) + 1);
  if (msg != NULL)
  {
    strcpy(msg, buffer);
    osMessageQueuePut(printQueueHandle, &msg, 0, osWaitForever);
  }
  osMutexRelease(printMutexHandle);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  uint32_t timer_clk_freq = HAL_RCC_GetPCLK2Freq();
  tick_us_pwm = (htim1.Init.Prescaler + 1) * 1.0f / (timer_clk_freq / 1000000.0f);

  SetServoGate1(180.0f);
  SetServoGate2(180.0f);

  TurnOffAllFeedback();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);

  Print("\r\nRTOS System Ready – Gates Open (180°)\r\n");

  closeEvent = osEventFlagsNew(NULL);
  openEvent = osEventFlagsNew(NULL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of stateMutex */
  stateMutexHandle = osMutexNew(&stateMutex_attributes);

  /* creation of printMutex */
  printMutexHandle = osMutexNew(&printMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ultrasonicSemGate1 */
  ultrasonicSemGate1Handle = osSemaphoreNew(1, 0, &ultrasonicSemGate1_attributes);

  /* creation of ultrasonicSemGate2 */
  ultrasonicSemGate2Handle = osSemaphoreNew(1, 0, &ultrasonicSemGate2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of printQueue */
  printQueueHandle = osMessageQueueNew (20, sizeof(char*), &printQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorMonitor1 */
  SensorMonitor1Handle = osThreadNew(StartSensorMonitor1Task, NULL, &SensorMonitor1_attributes);

  /* creation of SensorMonitor2 */
  SensorMonitor2Handle = osThreadNew(StartSensorMonitor2Task, NULL, &SensorMonitor2_attributes);

  /* creation of ServoControl1 */
  ServoControl1Handle = osThreadNew(StartServoControl1Task, NULL, &ServoControl1_attributes);

  /* creation of ServoControl2 */
  ServoControl2Handle = osThreadNew(StartServoControl2Task, NULL, &ServoControl2_attributes);

  /* creation of LEDBuzzer */
  LEDBuzzerHandle = osThreadNew(StartLEDBuzzerTask, NULL, &LEDBuzzer_attributes);

  /* creation of UARTPrint */
  UARTPrintHandle = osThreadNew(StartUARTPrintTask, NULL, &UARTPrint_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 639;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 639;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB2 LD3_Pin LD2_Pin
                           PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) // Gate 1
  {
    if (!is_first_capture_gate1)
    {
      ic_val1_gate1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
      is_first_capture_gate1 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      ic_val2_gate1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

      if (ic_val2_gate1 > ic_val1_gate1)
        echo_ticks_gate1 = ic_val2_gate1 - ic_val1_gate1;
      else
        echo_ticks_gate1 = (0xFFFF - ic_val1_gate1) + ic_val2_gate1 + 1;

      distance_cm_gate1 = (echo_ticks_gate1 * 0.0343f * 3.125f) / 2.0f;

      is_first_capture_gate1 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
  else if (htim->Instance == TIM4) // Gate 2
  {
    if (!is_first_capture_gate2)
    {
      ic_val1_gate2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      is_first_capture_gate2 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      ic_val2_gate2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

      if (ic_val2_gate2 > ic_val1_gate2)
        echo_ticks_gate2 = ic_val2_gate2 - ic_val1_gate2;
      else
        echo_ticks_gate2 = (0xFFFF - ic_val1_gate2) + ic_val2_gate2 + 1;

      distance_cm_gate2 = (echo_ticks_gate2 * 0.0343f * 3.125f) / 2.0f;

      is_first_capture_gate2 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}
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
	/* USER CODE BEGIN StartDefaultTask */
	for(;;)
	{
	  vTaskDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}
/* USER CODE BEGIN Header_StartSensorMonitor1Task */
/**
* @brief Function implementing the SensorMonitor1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorMonitor1Task */
void StartSensorMonitor1Task(void *argument)
{
	/* USER CODE BEGIN StartSensorMonitor1Task */
	uint8_t last_ir1 = GPIO_PIN_SET;
	for(;;)
	{
	  uint8_t ir1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  if (ir1 == GPIO_PIN_RESET && last_ir1 == GPIO_PIN_SET)
	  {
	    osMutexAcquire(stateMutexHandle, osWaitForever);
	    if (system_state == 0)
	    {
	      Print("IR Gate 1 Detected - Train Coming - Closing Both Gates\r\n");
	      system_state = 1; // Warning state
	      osEventFlagsSet(closeEvent, 0x01);
	    }
	    osMutexRelease(stateMutexHandle);
	    vTaskDelay(pdMS_TO_TICKS(200));
	  }
	  last_ir1 = ir1;
	  vTaskDelay(pdMS_TO_TICKS(50));
	}
	/* USER CODE END StartSensorMonitor1Task */
}

/* USER CODE BEGIN Header_StartSensorMonitor2Task */
/**
* @brief Function implementing the SensorMonitor2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorMonitor2Task */
void StartSensorMonitor2Task(void *argument)
{
	/* USER CODE BEGIN StartSensorMonitor2Task */
	uint8_t last_ir2 = GPIO_PIN_SET;
	for(;;)
	{
	  uint8_t ir2 = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
	  if (ir2 == GPIO_PIN_RESET && last_ir2 == GPIO_PIN_SET)
	  {
	    osMutexAcquire(stateMutexHandle, osWaitForever);
	    if (system_state == 3) // Passing state
	    {
	      Print("IR Gate 2 Detected - Train Exited - Opening Both Gates\r\n");
	      system_state = 4;
	      osEventFlagsSet(openEvent, 0x01);
	    }
	    osMutexRelease(stateMutexHandle);
	    vTaskDelay(pdMS_TO_TICKS(200));
	  }
	  last_ir2 = ir2;
	  vTaskDelay(pdMS_TO_TICKS(50));
	}
	/* USER CODE END StartSensorMonitor2Task */
}

/* USER CODE BEGIN Header_StartServoControl1Task */
/**
* @brief Function implementing the ServoControl1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControl1Task */
void StartServoControl1Task(void *argument)
{
	/* USER CODE BEGIN StartServoControl1Task */
	for (;;)
	{
	  /* Wait for close command */
	  osEventFlagsWait(closeEvent, 0x01, osFlagsWaitAny, osWaitForever);
	  osEventFlagsClear(closeEvent, 0x01);

	  /* NEW: Both gates now wait 2 seconds for warning period (yellow lights assumed handled elsewhere or by hardware) */
	  vTaskDelay(pdMS_TO_TICKS(2000));

	  /* Yellow blinking removed from here - no more blocking delay unique to Gate 1 */

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  sweeping_down = 1;
	  system_state = 2;
	  osMutexRelease(stateMutexHandle);

	  gate1_current_angle = 180.0f;
	  gate1_pause_end = HAL_GetTick();

	  while (gate1_current_angle > 0.0f)
	  {
	    float dist = Get_Ultrasonic_Distance_Gate1();
	    if (dist >= ULTRASONIC_MIN_CM && dist <= ULTRASONIC_MAX_CM && HAL_GetTick() >= gate1_pause_end)
	    {
	      Print("Gate 1 Ultrasonic Detected at %.1f cm - Pausing at 90°, Wait 1s\r\n", dist);
	      SetServoGate1(90.0f);
	      gate1_pause_end = HAL_GetTick() + 1000;
	    }

	    if (HAL_GetTick() >= gate1_pause_end)
	    {
	      gate1_current_angle -= 10.0f;
	      if (gate1_current_angle < 0.0f) gate1_current_angle = 0.0f;
	      SetServoGate1(gate1_current_angle);
	      Print("Gate 1 Angle: %.1f°\r\n", gate1_current_angle);
	    }

	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  Print("Gate 1 Closed\r\n");

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  gates_closed_count++;
	  if (gates_closed_count == 2)
	  {
	    Print("Both Gates Closed (0°) - Train Passing - 3s Delay\r\n");
	    passing_end = HAL_GetTick() + 3000;
	    gates_closed = 1;
	    system_state = 3;
	    gates_closed_count = 0;
	  }
	  osMutexRelease(stateMutexHandle);

	  /* Wait for open command */
	  osEventFlagsWait(openEvent, 0x01, osFlagsWaitAny, osWaitForever);
	  osEventFlagsClear(openEvent, 0x01);

	  open_delay_end = HAL_GetTick() + 1000;
	  while (HAL_GetTick() < open_delay_end)
	  {
	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  sweeping_down = 0;
	  system_state = 4;
	  osMutexRelease(stateMutexHandle);

	  gate1_current_angle = 0.0f;
	  while (gate1_current_angle < 180.0f)
	  {
	    gate1_current_angle += 10.0f;
	    if (gate1_current_angle > 180.0f) gate1_current_angle = 180.0f;
	    SetServoGate1(gate1_current_angle);
	    Print("Gate 1 Angle: %.1f°\r\n", gate1_current_angle);
	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  Print("Gate 1 Open\r\n");

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  gates_opened_count++;
	  if (gates_opened_count == 2)
	  {
	    sweeping_down = 0;
	    gates_closed = 0;
	    train_passing = 0;
	    system_state = 0;
	    gates_opened_count = 0;
	    Print("Both Gates Open (180°) - Safe State\r\n");
	  }
	  osMutexRelease(stateMutexHandle);
	}
	/* USER CODE END StartServoControl1Task */
}

/* USER CODE BEGIN Header_StartServoControl2Task */
/**
* @brief Function implementing the ServoControl2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControl2Task */
void StartServoControl2Task(void *argument)
{
	/* USER CODE BEGIN StartServoControl2Task */
	for (;;)
	{
	  osEventFlagsWait(closeEvent, 0x01, osFlagsWaitAny, osWaitForever);
	  osEventFlagsClear(closeEvent, 0x01);

	  /* NEW: Add same 2-second delay as Gate 1 for perfect closing sync */
	  vTaskDelay(pdMS_TO_TICKS(2000));

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  sweeping_down = 1;
	  system_state = 2;
	  osMutexRelease(stateMutexHandle);

	  gate2_current_angle = 180.0f;
	  gate2_pause_end = HAL_GetTick();

	  while (gate2_current_angle > 0.0f)
	  {
	    float dist = Get_Ultrasonic_Distance_Gate2();
	    if (dist >= ULTRASONIC_MIN_CM && dist <= ULTRASONIC_MAX_CM && HAL_GetTick() >= gate2_pause_end)
	    {
	      Print("Gate 2 Ultrasonic Detected at %.1f cm - Pausing at 90°, Wait 1s\r\n", dist);
	      SetServoGate2(90.0f);
	      gate2_pause_end = HAL_GetTick() + 1000;
	    }

	    if (HAL_GetTick() >= gate2_pause_end)
	    {
	      gate2_current_angle -= 10.0f;
	      if (gate2_current_angle < 0.0f) gate2_current_angle = 0.0f;
	      SetServoGate2(gate2_current_angle);
	      Print("Gate 2 Angle: %.1f°\r\n", gate2_current_angle);
	    }

	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  Print("Gate 2 Closed\r\n");

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  gates_closed_count++;
	  if (gates_closed_count == 2)
	  {
	    Print("Both Gates Closed (0°) - Train Passing - 3s Delay\r\n");
	    passing_end = HAL_GetTick() + 3000;
	    gates_closed = 1;
	    system_state = 3;
	    gates_closed_count = 0;
	  }
	  osMutexRelease(stateMutexHandle);

	  osEventFlagsWait(openEvent, 0x01, osFlagsWaitAny, osWaitForever);
	  osEventFlagsClear(openEvent, 0x01);

	  open_delay_end = HAL_GetTick() + 1000;
	  while (HAL_GetTick() < open_delay_end)
	  {
	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  sweeping_down = 0;
	  system_state = 4;
	  osMutexRelease(stateMutexHandle);

	  gate2_current_angle = 0.0f;
	  while (gate2_current_angle < 180.0f)
	  {
	    gate2_current_angle += 10.0f;
	    if (gate2_current_angle > 180.0f) gate2_current_angle = 180.0f;
	    SetServoGate2(gate2_current_angle);
	    Print("Gate 2 Angle: %.1f°\r\n", gate2_current_angle);
	    vTaskDelay(pdMS_TO_TICKS(100));
	  }

	  Print("Gate 2 Open\r\n");

	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  gates_opened_count++;
	  if (gates_opened_count == 2)
	  {
	    sweeping_down = 0;
	    gates_closed = 0;
	    train_passing = 0;
	    system_state = 0;
	    gates_opened_count = 0;
	    Print("Both Gates Open (180°) - Safe State\r\n");
	  }
	  osMutexRelease(stateMutexHandle);
	}
	/* USER CODE END StartServoControl2Task */
}

/* USER CODE BEGIN Header_StartLEDBuzzerTask */
/**
* @brief Function implementing the LEDBuzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDBuzzerTask */
void StartLEDBuzzerTask(void *argument)
{
	/* USER CODE BEGIN StartLEDBuzzerTask */
	for(;;)
	{
	  osMutexAcquire(stateMutexHandle, osWaitForever);
	  uint8_t active = (system_state == 2 || system_state == 4); // Closing or opening
	  uint8_t passing = (system_state == 3);
	  osMutexRelease(stateMutexHandle);

	  if (passing)
	  {
	    BlinkRedAndBuzzBoth(); // Steady on during passing
	  }
	  else if (active)
	  {
	    BlinkRedAndBuzzBoth();
	    vTaskDelay(pdMS_TO_TICKS(500));
	    TurnOffAllFeedback();
	    vTaskDelay(pdMS_TO_TICKS(500));
	  }
	  else
	  {
	    TurnOffAllFeedback();
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
	  }

	  vTaskDelay(pdMS_TO_TICKS(50));
	}
	/* USER CODE END StartLEDBuzzerTask */
}

/* USER CODE BEGIN Header_StartUARTPrintTask */
/**
* @brief Function implementing the UARTPrint thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTPrintTask */
void StartUARTPrintTask(void *argument)
{
	/* USER CODE BEGIN StartUARTPrintTask */
	char *msg;
	for(;;)
	{
	  if (osMessageQueueGet(printQueueHandle, &msg, NULL, osWaitForever) == osOK)
	  {
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	    vPortFree(msg);
	  }
	}
	/* USER CODE END StartUARTPrintTask */
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
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
