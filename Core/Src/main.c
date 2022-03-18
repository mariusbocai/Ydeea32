/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *	  //ADXL345: R: 0xA7 W: 0xA6
  *  //L3g4200D: R: 0xD3 W: 0xD2
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"
#include "attitude_calc.h"
#include "pid.h"
#include "output.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	short PitchAngle;
	short RollAngle;
	short RawYawRate;
}usefulData_t;
typedef struct {
	usefulData_t Data;
	unsigned char FrameNumber;
	unsigned char StopBuffer;
}telemetryStruct_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_HAL_UART_REGISTER_CALLBACKS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

osThreadId defaultTaskHandle;
osThreadId testTask10msHandle;
uint32_t testTask10msBuffer[ 128 ];
osStaticThreadDef_t testTask10msControlBlock;
osThreadId IdleTaskHandle;
osThreadId SensorInputTaskHandle;
osThreadId AttitudeCalcHandle;
uint32_t AttitudeCalcBuffer[ 3000 ];
osStaticThreadDef_t AttitudeCalcControlBlock;
osThreadId telemetryTaskHandle;
osThreadId OutputTaskHandle;
osThreadId rcInputTaskHandle;
/* USER CODE BEGIN PV */
uint8_t firstRun;
uint32_t tickValOn, tickValOff, sensorInputTime, telemetryTime, rcInputTime;
/*===Telemetry Variables===*/
telemetryStruct_t telemetryData;
/*=== Structure where to put the read data from the gyro ===*/
gyro_input_struct_t GYRO_RAW_INPUT_STRUCT;
acc_input_struct_t ACC_RAW_INPUT_STRUCT;
//mag_input_struct_t MAG_RAW_INPUT_STRUCT;

unsigned char rcDataReceived, syncIsRunning;
unsigned char rcDataBuffer[40];

extern unsigned char Red, Green, Blue, Orange;

const unsigned char GyroResetVal = 0x80;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void const * argument);
void Main_10ms(void const * argument);
void IdleTask_func(void const * argument);
void SensorInputTask_func(void const * argument);
void AttitudeCalc_func(void const * argument);
void telemetryTask_func(void const * argument);
void OutputTask_Main(void const * argument);
void rcInputTask_func(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
pUART_CallbackTypeDef rcCompleteCbk(void)
{
	/*well, do something!!!! :)*/;
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
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  firstRun = 1;
  HAL_UART_RegisterCallback(&huart5, HAL_UART_RX_COMPLETE_CB_ID, *rcCompleteCbk);
  //(void)HAL_I2C_Mem_Write(&hi2c1, 0xA6, 0x2D, 1, &AccResetVal, 1, 100);
  /*ReBOOT the gyroscope at startup to clear previous initializations*/
  (void)HAL_I2C_Mem_Write(&hi2c1, 0xD2, 0x24, 1, &GyroResetVal, 1, 100);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of testTask10ms */
  osThreadStaticDef(testTask10ms, Main_10ms, osPriorityNormal, 0, 128, testTask10msBuffer, &testTask10msControlBlock);
  testTask10msHandle = osThreadCreate(osThread(testTask10ms), NULL);

  /* definition and creation of IdleTask */
  osThreadDef(IdleTask, IdleTask_func, osPriorityIdle, 0, 128);
  IdleTaskHandle = osThreadCreate(osThread(IdleTask), NULL);

  /* definition and creation of SensorInputTask */
  osThreadDef(SensorInputTask, SensorInputTask_func, osPriorityHigh, 0, 3000);
  SensorInputTaskHandle = osThreadCreate(osThread(SensorInputTask), NULL);

  /* definition and creation of AttitudeCalc */
  osThreadStaticDef(AttitudeCalc, AttitudeCalc_func, osPriorityAboveNormal, 0, 3000, AttitudeCalcBuffer, &AttitudeCalcControlBlock);
  AttitudeCalcHandle = osThreadCreate(osThread(AttitudeCalc), NULL);

  /* definition and creation of telemetryTask */
  osThreadDef(telemetryTask, telemetryTask_func, osPriorityLow, 0, 512);
  telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);

  /* definition and creation of OutputTask */
  osThreadDef(OutputTask, OutputTask_Main, osPriorityAboveNormal, 0, 2400);
  OutputTaskHandle = osThreadCreate(osThread(OutputTask), NULL);

  /* definition and creation of rcInputTask */
  osThreadDef(rcInputTask, rcInputTask_func, osPriorityRealtime, 0, 128);
  rcInputTaskHandle = osThreadCreate(osThread(rcInputTask), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart4, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	static unsigned char LoopCounter = 0;
	static unsigned char Red1, Green1, Blue1, Orange1;
  /* Infinite loop */
  for(;;)
  {
	  /*Check the LED*/
	  if(LoopCounter == 0)
	  {
		  tickValOn = xTaskGetTickCount();
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		  Red1 = Red;
		  Green1 = Green;
		  Blue1 = Blue;
		  Orange1 = Orange;
	  }
	  if(Red1 <= LoopCounter)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	  }
	  if(Green1 <= LoopCounter)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  if(Blue1 <= LoopCounter)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  }
	  if(Orange1 <= LoopCounter)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	  }
	  if (LoopCounter==9)
	  {
		  LoopCounter = 0;
	  }
	  else
	  {
		  LoopCounter++;
	  }
	  osDelayUntil(&tickValOn, 1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Main_10ms */
/**
* @brief Function implementing the testTask10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Main_10ms */
void Main_10ms(void const * argument)
{
  /* USER CODE BEGIN Main_10ms */
  /* Infinite loop */
  for(;;)
  {
	  if(firstRun == 1)
	  {
		  firstRun = 0;
		  Init_PID = 1;
		  tickValOff = xTaskGetTickCount(); /*Initialize the tick counter value for task scheduling*/
		  osDelayUntil(&tickValOff,500); /*wait for 500ms until we call this task to switch off the LED*/
	  }
	  else
	  {
		  /*Switch OFF LED*/
		  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		  /*set next task activation time*/
		  osDelayUntil(&tickValOff, 1000);
	  }
  }
  /* USER CODE END Main_10ms */
}

/* USER CODE BEGIN Header_IdleTask_func */
/**
* @brief Function implementing the IdleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IdleTask_func */
void IdleTask_func(void const * argument)
{
  /* USER CODE BEGIN IdleTask_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10000);
  }
  /* USER CODE END IdleTask_func */
}

/* USER CODE BEGIN Header_SensorInputTask_func */
/**
* @brief Function implementing the SensorInputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorInputTask_func */
void SensorInputTask_func(void const * argument)
{
  /* USER CODE BEGIN SensorInputTask_func */
	unsigned char sensorInput, retVal, whoAmIAcc, whoAmIGyro, retValGyro;
	static unsigned char sensorInitState = 0;

	whoAmIAcc = 0xFF;
	whoAmIGyro = 0xFF;
  /* Infinite loop */
  for(;;)
  {
	  calcStateMachine();
	  /*Check if the correct sensors are present on IMU board*/
	  if(sensorInitState == 0)
	  {
		  GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = 0;
		  GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH = 0;
		  GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = 0;
		  GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH = 0;
		  GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = 0;
		  GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH = 0;
		  retVal = HAL_I2C_Mem_Read(&hi2c1, 0xA7, 0x0, 1, &sensorInput, 1, 100);
		  if (retVal == 0)
		  {
			whoAmIAcc = sensorInput;
		  }

		  retVal = HAL_I2C_Mem_Read(&hi2c1, 0xD3, 0x0F, 1, &sensorInput, 1, 100);
		  if (retVal == 0)
		  {
			  whoAmIGyro = sensorInput;
		  }

		  if ((whoAmIAcc == 0xE5) && (whoAmIGyro == 0xD3))
		  {
			  unsigned char ACC_INIT_VAL = 0x8;
			  unsigned char ACC_FIFO_INIT_VAL = 0xC0;
			  unsigned char GYRO_INIT_VAL[5] = {0x4F, 0x21, 0x0, 0x0, 0x0};
			  unsigned char GYRO_FIFO_INIT_VAL = 0x0;

			  sensorInitState = 1;
			  /*Initialize the sensors*/
			  /*ACC init routine*/
			  retVal = HAL_I2C_Mem_Write(&hi2c1, 0xA6, 0x2D, 1, &ACC_INIT_VAL, 1, 100);
			  if(retVal == 0)
			  {
				  retVal = HAL_I2C_Mem_Write(&hi2c1, 0xA6, 0x38, 1, &ACC_FIFO_INIT_VAL, 1, 100);
			  }
			  /*GYRO init routine*/
			  retValGyro = HAL_I2C_Mem_Write(&hi2c1, 0xD2, 0xA0, 1, &GYRO_INIT_VAL, 5, 100);
			  if(retValGyro == 0)
			  {
				  retValGyro = HAL_I2C_Mem_Write(&hi2c1, 0xD2, 0xAE, 1, &GYRO_FIFO_INIT_VAL, 1, 100);
			  }

			  if((retValGyro == 0)&&(retVal == 0))
			  {
				  /*Correct sensors found, init complete, program can continue*/
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			  }
			  /*Save the timestamp for future task scheduling*/
			  sensorInputTime = xTaskGetTickCount();
		  }
		  else
		  {
			  /*At least one sensor was not the expected one, so no-go*/
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		  }
	  }
	  else
	  {
		  /*Read Raw sensor data*/
		  taskENTER_CRITICAL();
		  retVal = HAL_I2C_Mem_Read(&hi2c1, 0xA7, 0x32, 1, (unsigned char *)&ACC_RAW_INPUT_STRUCT, 6, 100);
		  retVal = HAL_I2C_Mem_Read(&hi2c1, 0xD3, 0xA6, 1, (unsigned char *)&GYRO_RAW_INPUT_STRUCT, 8, 100);
		  taskEXIT_CRITICAL();
		  gyroCalculateData();
		  accCalculateData();
		  /*Unblock the Attitude processing task*/
		  //osSignalSet(AttitudeCalcHandle, 1);
		  /*TODO: add SD card memory handling - save data in an array so the telemetry task can write it to SD later*/
		  osThreadResume(AttitudeCalcHandle);
	  }
	  osDelayUntil(&sensorInputTime, 5);
  }
  /* USER CODE END SensorInputTask_func */
}

/* USER CODE BEGIN Header_AttitudeCalc_func */
/**
* @brief Function implementing the AttitudeCalc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AttitudeCalc_func */
void AttitudeCalc_func(void const * argument)
{
  /* USER CODE BEGIN AttitudeCalc_func */
//	uint32_t ulNotifiedValue;
  /* Infinite loop */
  for(;;)
  {
//	  xTaskNotifyWait( 0x00, /* Don't clear any notification bits on entry. */
//			  	  	  ULONG_MAX, /* Reset the notification value to 0 on exit. */
//					  &ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
//					  portMAX_DELAY ); /* Block indefinitely. */
	  //callCounter++;
	  calcActualAngle();
	  //osSignalWait(0,0);
	  //osDelay(1000);
	  /*Unblock the Output task to begin output calculations*/
	  osThreadResume(OutputTaskHandle);
	  osThreadSuspend(AttitudeCalcHandle);
  }
  /* USER CODE END AttitudeCalc_func */
}

/* USER CODE BEGIN Header_telemetryTask_func */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_telemetryTask_func */
void telemetryTask_func(void const * argument)
{
  /* USER CODE BEGIN telemetryTask_func */
  telemetryData.StopBuffer = 0xAA;
  telemetryData.FrameNumber = 0;
  /* Infinite loop */
  for(;;)
  {
    /*osDelay(1);*/
    telemetryTime = xTaskGetTickCount();
    /*Arrange serial buffer to be transmitted to ESP32*/
    telemetryData.Data.PitchAngle = (short)(actualPitchAngle*10);
    telemetryData.Data.RollAngle = (short)(actualRollAngle*10);
    telemetryData.Data.RawYawRate = (short)(actualRawYawRate*10);
    telemetryData.FrameNumber++;
    /*Start data transmission*/
    HAL_UART_Transmit(&huart4, (uint8_t *)&telemetryData, sizeof(telemetryData), 1);
    /*Re-activate task each 100ms*/
    osDelayUntil(&telemetryTime, 100);
  }
  /* USER CODE END telemetryTask_func */
}

/* USER CODE BEGIN Header_OutputTask_Main */
/**
* @brief Function implementing the OutputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputTask_Main */
void OutputTask_Main(void const * argument)
{
  /* USER CODE BEGIN OutputTask_Main */
  /* Infinite loop */
  for(;;)
  {
	PID_Main();
	PWM_Main();
	osThreadSuspend(OutputTaskHandle);
  }
  /* USER CODE END OutputTask_Main */
}

/* USER CODE BEGIN Header_rcInputTask_func */
/**
* @brief Function implementing the rcInputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rcInputTask_func */
void rcInputTask_func(void const * argument)
{
  /* USER CODE BEGIN rcInputTask_func */
	//rcDataBuffer
	static unsigned char noRcDataCounter;
	noRcDataCounter = 0;
	syncIsRunning = 1;
	/*Enable reception interrupt*/
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  /* Infinite loop */
  for(;;)
  {
	  rcInputTime = xTaskGetTickCount();
	  if(syncIsRunning == 1)
	  {
		  if(rcDataReceived == 0)
		  {
			  if(++noRcDataCounter == 3)/*wait 3 ms to be sure no Rx messages are in*/
			  {
				  noRcDataCounter = 0;
				  syncIsRunning = 0;
			  }
		  }
		  else
		  {
			  rcDataReceived = 0;/*do nothing, just wait for sync to finish*/;
		  }

	  }
	  if(syncIsRunning == 0)
	  {
		  HAL_UART_Receive_IT(&huart5, rcDataBuffer, 0x20);
	  }
	  osDelayUntil(&rcInputTime, 1);
  }
  /* USER CODE END rcInputTask_func */
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
