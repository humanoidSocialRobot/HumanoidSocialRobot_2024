/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pca9685.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t  counter=0;
volatile uint32_t counter2=0;

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	counter  = __HAL_TIM_GET_COUNTER(&htim3);
//	counter2 = __HAL_TIM_GET_COUNTER(&htim4);
//}
//void HAL_TIM_CaptureCallback(TIM_HandleTypeDef *htim){
////	counter = __HAL_TIM_GET_COUNTER(&htim3);
////	counter2 = __HAL_TIM_GET_COUNTER(&htim4);
//	__NOP();
//}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//		counter = __HAL_TIM_GET_COUNTER(&htim3);
//		counter2 = __HAL_TIM_GET_COUNTER(&htim4);
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  PCA9685_Init(&hi2c1);


////Motor A الكت�? اليمين جوة
//TIM5->CCR1=65.9;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//// MotorA' الكن�? الشمال جوة
//TIM5->CCR2=81.4;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
////Motor D
//TIM5->CCR3=6.85;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
////MotorD'
//TIM9->CCR1=73.8;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

////Motor C
//TIM1->CCR1=4.29;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

////Motor C'
//TIM5->CCR2=94.3;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//HAL_Delay(5000);

////Motor B
//TIM1->CCR3= 0;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

////Motor B'
//TIM1->CCR3= 0;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//fingers
//Right hand
//PCA9685_SetServoAngle(1,  10);  //pinky

//  PCA9685_SetServoAngle(1,  140);
//  HAL_Delay(1000);
//  PCA9685_SetServoAngle(1,  10);  //Pinky


// In this example, the duty cycle is (off time - on time) / 4096 = (2048 - 0) / 4096 = 50%
 //PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	counter = TIM3->CNT;
//	counter2 =  TIM4->CNT;
	  /*
// pinkey Right hand (1)
PCA9685_SetServoAngle(1,  140);
//Ring Right hand (2)
PCA9685_SetServoAngle(0,  140);
//middle Right hand (3)
PCA9685_SetServoAngle(2,  140);
//index Right hand (4)
PCA9685_SetServoAngle(3,  170);
//thumb Right hand (5)
PCA9685_SetServoAngle(4,  15);

// pinkey Left hand (1')
PCA9685_SetServoAngle(5,  0);
//Ring Left hand (2')
PCA9685_SetServoAngle(7,  0);
//middle Left hand (3')
PCA9685_SetServoAngle(6,  180);
//index Left hand (4')
PCA9685_SetServoAngle(8,  0);
//thumb Left hand (5')
PCA9685_SetServoAngle(9,  180);
HAL_Delay(3000);

// pinkey Right hand (1)
PCA9685_SetServoAngle(1,  10);
//Ring Right hand (2)
PCA9685_SetServoAngle(0,  0);
//middle Right hand (3)
PCA9685_SetServoAngle(2,  0);
//index Right hand (4)
PCA9685_SetServoAngle(3,  75);
//thumb Right hand (5)
PCA9685_SetServoAngle(4,  75);

// pinkey Left hand (1')
PCA9685_SetServoAngle(5,  180);
//Ring Left hand (2')
PCA9685_SetServoAngle(7,  180);
//middle Left hand (3')
PCA9685_SetServoAngle(6,  0);
//index Left hand (4')
PCA9685_SetServoAngle(8,  180);
//thumb Left hand (5')
PCA9685_SetServoAngle(9,  90);
HAL_Delay(3000);
*/
//// D rest right
//PCA9685_SetServoAngle(10,  10);
//// D'
//PCA9685_SetServoAngle(11,  150);
//// C elko3 right
//PCA9685_SetServoAngle(12,  150);
//// C'
//PCA9685_SetServoAngle(13,  150);
//// B arm right
////PCA9685_SetServoAngle(14,  90);
//// B'
//PCA9685_SetServoAngle(15,  50);
//HAL_Delay(5000);
//// D rest right
//PCA9685_SetServoAngle(10,  80);
//// D'
//PCA9685_SetServoAngle(11,  30);
//// C elko3 right
//PCA9685_SetServoAngle(12,  55);
//// C'
//PCA9685_SetServoAngle(13,  40);
//// B arm right
////PCA9685_SetServoAngle(14,  90);
//// B'
//PCA9685_SetServoAngle(15,  80);
//HAL_Delay(5000);


////Motor A الكت�? اليمين جوة
//TIM5->CCR1=45.2;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
////MotorA' الكت�? اللشمال جوة
//TIM5->CCR2=40;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//HAL_Delay(5000);
////Motor A الكت�? اليمين جوة
//TIM5->CCR1=86.5;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
////MotorA' ا
//TIM5->CCR2=130;          //112.3;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//HAL_Delay(5000);

//
//	  TIM5->CCR3=2;
//	  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//	  HAL_Delay(2000);
//	  TIM5->CCR3=100;
//      HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//	  TIM5->CCR3=120;
//	  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//	  HAL_Delay(2000);

//      //Rotating head Right&Left
//	  TIM9->CCR2=30;
//	  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
//	  HAL_Delay(2000);
//	  TIM9->CCR2=100;
//	  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
//	  HAL_Delay(2000);







//Motor D
//TIM5->CCR3=32.4;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//HAL_Delay(5000);
//TIM5->CCR3=68.5;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//HAL_Delay(5000);
//
////MotorD'
//TIM9->CCR1=4.29;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//HAL_Delay(5000);
//TIM5->CCR1=75;
//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//HAL_Delay(5000);

////Motor C
//TIM1->CCR1=4.29;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//HAL_Delay(5000);
//TIM1->CCR1=75;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//HAL_Delay(5000);

////Motor C'
//TIM1->CCR2=83.9;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//HAL_Delay(5000);
//TIM1->CCR2=104.6;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//HAL_Delay(5000);


////Motor B
//TIM1->CCR3= 50;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//HAL_Delay(5000);
//TIM1->CCR3=80;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//HAL_Delay(5000);
//
//
////Motor B'
//
//TIM1->CCR3= 50;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//HAL_Delay(5000);
//TIM1->CCR3=80;
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//HAL_Delay(5000);




HAL_GPIO_WritePin(GPIOC, 13, 1); //enable R
HAL_GPIO_WritePin(GPIOC, 14, 1); //enable L
HAL_GPIO_WritePin(GPIOC, 15, 0); //is left
HAL_GPIO_WritePin(GPIOB, 5, 0);  //is Right
TIM1->CCR1=40;    //PWM right  freq 100 ARR 255
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);







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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
//  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1440-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1440-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MotorR_ER_Pin|MotorR_EL_Pin|MotorR_isL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MotorL_isL_Pin|MotorL_isR_Pin|MotorL_EL_Pin|MotorL_ER_Pin
                          |MotorR_isR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MotorR_ER_Pin MotorR_EL_Pin MotorR_isL_Pin */
  GPIO_InitStruct.Pin = MotorR_ER_Pin|MotorR_EL_Pin|MotorR_isL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorL_isL_Pin MotorL_isR_Pin MotorL_EL_Pin MotorL_ER_Pin
                           MotorR_isR_Pin */
  GPIO_InitStruct.Pin = MotorL_isL_Pin|MotorL_isR_Pin|MotorL_EL_Pin|MotorL_ER_Pin
                          |MotorR_isR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//funtion written by salma&yara

void SERV0_Angle(uint32_t Timer_number, uint32_t Channel, uint32_t  angle , float start )
{
	float Duty_Cycle;
	Duty_Cycle= start;
	switch(Timer_number){
	case 1:{
		switch(Channel){
		case 1:
			{
				TIM1->CCR1=Duty_Cycle;
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			}
		case 2:
			{
				TIM1->CCR2= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			}
		case 3:
			{
				TIM1->CCR3= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			}
		case 4:
			{
				TIM1->CCR4= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			}
		}
		}
	case 3:{
		switch(Channel){
		case 1:
			{
				TIM3->CCR1= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			}
		case 2:
			{
				TIM3->CCR2= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			}
		case 3:
			{
				TIM3->CCR3= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			}
		}
	}
	case 5:{
		switch(Channel){
		case 1:
			{
				TIM5->CCR1= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
			}
		case 2:
			{
				TIM5->CCR2= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
			}
		case 3:
			{
				TIM5->CCR3= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
			}
		}
		}
	case 9:{
		switch(Channel){
		case 2:
			{
				TIM9->CCR2= Duty_Cycle;
				HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
			}
		}
		}

	}
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
