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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "MPU6050.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE);
void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE);
void MPU6050_Calibrate(void);
void MPU6050_GetAllData(int16_t *Data);
void MPU6050_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t isinitialized = 0;
volatile uint64_t uiTicksCNT = 0;
struct tsMPU6050_Data {
	int aRoll;
	int aPitch;
	int aYaw;
} MPU6050_Data;
int fGX_Cal;
int fGY_Cal;
int fGZ_Cal;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  int flag = 0;
	  int roll_1 = 0;
	  int pitch_1 = 0;
	  int yaw_1 = 0;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_TIM_Base_Start(&htim1);
  MPU6050_Init();
  MPU6050_Calibrate();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  flag = 0;
	  char msg[31];
	  uint16_t len = sprintf(msg, "Roll:%i  Pitch:%i Yaw:%i\r\n", (int16_t)MPU6050_Data.aRoll, (int16_t)MPU6050_Data.aPitch, (int16_t)MPU6050_Data.aYaw);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, len, HAL_MAX_DELAY);
	  HAL_Delay(100);
	 int roll = (int16_t)MPU6050_Data.aRoll;
	 int pitch = (int16_t)MPU6050_Data.aPitch;
	 int yaw = (int16_t)MPU6050_Data.aYaw;
	 if ((roll_1 - roll > 20) || (roll - roll_1 > 20)) {
		 flag = 1;
		 //roll_1 = roll;
	 }
	 else if ((pitch_1 - pitch > 20) || (pitch - pitch_1 > 20)) {
	 		 flag = 1;
	 		 //pitch_1 = pitch;
	 	 }
	 else if ((yaw_1 - yaw > 20) || (yaw - yaw_1 > 20)) {
	 	 		 flag = 1;
	 	 		 //yaw_1 = yaw;
	 	 	 }
	 roll_1 = roll;
	 pitch_1 = pitch;
	 yaw_1 = yaw;
	 if (flag == 1) {
		// isinitialized = 0;
		  /*случайное число из таймера*/
		  int count = __HAL_TIM_GET_COUNTER(&htim1);
		  srand(count);
		  /* генерируем пять случайных целых чисел из отрезка [1;4] */
		  int string_num = 1 + rand()%(4 - 1 + 1);
		  char str1[10] = "Yes";
		  char str2[10] = "No";
		  char str3[15] = "Try Again";
		  char str4[10] = "Maybe";
		  uint8_t y = 0;
		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(2, y);
		  if (string_num == 1) {
			  ssd1306_WriteString(str1, Font_11x18, White);
			  y +=18;
		  }
		  else if (string_num == 2) {
				  ssd1306_WriteString(str2, Font_11x18, White);
				  y +=18;
			  }
		  else if (string_num == 3) {
				  ssd1306_WriteString(str3, Font_11x18, White);
				  y +=18;
			  }
		  else if (string_num == 4) {
				  ssd1306_WriteString(str4, Font_11x18, White);
				  y +=18;
			  }
		  ssd1306_UpdateScreen();
		 // HAL_Delay(1500);
		 // MPU6050_Data.aRoll += 0;
		 // MPU6050_Data.aPitch += 0; НУЖНО ВЫСТАВ�?ТЬ ЗАНОВО 0 !!!!!!!!!!!!!!!!!!!!!!!
		 // MPU6050_Data.aYaw += 0;
		 // MPU6050_Init();
		 // MPU6050_Calibrate();
		 // HAL_Delay(1500);
	 }
	 HAL_SYSTICK_Callback();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 64000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void MPU6050_Calibrate(void){

  	    int16_t mpu6050data[6];
  	    uint16_t iNumCM = 1000;
  	    for (int i = 0; i < iNumCM ; i ++){
  	      MPU6050_GetAllData(mpu6050data);
  	      fGX_Cal += mpu6050data[3];
  	      fGY_Cal += mpu6050data[4];
  	      fGZ_Cal += mpu6050data[5];
  	      HAL_Delay(3); // 3 сек на калибровку
  	    }
  	    fGX_Cal /= iNumCM;
  	    fGY_Cal /= iNumCM;
  	    fGZ_Cal /= iNumCM;
  	     isinitialized = 1;
  	  }
/*функция вызывается 1000 раз в секунду*/
	  void HAL_SYSTICK_Callback(void){

	      if(isinitialized && ++uiTicksCNT <= 100){ // 10 раз в секунду
	        int16_t mpu6050data[6];
	        // перемещаем в массив mpu6050data все данные с датчика
	        MPU6050_GetAllData(mpu6050data);

	        // РАБОТА С Г�?РОСКОПОМ
	        //как перевести Юниты в реальные градусы развернуто на примере Roll
	        float Roll = mpu6050data[4] - fGY_Cal; // относительно "нуля"
	        Roll = Roll/65.5/10;
	        MPU6050_Data.aRoll += Roll;

	        //сокращенная запись для Pitch
	        MPU6050_Data.aPitch += (mpu6050data[3] - fGX_Cal)/65.5/10;


	        float Yaw = (mpu6050data[5] - fGZ_Cal)/65.5/10;
	        MPU6050_Data.aYaw += Yaw;
	        //учитываем параметр по Z если по нему есть движение
	        if(Yaw > 0.01){//TODO: сравнение с дельтой
	          float _Y = sin(Yaw * 3.1415/180);
	          MPU6050_Data.aPitch += MPU6050_Data.aRoll  * _Y;
	          MPU6050_Data.aRoll -= MPU6050_Data.aPitch * _Y;
	        }

	        uiTicksCNT = 0;
	      }

	  }


	  void MPU6050_GetAllData(int16_t *Data){

	    uint8_t accelbuffer[14];

	    // с 0x3B 14 следующих регистров содержат данные измерения модуля
	    I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW,MPU6050_RA_ACCEL_XOUT_H,accelbuffer,14);

	    /* Registers 59 to 64 – Accelerometer Measurements */
	    for (int i = 0; i< 3; i++)
	        Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

	    /* Registers 65 and 66 – Temperature Measurement */
	    //пока пропускаем Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53

	    /* Registers 67 to 72 – Gyroscope Measurements */
	    for (int i = 4; i < 7; i++)
	        Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

	  }

	  void MPU6050_Init(void){

	      uint8_t buffer[7];

	      // включение/побудка модуля
	      buffer[0] = MPU6050_RA_PWR_MGMT_1;
	      buffer[1] = 0x00;
	      I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

	      // конфиг гироскопа на ±500°/с
	      buffer[0] = MPU6050_RA_GYRO_CONFIG;
	      buffer[1] = 0x8;
	      I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

	      // конфиг акселерометра на ±8g
	      buffer[0] = MPU6050_RA_ACCEL_CONFIG;
	      buffer[1] = 0x10;
	      I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
	  }

	  void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE){
		  I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);
		  while (HAL_I2C_Master_Receive(&hi2c2, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK) { /*ждем, что придет в ответ. То, что пришло в ответе отправляетм обратно в rxбуффер*/
		 			 if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {                                            /*если ошибка, вызываем хэндлер*/
		 				 Error_Handler();
		 			 }
		 		 }
		 		 while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
	  }
	  void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) {
		 while (HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDRESS<<1, aTxBuffer, TXBUFFERSIZE, 1000) != HAL_OK) { /*пока не отправлено крутиться в цикле*/
			 if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {                                            /*если ошибка, вызываем хэндлер*/
				 Error_Handler();
			 }
		 }
		 while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}  /*если отправилась, берется состояние шины, если она готова, тодальше передается управление в то место, откуда была вызвана функция*/
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

