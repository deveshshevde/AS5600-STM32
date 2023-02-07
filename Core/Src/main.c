/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AS5600_SLAVE_ADDRESS			0x36
#define AS5600_SHIFTED_SLAVE_ADDRESS	0x6c//(for create spacing for R/W bit)
#define AS5600_I2C_TIMEOUT_DEFAULT		10	// can be choosen from (1 - 30) ms
#define I2C_MEMADD_SIZE_8BIT            0x00000001U //from HAL library

float degree_ang = 0;
//datasheet : https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf

/* AS5600 configuration registers */
#define AS5600_REGISTER_ZMCO			0x00
#define AS5600_REGISTER_ZPOS_HIGH		0x01
#define AS5600_REGISTER_ZPOS_LOW		0x02
#define AS5600_REGISTER_MPOS_HIGH		0x03
#define AS5600_REGISTER_MPOS_LOW		0x04
#define AS5600_REGISTER_MANG_HIGH		0x05
#define AS5600_REGISTER_MANG_LOW		0x06
#define AS5600_REGISTER_CONF_HIGH		0x07
#define AS5600_REGISTER_CONF_LOW		0x08
/* AS5600 output registers */
#define AS5600_REGISTER_RAW_ANGLE_HIGH	0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW	0x0D
#define AS5600_REGISTER_ANGLE_HIGH		0x0E
#define AS5600_REGISTER_ANGLE_LOW		0x0F
/* AS5600 status registers */
#define AS5600_REGISTER_STATUS			0x0B
#define AS5600_REGISTER_AGC				0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH	0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW	0x1C
#define AS5600_REGISTER_BURN			0xFF

//Ignoring power setting register , hysteresis register , pwm , slow filte , fast filter ,watchdog setting

/* AS5600 direction definitions */
#define AS5600_DIR_CW					1
#define AS5600_DIR_CCW					2

/* AS5600 bit mask */
#define AS5600_12_BIT_MASK				(uint16_t)4095 // 4096 resolution
/* AS5600 angle conversions */
#define AS5600_DEG_CONV 8.7890625e-2    /* 360/4096 */
#define AS5600_RAD_CONV 1.5339808e-3    /* 2pi/4096 */
uint8_t  deltaT;
int ptime ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float speed = 0 ;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

#define I2C_TIMEOUT_BASE 10
#define I2C_TIMEOUT_BYTE 1

#define AS5600_RAW_ADDR 0X36

#define AS5600_ADDR (AS5600_RAW_ADDR << 1)

#define ZMCO		0x00
#define ZPOS_H		0x01
#define ZPOS_L		0x02
#define MPOS_H		0x03
#define MPOS_L		0x04
#define MANG_H		0x05
#define MANG_L		0x06
#define CONF_L		0x07
#define CONF_H		0x08
#define RAWANG_L	0x0D
#define ANGLE_H		0x0E
#define ANGLE_L		0x0F

#define BURN		0xFF

#define AS5600_RESOLUTION 4096//12bit/lines resolution
#define RAWANG_H 0X0C


double myANG = 0 ;
float raw_ang = 0 ;

float actual_angle = 0 ;

uint8_t buffer[2] = {0};



static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;


    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, pData, count, 100);
    return status;
}
static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;

    status =  HAL_I2C_Master_Receive(&hi2c1, (dev_addr | 1), pData, count, 100);
    return status;
}



uint16_t as5600GetRawAngle(void) {
    uint16_t raw_angle;

    uint8_t raw_angle_reg = RAWANG_H;

    i2cWrite(AS5600_ADDR, &raw_angle_reg, 1);

    i2cRead(AS5600_ADDR, buffer, 2);

    raw_angle = (((uint16_t) buffer[0] << 8) | (uint16_t) buffer[1]);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    return raw_angle;

}

uint32_t CurrentMicro(void)
{
  uint32_t m0 = HAL_GetTick();
  uint32_t u0 = SysTick->LOAD - SysTick->VAL;
  uint32_t m1 = HAL_GetTick();
  uint32_t u1 = SysTick->LOAD - SysTick->VAL;

  if (m1 > m0) {
    return ( m1 * 1000 + (u1 * 1000) / SysTick->LOAD);
  } else {
    return ( m0 * 1000 + (u0 * 1000) / SysTick->LOAD);
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
//	initialise_monitor_handles();
//    as5600_init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* fReset of all peripherals, Initializes the Flash interface and the Systick. */
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   ptime = CurrentMicro();
	  raw_ang = as5600GetRawAngle()/11.3;
	  int ctime;
	  float new_raw_ang = as5600GetRawAngle()/11.3;
	   deltaT = ctime - ptime;
	  speed =((new_raw_ang - raw_ang)/deltaT*1.e06 );
	  new_raw_ang = raw_ang;
	  ptime = ctime;



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
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
