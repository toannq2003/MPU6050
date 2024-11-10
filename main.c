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
#include <math.h>
#include "GY521.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void mpu6050_offset(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int i = 0;
uint32_t tick_start, elapsed_time;

int accel_buffer[3] = {0};
int gyro_buffer[3] = {0};

uint8_t accel[6], gyro[6];

int16_t accel_raw[3], gyro_raw[3];   //accel_x_raw, accel_y_raw, accel_z_raw
                                     //gyro_x_raw, gyro_y_raw, gyro_z_raw

int16_t a_linear[2], gyro_offset[3];

float Ax, Ay, Az;
double accel_g_base[3], accel_g[3], w_gyro[3], w_raw, g_base, angle_div, sin_angle, dt = 0;

double g_l, q_l, q[4] = {1, 0, 0, 0}, q_current[4], R[3][3], g_unit[3], v_linear[2] = {0};


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
  /* USER CODE BEGIN 2 */
	HAL_Delay(300);
  MPU6050_Init();
  HAL_Delay(300);
	mpu6050_offset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		for (int k = 0; k < 1000; k++) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			tick_start = HAL_GetTick();
			i = 1;
			HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, accel, 6, 1000);
			HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, gyro, 6, 1000);
				
			accel_raw[0] = (int16_t)(accel[0] << 8 | accel[1]);
			accel_raw[1] = (int16_t)(accel[2] << 8 | accel[3]);
			accel_raw[2] = (int16_t)(accel[4] << 8 | accel[5]);
				
			gyro_raw[0] = (int16_t)(gyro[0] << 8 | gyro[1]) - gyro_offset[0];
			gyro_raw[1] = (int16_t)(gyro[2] << 8 | gyro[3]) - gyro_offset[1];
			gyro_raw[2] = (int16_t)(gyro[4] << 8 | gyro[5]) - gyro_offset[2];
			
			if ((gyro_raw[0] > -500) && (gyro_raw[0] < 500))
			{
				gyro_raw[0] = 0;
			}
			
			if ((gyro_raw[1] > -500) && (gyro_raw[1] < 500))
			{
				gyro_raw[1] = 0;
			}
			
			if ((gyro_raw[2] > -500) && (gyro_raw[2] < 500))
			{
				gyro_raw[2] = 0;
			}
			
			//determination quaternion q_current
			w_raw = sqrt(gyro_raw[0]*gyro_raw[0] + gyro_raw[1]*gyro_raw[1] + gyro_raw[2]*gyro_raw[2]);
			if (w_raw == 0)
			{
				q_current[0] = 1;
				q_current[1] = 0;
				q_current[2] = 0;
				q_current[3] = 0;
			}
			else 
			{
				angle_div = (w_raw/131.0)*dt*(PI/180)/2;
				q_current[0] = cos(angle_div);
				sin_angle = sin(angle_div);
				q_current[1] = (gyro_raw[0]/w_raw)*sin_angle;
				q_current[2] = (gyro_raw[1]/w_raw)*sin_angle;
				q_current[3] = (gyro_raw[2]/w_raw)*sin_angle;
			}
			
			//update quaternion q_new = q_current * q_old
			q[0] = q_current[0]*q[0] - q_current[1]*q[1] - q_current[2]*q[2] - q_current[3]*q[3];
			q[1] = q_current[0]*q[1] + q_current[1]*q[0] + q_current[2]*q[3] - q_current[3]*q[2];
			q[2] = q_current[0]*q[2] - q_current[1]*q[3] + q_current[2]*q[0] + q_current[3]*q[1];
			q[3] = q_current[0]*q[3] + q_current[1]*q[2] - q_current[2]*q[1] + q_current[3]*q[0];
			q_l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
			//update rotation matrix R
			R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];    R[0][1] = 2*q[1]*q[2] - 2*q[0]*q[3];                        R[0][2] = 2*q[1]*q[3] + 2*q[0]*q[2];
			R[1][0] = 2*q[1]*q[2] + 2*q[0]*q[3];                        R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];    R[1][2] = 2*q[2]*q[3] - 2*q[0]*q[1];
			R[2][0] = 2*q[1]*q[3] - 2*q[0]*q[2];                        R[2][1] = 2*q[2]*q[3] + 2*q[0]*q[1];                        R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
			
			
			//update gravity acceleration g_new = q_new*g*(q_new^-1) or g_new = R*g
			for (int i = 0; i < 3; i++)
			{
				accel_g[i] = (R[i][0]*accel_g_base[0] + R[i][1]*accel_g_base[1]+ R[i][2]*accel_g_base[2]);
			}
			g_l = sqrt(accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1] + accel_g[2]*accel_g[2]);
			
			//determination linear acceleration
			g_unit[0] = accel_g[0]/g_l;   g_unit[1] = accel_g[1]/g_l;   g_unit[2] = accel_g[2]/g_l;
			a_linear[0] = (uint16_t)(accel_raw[0]*g_unit[2] - accel_raw[2]*g_unit[0]);
			a_linear[1] = (uint16_t)((accel_raw[0]*g_unit[0] + accel_raw[1]*g_unit[1] + accel_raw[2]*g_unit[2]) - g_base);
			
			//determination linear velocity
			v_linear[0] += a_linear[0]*dt;
			v_linear[1] += a_linear[1]*dt;
		
			elapsed_time = HAL_GetTick() - tick_start;
			dt = elapsed_time / 1000.0;
			/*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(1000);*/
		}
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void mpu6050_offset(void)
{
	for (int i = 0; i < 100; i++)
	{
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, accel, 6, 1000);
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, gyro, 6, 1000);
		
		accel_buffer[0] += (int16_t)(accel[0] << 8 | accel[1]);
		accel_buffer[1] += (int16_t)(accel[2] << 8 | accel[3]);
		accel_buffer[2] += (int16_t)(accel[4] << 8 | accel[5]);
		
		gyro_buffer[0] += (int16_t)(gyro[0] << 8 | gyro[1]);
		gyro_buffer[1] += (int16_t)(gyro[2] << 8 | gyro[3]);
		gyro_buffer[2] += (int16_t)(gyro[4] << 8 | gyro[5]);
	}
	
	accel_g_base[0] = accel_buffer[0]/100.0;
	accel_g_base[1] = accel_buffer[1]/100.0;
	accel_g_base[2] = accel_buffer[2]/100.0;
	
	g_base = sqrt(accel_g_base[0]*accel_g_base[0] + accel_g_base[1]*accel_g_base[1] + accel_g_base[2]*accel_g_base[2]);
	
	gyro_offset[0] = gyro_buffer[0]/100;
	gyro_offset[1] = gyro_buffer[1]/100;
	gyro_offset[2] = gyro_buffer[2]/100;
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
