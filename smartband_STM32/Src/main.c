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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{

	if('\n' == ch)
	{
		int ch_tmp = '\r';

		HAL_UART_Transmit(&huart1, (uint8_t *)&ch_tmp, 1, 0xFFFF);
	}

	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU9250_t MPU9250;
float filtered_Az;
TM_AHRSIMU_t IMU;
char UartBuffer[32];
int gyro_onoff = 0;
int hr_flag = 0;
int flag = 0;
int operation = 0;
int fall_flag = 0;
uint32_t tickstart1;
uint32_t tickstart2;
int fflag = 0;

void I2C2_ClearBusyFlagErratum(I2C_HandleTypeDef *instance, I2C_HandleTypeDef* instance2)
{
    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct2;
    int timeout =100;
    int timeout_cnt=0;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
    // 1. Clear PE bit.
    instance->Instance->CR1 &= ~(0x0001);
	instance2->Instance->CR1 &= ~(0x0001);
	//HAL_UART_Transmit(&huart1, (uint8_t *)"1\n", strlen("1\n"), 0xFFFF);
    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Alternate    = GPIO_AF4_I2C1;
    GPIO_InitStruct.Pull         = GPIO_PULLUP;
    GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct2.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct2.Alternate    = GPIO_AF4_I2C2;
    GPIO_InitStruct2.Pull         = GPIO_PULLUP;
    GPIO_InitStruct2.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin          = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin          = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
    GPIO_InitStruct2.Pin          = I2C2_SCL_PIN;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct2);
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
    GPIO_InitStruct2.Pin          = I2C2_SDA_PIN;
    HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct2);
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

    //HAL_UART_Transmit(&huart1, (uint8_t *)"2\n", strlen("1\n"), 0xFFFF);
    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    //HAL_UART_Transmit(&huart1, (uint8_t *)"3\n", strlen("1\n"), 0xFFFF);
    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_RESET);
	//HAL_UART_Transmit(&huart1, (uint8_t *)"4\n", strlen("1\n"), 0xFFFF);
    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    //HAL_UART_Transmit(&huart1, (uint8_t *)"5\n", strlen("1\n"), 0xFFFF);
    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
	//HAL_UART_Transmit(&huart1, (uint8_t *)"6\n", strlen("1\n"), 0xFFFF);
    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    //HAL_UART_Transmit(&huart1, (uint8_t *)"7\n", strlen("1\n"), 0xFFFF);
    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
	//HAL_UART_Transmit(&huart1, (uint8_t *)"8\n", strlen("1\n"), 0xFFFF);
    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    //HAL_UART_Transmit(&huart1, (uint8_t *)"9\n", strlen("1\n"), 0xFFFF);
    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);
	//HAL_UART_Transmit(&huart1, (uint8_t *)"10\n", strlen("10\n"), 0xFFFF);
    // 11. Check SDA High level in GPIOx_IDR.

	/*
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout){
        	return;
        }
    }
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout){
            printf("timeout\n");
        	return;
        }
    }*/

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }


    //HAL_UART_Transmit(&huart1, (uint8_t *)"11\n", strlen("10\n"), 0xFFFF);
    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	GPIO_InitStruct2.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct2.Pull = GPIO_PULLUP;
    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct2.Alternate = GPIO_AF4_I2C2;

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
	GPIO_InitStruct2.Pin = I2C2_SCL_PIN;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct2);
    GPIO_InitStruct2.Pin = I2C2_SDA_PIN;
    HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct2);

    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);
    //HAL_UART_Transmit(&huart1, (uint8_t *)"12\n", strlen("10\n"), 0xFFFF);
    // 13. Set SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 |= 0x8000;
	instance2->Instance->CR1 |= 0x8000;
    asm("nop");
    //HAL_UART_Transmit(&huart1, (uint8_t *)"13\n", strlen("10\n"), 0xFFFF);
    // 14. Clear SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 &= ~0x8000;
	instance2->Instance->CR1 &= ~0x8000;
    asm("nop");
    //HAL_UART_Transmit(&huart1, (uint8_t *)"14\n", strlen("10\n"), 0xFFFF);
    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    instance->Instance->CR1 |= 0x0001;
	instance2->Instance->CR1 |= 0x0001;
	//HAL_UART_Transmit(&huart1, (uint8_t *)"15\n", strlen("10\n"), 0xFFFF);
    // Call initialization function.
    HAL_I2C_Init(instance);
	HAL_I2C_Init(instance2);

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_SET);
	if(gyro_onoff)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
	if(hr_flag)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
	if(fall_flag)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	int hr, hr_count = 0;
	int sum_hr = 0;
	int avr_hr = 0;
	char tx_test[100];
	char uartbuff[20];

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
  if(HAL_UART_Receive_IT(&huart1, g_Uart_RxBuffer, UART_RXBUFFERSIZE) != HAL_OK)
  {
      Error_Handler();

  }
  if(HAL_UART_Receive_IT(&huart6, g_Uart_6_Com_RxBuffer, UART_RXBUFFERSIZE) != HAL_OK)
  {
      Error_Handler();

  }
  HAL_UART_Transmit(&huart6, (uint8_t *)"test\n", strlen("test\n"), 0xFFFF);


#if 1 ////GYRO////
	if (MPU9250_Init(&hi2c2, &MPU9250, MPU9250_Device_0) != MPU9250_Result_Ok) {
		printf("Device error!\r\n");
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
		while(MPU9250_Init(&hi2c2, &MPU9250, MPU9250_Device_0) != MPU9250_Result_Ok){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
			I2C2_ClearBusyFlagErratum(&hi2c1, &hi2c2);
		}
	}
	TM_AHRSIMU_Init(&IMU, 1000, 0.5, 0);
#endif ////GYRO////

#if 1 ////HEART RATE////
	if(Max30102_Init(&hi2c1) != MAX30102_OK){
		//printf("MAX30102 error\r\n");
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
		I2C2_ClearBusyFlagErratum(&hi2c1, &hi2c2);
		Max30102_Init(&hi2c1);
	}
#endif ////HEART RATE////

	printf("START\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		UART6_Test();
#if 1
		if (MPU9250_DataReady(&hi2c2, &MPU9250) == MPU9250_Result_Ok) {
			MPU9250_ReadAcce(&hi2c2, &MPU9250);
			MPU9250_ReadGyro(&hi2c2, &MPU9250);
			MPU9250_ReadMag(&hi2c2, &MPU9250);
		}
		else{
			do{
				printf("MPU9250_Result_Error\n");
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);
				I2C2_ClearBusyFlagErratum(&hi2c1, &hi2c2);
			}while(MPU9250_Init(&hi2c2, &MPU9250, MPU9250_Device_0) != MPU9250_Result_Ok);


		}
		TM_AHRSIMU_UpdateIMU(&IMU, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Ax, MPU9250.Ay, MPU9250.Az);
#endif

#if 1 ////GYRO////
		if(gyro_onoff) {
			sprintf(tx_test, "[%d,%.0f,%.0f\n", flag, IMU.Roll, IMU.Pitch);
			if(flag == 1){
				HAL_UART_Transmit(&huart6, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
			}
		} else
		{
			double SMV = sqrt(MPU9250.Ax * MPU9250.Ax + MPU9250.Ay * MPU9250.Ay + MPU9250.Az * MPU9250.Az);
			if(fall_flag && SMV > 1.4){
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
				printf("fall flag zero\n");
				fall_flag = 0;
				HAL_Delay(1000);
			}
			if(SMV > 3.4) {
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
				printf("SMV 3.4\n");
				tickstart2 = HAL_GetTick();
				fall_flag = 1;
				HAL_Delay(1000);
			}
			if(fall_flag && (HAL_GetTick() - tickstart2) >= 10000)
			{
				printf("fall detected\n");
				fall_flag = 0;
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);

				sprintf(uartbuff, "@");
				HAL_UART_Transmit(&huart1, (uint8_t *)uartbuff, strlen(uartbuff), 0xFFFF);
			}
			if(fflag == 1){
				sprintf(tx_test, "[%d,0,0\n", flag);
				HAL_UART_Transmit(&huart6, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
				fflag = 0;
			}
		}
#endif ////GYRO////

#if 1 ////HEART RATE////
		if(hr_flag){
			Max30102_Task();
			hr = (int)Max30102_GetHeartRate();

			if(40 < hr && hr < 130) {
				sum_hr += hr;
				hr_count++;
			}
			if((HAL_GetTick() - tickstart1) >= 10000)
			{
				avr_hr = sum_hr / hr_count;
				//sprintf(tx_test, "Heart Rate : %d bpm \r\n", avr_hr);
				sprintf(uartbuff, "<%d>", avr_hr);
				//HAL_UART_Transmit(&huart1, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
				HAL_UART_Transmit(&huart1, (uint8_t *)uartbuff, strlen(uartbuff), 0xFFFF);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
				hr_flag = 0;
			}

		}
#endif ////HEART RATE////
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_12) {
		Max30102_InterruptCallback();
	}
	else if(GPIO_Pin == GPIO_PIN_9) {
		if(gyro_onoff){
			gyro_onoff = 0;
			flag = 3;
			fflag = 1;
			/*
			sprintf(tx_test, "[%d,0,0\n", flag);
			HAL_UART_Transmit(&huart6, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
			printf("\n");
			*/
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
		} else {
			gyro_onoff = 1;
			flag = 1;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
		}
	}
	else if(GPIO_Pin == GPIO_PIN_10) {
		//flag = 2;
		printf("wait 10 sec...\n");
		hr_flag = 1;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		tickstart1 = HAL_GetTick();
	}
}

I2C_Result_t I2C_Write(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t data) {
	uint8_t d[2];

	/* Format array to send */
	d[0] = register_address;
	d[1] = data;

	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}
		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}

I2C_Result_t I2C_ReadMulti(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count) {

	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, &register_address, 1, 0xffff) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}
		if(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_TIMEOUT)

		/* Return error */
		return I2C_Result_Error;
	}

	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(hi2c, device_address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
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
