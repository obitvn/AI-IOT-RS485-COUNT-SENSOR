
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */


uint8_t flag_value_1, flag_value_2;
uint32_t value_out=0, last_value_out=0;
uint32_t time_start_value1, time_start_value2;

#define TIME_WAIT 500

#include "ObitString.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//#ifdef __GNUC__
// /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
// set to 'Yes') calls __io_putchar() */
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
// 
// /**
// * @brief Retargets the C library printf function to the USART.
// * @param None
// * @retval None
// */
//PUTCHAR_PROTOTYPE 
//{
// /* Place your implementation of fputc here */
// /* e.g. write a character to the USART */
// HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);


// return ch;
//}


#define RFID_UART_BUFFER_MAX_SIZE      128
uint8_t RFID_UartBuffer[RFID_UART_BUFFER_MAX_SIZE];
uint16_t RFID_UartRxIndex;
uint32_t RFID_UartLastTime;
uint8_t  RFID_DataBuff;

uint8_t val[3];
void SendData(uint8_t data)
{

	val[0] = data/100 +48;
	val[1]= data%100/10 +48;
	val[2]= data%10 +48;
	HAL_UART_Transmit(&huart1,(uint8_t*)"product11=",10,1);
	HAL_UART_Transmit(&huart1, val,3,1);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==huart1.Instance)
  {
    RFID_UartLastTime = HAL_GetTick();
	  RFID_UartBuffer[RFID_UartRxIndex] = 0; // clear 
	  RFID_UartBuffer[RFID_UartRxIndex] = RFID_DataBuff;
	  if(RFID_UartRxIndex < RFID_UART_BUFFER_MAX_SIZE - 1)
		 {
			RFID_UartRxIndex++;
		 }
		else 
		{
			RFID_UartRxIndex=0;
		  memset((char*)RFID_UartBuffer,0,RFID_UART_BUFFER_MAX_SIZE);
		}
		HAL_UART_Receive_IT(&huart1,&RFID_DataBuff,1);
  }
}


void CheckQuestionFromMaster( uint8_t *content_question, uint16_t content_question_length)
{
	if(obit_strcmp(content_question,content_question_length,RFID_UartBuffer,RFID_UartRxIndex) == 1)
	{
		RFID_UartRxIndex=0;
		memset((char*)RFID_UartBuffer,0,RFID_UART_BUFFER_MAX_SIZE);
		HAL_Delay(10); 
		SendData((uint8_t)value_out);
		if(value_out>0) last_value_out = value_out;
		value_out=0;
	}
}




void CountSensor()
{
	if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)==0)&&(flag_value_1 == 0))
	  {
		 flag_value_1 = 1;
		 time_start_value1 = HAL_GetTick();
	  }
	 if(((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)==1))&&(flag_value_1 == 1))
	 {
		 flag_value_1 = 0;
		 if((HAL_GetTick() - time_start_value1) > TIME_WAIT) 
		 {
			 value_out++;
		 }
	 }
	 //////////////////////////////////////////////////
	
	 if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==0)&&(flag_value_2 == 0))
	  {
		 flag_value_2 = 1;
		 time_start_value1 = HAL_GetTick();
	  }
	 if(((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==1))&&(flag_value_2 == 1))
	 {
		 flag_value_2 = 0;
		 if((HAL_GetTick() - time_start_value1) > TIME_WAIT) 
		 {
			 value_out++;
		 }
	 }
	 
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */
	//printf(" system init \r\n lightbolt \r\n count sensor rs485 \r\n by Tien Thinh 0981762826 \r\n");
  HAL_UART_Receive_IT(&huart1,&RFID_DataBuff,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   CheckQuestionFromMaster((uint8_t*)"@dv=product11",strlen("@dv=product11"));
	 CountSensor();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
