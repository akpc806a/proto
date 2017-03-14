/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "PS2Interface.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}


T_PS2Interface PS2Interface0;

void PS2_0_TimeOut_Restart()
{
  // start time-out timer from 0 value
  HAL_TIM_OC_Start_IT(&htim14, TIM_CHANNEL_1);
  __HAL_TIM_SET_COUNTER(&htim14, 0);
}

// functions for driving pins should impelement the following:
// if value == 0, configure pin as open drain and pull it do groung
// if value == 1, configure pin as input
// then make pause to form proper baudrate
void PS2_0_SCLK_Drive(uint8_t value)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int i;
  
  GPIO_InitStruct.Pin = CLK0_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  
  if (value == 0)
  {
    // configure as open-drain
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(CLK0_GPIO_Port, &GPIO_InitStruct);
    
    // write 0
    HAL_GPIO_WritePin(CLK0_GPIO_Port, CLK0_Pin, GPIO_PIN_RESET);
  }
  else
  {
    // configure as input, external interrupt (will be externally pulled up)
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(CLK0_GPIO_Port, &GPIO_InitStruct);
  }
  
  //for (i = 0; i < 10000; i++) ;
}

void PS2_0_DATA_Drive(uint8_t value)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int i;
  
  GPIO_InitStruct.Pin = DATA0_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  
  if (value == 0)
  {
    // configure open-drain
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(DATA0_GPIO_Port, &GPIO_InitStruct);
    
    // write 0
    HAL_GPIO_WritePin(DATA0_GPIO_Port, DATA0_Pin, GPIO_PIN_RESET);
  }
  else
  {
    // configure as input, input (will be externally pulled up)
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DATA0_GPIO_Port, &GPIO_InitStruct);
  }
  
  //for (i = 0; i < 10000; i++) ;
}

uint8_t PS2_0_SCLK_Get() { return (HAL_GPIO_ReadPin(CLK0_GPIO_Port, CLK0_Pin) != 0); }
uint8_t PS2_0_DATA_Get() { return (HAL_GPIO_ReadPin(DATA0_GPIO_Port, DATA0_Pin) != 0); }


int X_0 = 0;
int Y_0 = 0;
int X_1 = 0;
int Y_1 = 0;
uint32_t beep_timestamp = 0;
uint8_t beep = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  int iRes = 0;
  uint8_t data = 0;
  int8_t dx, dy;
  uint8_t done = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */
  __HAL_DBGMCU_FREEZE_TIM14();
  
/*
while (1)
{
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
PS2_0_TimeOut_Restart();
HAL_Delay(50); 
}
*/

  PS2Interface_Init(&PS2Interface0, PS2_0_SCLK_Drive, PS2_0_DATA_Drive, PS2_0_SCLK_Get, PS2_0_DATA_Get, PS2_0_TimeOut_Restart);

  // setup first mouse

  HAL_Delay(50); 

	// reset response
  while( byteQueue_IsEmpty(& PS2Interface0.RxQueue) ) ; // waiting for initial response
	data = byteQueue_Get(& PS2Interface0.RxQueue); 
	if (data != 0xAA) 
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  printf("%x\r\n", data);
	while (byteQueue_IsEmpty(& PS2Interface0.RxQueue) == 0) 
  {	
    data = byteQueue_Get(& PS2Interface0.RxQueue); printf("%x\r\n", data); 
    HAL_Delay(1);
  }
  
  HAL_Delay(5);

  // 800 dpi command
  PS2Interface_Send(&PS2Interface0, 0xE8);
  PS2Interface_Send(&PS2Interface0, 0x03);
  
  HAL_Delay(1); 
  // start broadcasting
  PS2Interface_Send(&PS2Interface0, 0xF4);
  
  // wait until all responses received
  HAL_Delay(10); 
  // empty rx buffer
while (byteQueue_IsEmpty(& PS2Interface0.RxQueue) == 0) {	data = byteQueue_Get(& PS2Interface0.RxQueue); printf("%x\r\n", data); } 

	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    // we got first byte
    if ( ! byteQueue_IsEmpty(& PS2Interface0.RxQueue) )
    {
      //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      
      data = byteQueue_Get(& PS2Interface0.RxQueue);
      if (data & 0x8) // probably first byte?
      {
        // wait for x byte
        while (byteQueue_IsEmpty(& PS2Interface0.RxQueue) ) ;  
        dx = byteQueue_Get(& PS2Interface0.RxQueue);
        // wait for y bye
        while (byteQueue_IsEmpty(& PS2Interface0.RxQueue) ) ;  
        dy = byteQueue_Get(& PS2Interface0.RxQueue);
        X_0 += dx;
        Y_0 += dy;
        
        //printf("X_0 = %d, Y_0 = %d\r\n", X_0, Y_0);
        printf("X_0 = %d, Y_0 = %d, X_1 = %d, Y_1 = %d\r\n", X_0, Y_0, X_1, Y_1);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      }
      
      //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
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

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK0_Pin */
  GPIO_InitStruct.Pin = CLK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA0_Pin */
  GPIO_InitStruct.Pin = DATA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim14)
  {
    HAL_TIM_OC_Stop_IT(&htim14, TIM_CHANNEL_1); // stop timer
    
    PS2Interface_RxTimeOutElapsed(&PS2Interface0); // call PS/2 instance callback
    
    //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t bData;
  
  if (GPIO_Pin == CLK0_Pin)
  {
    //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    
    bData = HAL_GPIO_ReadPin(DATA0_GPIO_Port, DATA0_Pin); // read pin value
    
    PS2Interface_ProcessRx(&PS2Interface0, bData); // pass bit value to PS/2 instance
		
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
