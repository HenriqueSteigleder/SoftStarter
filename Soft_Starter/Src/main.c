/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include "lcd.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId Comanda_DisplayHandle;
osThreadId Leitura_TempoHandle;
osThreadId Disparo_TiristHandle;
osThreadId Comunica_PCHandle;
osThreadId Start_StopHandle;
osThreadId Detector_CorentHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define SERIAL_TIMEOUT 10

int Desloca_Pulso = 730;
int Tempo_Estouro = 0;
int Tempo_Subida = 5;
int Tempo_Descida = 0;
int RPM = 0;
uint16_t Tempo_ARR_13 = 0;
uint8_t Comando = 0;
uint8_t Alerta = 0;
uint8_t Estado = 0;				//(0->Parado)(1->Partindo)(2->Girando)(3->Parando)
uint8_t T_Recebe_Tempo[5];
uint8_t T_Comandos;
uint8_t Flag;
uint8_t Confirma[128];
uint8_t Chave = 0;
int Estagio_Com = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
void Comanda_Display_Task(void const * argument);
void Leitura_Tempo_Task(void const * argument);
void Disparo_Tirist_Task(void const * argument);
void Comunica_PC_Task(void const * argument);
void Start_Stop_Task(void const * argument);
void Detector_Corent_Task(void const * argument);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM13_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim13);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  lcd_init();
  lcd_puts("Teste Display");
  sprintf(Confirma,"OLA  \r\n");



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

  /* Create the thread(s) */
  /* definition and creation of Comanda_Display */
  osThreadDef(Comanda_Display, Comanda_Display_Task, osPriorityNormal, 0, 128);
  Comanda_DisplayHandle = osThreadCreate(osThread(Comanda_Display), NULL);

  /* definition and creation of Leitura_Tempo */
  osThreadDef(Leitura_Tempo, Leitura_Tempo_Task, osPriorityNormal, 0, 128);
  Leitura_TempoHandle = osThreadCreate(osThread(Leitura_Tempo), NULL);

  /* definition and creation of Disparo_Tirist */
  osThreadDef(Disparo_Tirist, Disparo_Tirist_Task, osPriorityNormal, 0, 128);
  Disparo_TiristHandle = osThreadCreate(osThread(Disparo_Tirist), NULL);

  /* definition and creation of Comunica_PC */
  osThreadDef(Comunica_PC, Comunica_PC_Task, osPriorityNormal, 0, 128);
  Comunica_PCHandle = osThreadCreate(osThread(Comunica_PC), NULL);

  /* definition and creation of Start_Stop */
  osThreadDef(Start_Stop, Start_Stop_Task, osPriorityNormal, 0, 128);
  Start_StopHandle = osThreadCreate(osThread(Start_Stop), NULL);

  /* definition and creation of Detector_Corent */
  osThreadDef(Detector_Corent, Detector_Corent_Task, osPriorityNormal, 0, 128);
  Detector_CorentHandle = osThreadCreate(osThread(Detector_Corent), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 839;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 829;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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

  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 10;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8399;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 839;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 684;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D0_Pin|Habilita_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D2_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Detector_Sobre_Corrente_Pin */
  GPIO_InitStruct.Pin = Detector_Sobre_Corrente_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Detector_Sobre_Corrente_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Habilita_Analog_Pin Start_Soft_Pin */
  GPIO_InitStruct.Pin = Habilita_Analog_Pin|Start_Soft_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin Habilita_Pin RS_Pin */
  GPIO_InitStruct.Pin = D0_Pin|Habilita_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : D3_Pin */
  GPIO_InitStruct.Pin = D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D1_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* Comanda_Display_Task function */
void Comanda_Display_Task(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  if(Comando != 0){
		  lcd_clear();
	  }
	  switch(Comando){
	  case 1:
		  lcd_puts("Partindo");
		  break;
	  case 2:
		  lcd_puts("Girando");
		  break;
	  case 3:
		  lcd_puts("Desacelerando");
		  break;
	  case 4:
		  lcd_puts("Parado");
		  break;
	  case 5:
		  lcd_puts("RPM = ");
		  lcd_printd(RPM);
		  break;
	  case 6:
		  lcd_puts("Sobre-Corrente");
		  break;
	  case 7:
		  lcd_puts("Teste Display");
		  break;
	  case 8:
		  lcd_puts("Teste Display");
		  break;
	  case 9:
		  lcd_puts("Teste Display");
		  break;
	  }
	  Comando = 0;
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* Leitura_Tempo_Task function */
void Leitura_Tempo_Task(void const * argument)
{
  /* USER CODE BEGIN Leitura_Tempo_Task */
  /* Infinite loop */
  for(;;)
  {
	   if((Tempo_Subida>4)&&(Tempo_Subida<51)&&(Tempo_Subida != Tempo_Descida)){

		  Tempo_ARR_13 = (Tempo_Subida*100000)/730;
		  __HAL_TIM_SET_AUTORELOAD(&htim13, Tempo_ARR_13);
		  Tempo_Descida = Tempo_Subida;

	  }
    osDelay(1);
  }
  /* USER CODE END Leitura_Tempo_Task */
}

/* Disparo_Tirist_Task function */
void Disparo_Tirist_Task(void const * argument)
{
  /* USER CODE BEGIN Disparo_Tirist_Task */
  /* Infinite loop */
  for(;;)
  {
	  if(1 == __HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE)){
	 		  __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_UPDATE);
	 		  switch(Estado){

	 		  case 1:
	 			  Desloca_Pulso--;
	 			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Desloca_Pulso);
	 			  __HAL_TIM_SET_AUTORELOAD(&htim1, (Desloca_Pulso+50));

	 			  if(Desloca_Pulso<2){
	 				  Estado = 2;
	 				  Comando = 2;
	 				  Flag = 1;
	 			  }
	 			  break;

	 		  case 3:
	 			  Desloca_Pulso++;

	 			  __HAL_TIM_SET_AUTORELOAD(&htim1, (Desloca_Pulso+50));
	 			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Desloca_Pulso);

	 			  if(Desloca_Pulso>730){

	 				  Estado = 0;
	 				  Comando = 4;
	 				  Flag = 1;
	 			  }
	 			  break;
	 		  }
	  }
    osDelay(1);
  }
  /* USER CODE END Disparo_Tirist_Task */
}

/* Comunica_PC_Task function */
void Comunica_PC_Task(void const * argument)
{
  /* USER CODE BEGIN Comunica_PC_Task */
  /* Infinite loop */
  for(;;)
  {

			switch(Estagio_Com){
			case 0:
				if(HAL_UART_Receive(&huart2, &T_Comandos,1, SERIAL_TIMEOUT)==HAL_OK){
					if(T_Comandos == 't') Estagio_Com = 1;
					if(T_Comandos == 'd') Chave = 1;
				}
				break;
			case 1:
				if(HAL_UART_Receive(&huart2, &T_Recebe_Tempo[0], 3, 0x5000)==HAL_OK){
				Tempo_Subida = (((T_Recebe_Tempo[1])-48)*10)+((T_Recebe_Tempo[2])-48);
				}
				Estagio_Com = 0;
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			case 6:
				break;
			}

    osDelay(1);
  }
  /* USER CODE END Comunica_PC_Task */
}

/* Start_Stop_Task function */
void Start_Stop_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Stop_Task */
  /* Infinite loop */
  for(;;)
  {

	//if(HAL_GPIO_ReadPin(Start_Soft_GPIO_Port, Start_Soft_Pin)){
	 if(Chave){
		  Chave = 0;
		switch(Estado){

			 case 0:
				 Desloca_Pulso = 730;
				__HAL_TIM_SET_COUNTER(&htim13, 0);
			  	Estado = 1;
			  	Comando = 1;
			  	Flag = 1;
			  	break;

			 case 2:
				 Desloca_Pulso = 0;
				 __HAL_TIM_SET_COUNTER(&htim13, 0);   //(0->Parado)(1->Partindo)(2->Girando)(3->Parando)
				Estado = 3;
				Comando = 3;
				Flag = 1;
				break;
		}
	 }
    osDelay(1);
  }
  /* USER CODE END Start_Stop_Task */
}

/* Detector_Corent_Task function */
void Detector_Corent_Task(void const * argument)
{
  /* USER CODE BEGIN Detector_Corent_Task */
  /* Infinite loop */
  for(;;)
  {
	  Alerta = !HAL_GPIO_ReadPin(Detector_Sobre_Corrente_GPIO_Port,Detector_Sobre_Corrente_Pin);
	  //Comando = 6;
    osDelay(1);
  }
  /* USER CODE END Detector_Corent_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

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
