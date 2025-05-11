/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t Comutation_State = 0;
uint32_t Comutation_State_Old = 1;
uint32_t CloseLoop = 0;
/*
	   PCL  PBL  PAL  PCH  PBH  PAH
	   PC5	PC4	 PC3  PC2  PC1  PC0
	    1    0    0    0    0    1		//0x21
		1    0    0    0    1    0		//0x22
		0    0    1    0    1    0		//0x0A
		0    0    1    1    0    0		//0x0C
		0    1    0    1    0    0		//0x14
		0    1    0    0    0    1		//0x11
*/


volatile	  uint32_t BldcOutODR[] = {
			  (0x00000021),
			  (0x00000022),
			  (0x0000000A),
			  (0x0000000C),
			  (0x00000014),
			  (0x00000011),
	  };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(CloseLoop == 1){

		  if(GPIO_Pin == BEMFA_Pin){												//if source is BEMFA
			  if( HAL_GPIO_ReadPin(BEMFA_GPIO_Port, BEMFA_Pin) == GPIO_PIN_SET){ 	//if Rising edge trigger
				  if(Comutation_State == 1){
					  Comutation_State = 2;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 4){
			  			  Comutation_State = 5;
			  		  }
			  	  }
		  }

		  if(GPIO_Pin == BEMFB_Pin){												//if source is BEMFB
			  if( HAL_GPIO_ReadPin(BEMFB_GPIO_Port, BEMFB_Pin) == GPIO_PIN_SET){ 	//if Rising edge trigger
				  if(Comutation_State == 3){
					  Comutation_State = 4;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 0){
			  			  Comutation_State = 1;
			  		  }
			  	  }
		  }

		  if(GPIO_Pin == BEMFC_Pin){												//if source is BEMFC
			  if( HAL_GPIO_ReadPin(BEMFC_GPIO_Port, BEMFC_Pin) == GPIO_PIN_SET){ 	//if Rising edge trigger
				  if(Comutation_State == 5){
					  Comutation_State = 0;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 2){
			  			  Comutation_State = 3;
			  		  }
			  	  }
		  }

		 uint32_t dly = 32000;
		  while(dly--);
		 if(Comutation_State != Comutation_State_Old){
			 GPIOC->ODR = BldcOutODR[Comutation_State];	//Set the output for Commutation Stage
			 Comutation_State_Old = Comutation_State;
		 }
	}
}


void BLDCOpenLoop(){
	  uint32_t dly=50;
	  uint32_t i = 0;

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  TIM3->CCR1 = 100;

	  CloseLoop = 0; //close loop is inactive

	  while(dly-- > 49){
		  for(i=0; i<1;i++){
			  GPIOC->ODR = BldcOutODR[i];
			  HAL_Delay(10);
		  }
	  }

	  CloseLoop = 1; //Switch to Close loop control with backemf
	  Comutation_State = 0;

}






//TEMP start

/*

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{



	if(CloseLoop == 1){

		  if(GPIO_Pin == BEMFB_Pin){							//if source is BEMFA
			  HAL_GPIO_TogglePin(S1_GPIO_Port, S1_Pin);

			  if( HAL_GPIO_ReadPin(BEMFB_GPIO_Port, BEMFB_Pin)){ //if Rising edge trigger
				  if(Comutation_State == 4){
					  Comutation_State = 5;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 1){
			  			  Comutation_State = 2;
			  		  }
			  	  }
		  }



		  if(GPIO_Pin == BEMFA_Pin){
			  HAL_GPIO_TogglePin(S2_GPIO_Port, S2_Pin);

			  if( HAL_GPIO_ReadPin(BEMFA_GPIO_Port, BEMFA_Pin)){ //if Rising edge trigger
				  if(Comutation_State == 0){
					  Comutation_State = 1;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 3){
			  			  Comutation_State = 4;
			  		  }
			  	  }
		  }


		  if(GPIO_Pin == BEMFC_Pin){
			  HAL_GPIO_TogglePin(S3_GPIO_Port, S3_Pin);

			  if( HAL_GPIO_ReadPin(BEMFC_GPIO_Port, BEMFC_Pin)){ //if Rising edge trigger
				  if(Comutation_State == 2){
					  Comutation_State = 3;
				  }
			  }
			  	  else{
			  		  if(Comutation_State == 5){
			  			  Comutation_State = 0;
			  		  }
			  	  }
		  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
	GPIOA->ODR = BldcOut[Comutation_State];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);

	}

}


*/
//TEMP End

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  BLDCOpenLoop();
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PAH_Pin|PBH_Pin|PCH_Pin|PAL_Pin
                          |PBL_Pin|PCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAH_Pin PBH_Pin PCH_Pin PAL_Pin
                           PBL_Pin PCL_Pin */
  GPIO_InitStruct.Pin = PAH_Pin|PBH_Pin|PCH_Pin|PAL_Pin
                          |PBL_Pin|PCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BEMFA_Pin BEMFB_Pin BEMFC_Pin */
  GPIO_InitStruct.Pin = BEMFA_Pin|BEMFB_Pin|BEMFC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
