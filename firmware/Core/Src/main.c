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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"

#include "encoder.h"
#include "motor.h"
#include "pid_controller.h"
#include "system_config.h"
#include "trajectory.h"
#include "uart.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
//=== Vel: PI
#if 1
float Kp 	= 8.7719;
float Ki 	= 438.5965;
float Kb 	= 50.0000;
float Kd  = 0;
float alpha_vel = 0.0f;

//float Kp 	= 23.3;
//float Ki 	= 776.67;
//float Kb 	= 33.33;//;
//float Kd  = 0;
//float alpha_vel = 0.0f;
#endif
#if 0
float Kp 	= 1.1363;//0.2058;
float Ki 	= 37.8261;//6.3327;
float Kb 	= 33.2889;//;
float Kd  = 0;
float alpha_vel = 0.0f;
#endif
//=== Improved Profile Gains
float	Kp_pos = 800.0;
float Kpp_improved = 10.0;   // P gain position controller
//float Kpp_improved = 5.0;   // P gain position controller
float Kest = 100000.0;           // Estimation gain (< 1.0: overdamped, > 1.0: underdamped)
float Kcom = 20.0;           	 // Compensation gain (< 1.0: overdamped, > 1.0: underdamped)
//=== Uart
char Rx_indx, Rx_Buffer[MaxBuffer],Rx_data[MaxBuffer];
//char Tx_Buffer[MaxBuffer];
volatile uint8_t uart1_flag = 0;

encoder Encoder = {0};
motor Motor = {0};
pid_controller PIDVel;
pid_controller PIDPos;
Trajectory_t trajectory = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float max_pwm_P = SYS_MaxPWM_P;
float speed = 0;
int8_t chieu = 0;
uint32_t count_element = 0;
char Tx_Buffer[MaxBuffer];  // 2 buffer xen kẽ
volatile uint8_t active_buffer = 0;
volatile uint8_t dma_busy = 0;
PlotBuffer_t plot_buffer = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM4) {    // 5ms
			switch(Motor.mode){
				case MOTOR_SetPos:
					MOTOR_CalculateParams(&Motor, &Encoder, trajectory.sign, trajectory.phase);
					speed = Motor.CurVel_rad;
					if(chieu == -1) Motor.pwm = -1000;
					else if(chieu == 0) Motor.pwm = 0;
					else Motor.pwm = 1000;					break;
				
				case MOTOR_P_PI:
					// TÍNH TOÁN THÔNG SỐ
					MOTOR_CalculateParams(&Motor, &Encoder, trajectory.sign, trajectory.phase);
					// CẬP NHẬT TRAJECTORY
					if(trajectory.use_v1) TRAJ_DrawTrajectory_v1(&trajectory);
					else TRAJ_DrawTrajectory(&trajectory);
					// PID VỊ TRÍ
					Motor.DesiredVel_mm = PID_Pos_Improved(&PIDPos, trajectory.q_desired, Motor.CurPos_mm);
					Motor.DesiredVel_rad = Motor.DesiredVel_mm * SYS_MM2RAD;
					//if(fabs(Motor.DesiredVel_rad) < 0.85) Motor.DesiredVel_rad = 0;
					// PID VẬN TỐC
					Motor.pwm = PID_Vel(&PIDVel, Motor.DesiredVel_rad, Motor.CurVel_rad, trajectory.phase, trajectory.sign);
					break;
				case MOTOR_P:
					// TÍNH TOÁN THÔNG SỐ
					MOTOR_CalculateParams(&Motor, &Encoder, trajectory.sign, trajectory.phase);
					// CẬP NHẬT TRAJECTORY
					if(trajectory.use_v1) TRAJ_DrawTrajectory_v1(&trajectory);
					else TRAJ_DrawTrajectory(&trajectory);
					// PID VỊ TRÍ
					Motor.pwm = PID_Pos_Simple(&PIDPos, trajectory.q_desired, Motor.CurPos_mm);
					

					break;
			}
			if(Motor.tick_enabled) Motor.tick++;
			MOTOR_SetRun(&Motor);
			#if 1
			if (Motor.plot_active) {
            // Ghi d? li?u vào buffer
            if (plot_buffer.count < PLOT_BUFFER_SIZE) {
                PlotData_t *data = &plot_buffer.buffer[plot_buffer.write_idx];
                
                data->q_desired = trajectory.q_desired_prev;
                data->q_actual = Motor.CurPos_mm;
                data->v_desired = 0.0;
                data->v_actual = (float)Motor.DesiredVel_mm;
                data->v_motor = Motor.CurVel_mm;
                data->a_desired = 0.0;
                data->a_motor = Motor.Acc_mm;
								data->pwm = Motor.pwm;
								
                plot_buffer.write_idx = (plot_buffer.write_idx + 1) % PLOT_BUFFER_SIZE;
                
                // CRITICAL: Tang count ph?i atomic
                __disable_irq();
                plot_buffer.count++;
                __enable_irq();
            }
        }
			// Cập nhật state
			trajectory.v_desired_prev = trajectory.v_desired;
			trajectory.q_desired_prev = trajectory.q_desired;
			#endif
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        dma_busy = 0;  // Cho phép gửi tiếp
    }
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//--- Timer
	HAL_TIM_Base_Start_IT(&htim4);
	//--- PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	//--- UART
	UART_Receive_Enable(&huart1, (uint8_t *)Rx_Buffer, MaxBuffer);
	//--- SetParams
	
	MOTOR_SetParams(&Motor, TIM_CHANNEL_3, MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_SetPos, MOTOR_MAXVEL_MM);
	PID_SetParams(&PIDVel, Kp, Ki, Kd, Kb, alpha_vel, SYS_MaxPWM, SYS_SAMPLETIME);
	if(Motor.mode == MOTOR_P)PID_SetParamsImproved(&PIDPos, Kp_pos, SYS_MaxPWM_P, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
	else PID_SetParamsImproved(&PIDPos, Kpp_improved, Motor.v_max_mm, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
	PID_ResetState(&PIDPos);

	TRAJ_SetParams(&trajectory, SYS_SAMPLETIME, SYS_RUN_TIME, SYS_STOP_TIME, Motor.a_max_mm * 2.0, Motor.v_max_mm);
	
	float Kpp_improved_pre = Kpp_improved;
	float preKest = Kest;
	float preKcom = Kcom;
	float premax_pwm_P = max_pwm_P;
	float preKp_pos = Kp_pos;
//	trajectory.test = 1;
//	Motor.pwm = 1000;
//	trajectory.sign = 1;
//	HAL_Delay(1000);
//	MOTOR_SetRun(&Motor);
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 1000);
  while (1)
  {
			#if 1
			if(plot_buffer.reset) {
					chieu = 0;
					memset(Tx_Buffer, 0, sizeof(Tx_Buffer));
					UART_ResetPlotData(&plot_buffer);
				
					MOTOR_ResetParams(&Motor);
					ENCODER_ResetParams(&Encoder);
					TRAJ_ResetParams(&trajectory);
					PID_ResetParams(&PIDVel);
					plot_buffer.reset = 0;
					HAL_Delay(200);
			}
			switch(Motor.mode){
					case MOTOR_SetPos:
						break;
					
					case MOTOR_P_PI:
							if (plot_buffer.count > 0 && !dma_busy) {
									PlotData_t *data = &plot_buffer.buffer[plot_buffer.read_idx];
									
									int len = snprintf(Tx_Buffer, MaxBuffer,
																		"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f 0.0\r\n",
																		data->q_desired,
																		data->q_actual,
																		data->v_desired,
																		data->v_actual,
																		data->v_motor,
																		data->a_desired,
																		data->a_motor,
																		data->pwm
																		);
									
									if (len > 0 && len < MaxBuffer) {
											dma_busy = 1;
											HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Tx_Buffer, len);
											
											plot_buffer.read_idx = (plot_buffer.read_idx + 1) % PLOT_BUFFER_SIZE;
											
											__disable_irq();
											plot_buffer.count--;
											__enable_irq();
											
											count_element++;
									}
							}
						if(trajectory.phase == TRAJ_PHASE_STOP) {  // Đã gần đích
								Motor.tick_enabled = 1;
								
								if(Motor.tick >= 400){  // 400 * 5ms = 2s
										// Sau 2s mà vẫn sai số > 10mm => CÓ LỖI
										if(fabs(trajectory.q_desired - Motor.CurPos_mm) > 10.0) {
												plot_buffer.reset = 1;
												PIDVel.err_state = 1;
										}
										else {
												// Đã ổn định => tắt tick
												Motor.check = 0;
												Motor.tick_enabled = 0;
										}
										Motor.tick = 0;
								}
						}
						else {
								// Động cơ đang chuyển động hoặc chưa gần đích => KHÔNG kiểm tra lỗi
								Motor.tick_enabled = 0;
								Motor.tick = 0;
						}
						break;
					case MOTOR_P:
							if (plot_buffer.count > 0 && !dma_busy) {
								PlotData_t *data = &plot_buffer.buffer[plot_buffer.read_idx];
								
								int len = snprintf(Tx_Buffer, MaxBuffer,
																	"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f 0.0\r\n",
																	data->q_desired,
																	data->q_actual,
																	0.0,
																	0.0,
																	data->v_motor,
																	0.0,
																	data->a_motor,
																	data->pwm
																	);
								
								if (len > 0 && len < MaxBuffer) {
										dma_busy = 1;
										HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Tx_Buffer, len);
										
										plot_buffer.read_idx = (plot_buffer.read_idx + 1) % PLOT_BUFFER_SIZE;
										
										__disable_irq();
										plot_buffer.count--;
										__enable_irq();
										
										chieu += 1.0;
								}
						}
						break;
				}
				if(Motor.DesiredPos_mm != Motor.DesiredPos_mm_pre){
						trajectory.plot_active = 1;
						trajectory.move_active = 1;
						Motor.plot_active = 1;
						Motor.tick_enabled = 0;
						Motor.tick = 0;
						Motor.DesiredPos_mm_pre = Motor.DesiredPos_mm;
								TRAJ_SetupMultiSegmentMove_100mm(&trajectory, Motor.DesiredPos_mm);
								TRAJ_ContinueNextSegment_100mm(&trajectory);
						PID_ResetState(&PIDPos);
				}
				if(trajectory.phase == TRAJ_PHASE_COMPLETE && 
					 (fabs(Motor.DesiredPos_mm - Motor.CurPos_mm)) <= Motor.cycle_error[Motor.cycle_index] && 
					 Motor.auto_run)
				{
						if(Motor.cycle_count > 0){
								// Kiểm tra đang ở vị trí nào
								if(fabs(Motor.DesiredPos_mm_pre) < 0.1){
										// Đang ở vị trí 0 → Đi đến vị trí đích
										Motor.DesiredPos_mm = Motor.AutoDesiredPos_mm;
								} else {
										// Đang ở vị trí đích → Quay về 0
										Motor.DesiredPos_mm = 0;
										
										// Giảm cycle sau khi hoàn thành 1 chu kỳ đầy đủ (đi + về)
										Motor.cycle_count--;
								}
						}
						else {
								// Hết cycle → Dừng auto_run
								Motor.auto_run = 0;
								Motor.DesiredPos_mm_pre = 0;
						}
				}
				#endif
				HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
struct __FILE
{
  int handle;
  
};
/* FILE is typedef'd in stdio.h. */
FILE __stdout;
int fputc(int ch, FILE *f)
{
  /* Your implementation of fputc(). */
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch,1,100);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) &ch,1);

  return ch;
}
// Ham ngat Uart
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1)
    {
        UART_Receive_Enable(&huart1, (uint8_t *)Rx_Buffer, MaxBuffer);
        
				char first_char = Rx_Buffer[0];
				if(first_char == 'r'){
						plot_buffer.reset = 1;
						PIDVel.err_state = 0;
						return;
				}
				memcpy(Rx_data, Rx_Buffer, MaxBuffer * sizeof(Rx_Buffer[0]));
				switch(first_char){
					case 'm':
								MOTOR_ResetParams(&Motor);
								ENCODER_ResetParams(&Encoder);
								TRAJ_ResetParams(&trajectory);
								if(strcmp(Rx_Buffer, "mode 0") == 0){
										Motor.cycle_index = 1;
										MOTOR_UpdateMode(&Motor, MOTOR_P);
										PID_SetParamsImproved(&PIDPos, Kp_pos, SYS_MaxPWM_P, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
								}
								else if(strcmp(Rx_Buffer, "mode 1") == 0){
										Kpp_improved = 5.0;
										trajectory.use_v1 = 0;
										Motor.cycle_index = 0;
										MOTOR_UpdateMode(&Motor, MOTOR_P_PI);
										PID_SetParamsImproved(&PIDPos, Kpp_improved, Motor.v_max_mm, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
								}
								else if(strcmp(Rx_Buffer, "mode 2") == 0){
										trajectory.use_v1 = 1;
										Motor.cycle_index = 0;
										MOTOR_UpdateMode(&Motor, MOTOR_P_PI);
										PID_SetParamsImproved(&PIDPos, Kpp_improved, Motor.v_max_mm, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
								}
								else{
										MOTOR_UpdateMode(&Motor, MOTOR_SetPos);
										trajectory.sign = 1;
								}
								plot_buffer.reset = 1;
								break;
					case 'c':
								if(Motor.mode == MOTOR_SetPos){
										if(strcmp(Rx_Buffer, "c-1") == 0){
												chieu = -1;
										}
										else if(strcmp(Rx_Buffer, "c0") == 0){
												chieu = 0;
										}
										else{
												chieu = 1;
										}
								}
					case 'x':
										{
											char *token1 = strtok(Rx_Buffer + 1, "x");  // Bỏ ký tự 'x' đầu
											char *token2 = strtok(NULL, "x");
											
											if (token1 != NULL && token2 != NULL) {
													int position = atof(token1);
													int cycles = atoi(token2);
													
													// Lưu vào struct trajectory hoặc motor
													MOTOR_UpdatePos(&Motor, position);
													Motor.AutoDesiredPos_mm = Motor.DesiredPos_mm;
													Motor.cycle_count = cycles;
													Motor.auto_run = 1;  // Bật chế độ chu trình
											}
										}
										break;
					default:
								break;
				}
				
				if(Motor.mode != MOTOR_SetPos){
						// Lấy ký tự cuối cùng làm command
						char last_char = Rx_Buffer[Size - 1];
						
						switch(last_char) {
								case 's':
										// Thay ký tự 's' bằng null terminator để atoi chỉ đọc phần số
										Rx_Buffer[Size - 1] = '\0';
										MOTOR_UpdatePos(&Motor, atoi(Rx_Buffer));
										break;
										
								case 'v':
										if(Motor.mode == MOTOR_P) break;
										// Thay ký tự 'v' bằng null terminator để atoi chỉ đọc phần số
										Rx_Buffer[Size - 1] = '\0';
										MOTOR_UpdateVel(&Motor, atoi(Rx_Buffer));
										
										PID_SetParamsImproved(&PIDPos, Kpp_improved, Motor.v_max_mm, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
										TRAJ_SetParams(&trajectory, SYS_SAMPLETIME, SYS_RUN_TIME, SYS_STOP_TIME, Motor.a_max_mm, Motor.v_max_mm);
										break;    
										
								case 'p':
										Rx_Buffer[Size - 1] = '\0';
										if(Motor.mode == MOTOR_P){
												Kp_pos = atof(Rx_Buffer);
												PID_SetParamsImproved(&PIDPos, Kp_pos, SYS_MaxPWM_P, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
										}
										else{
												Kpp_improved = atof(Rx_Buffer);
												PID_SetParamsImproved(&PIDPos, Kpp_improved, Motor.v_max_mm, SYS_SAMPLETIME, Kest, Kcom, Motor.a_max_mm);
										}
								case 'e':
										if(Motor.mode == MOTOR_P) break;
										Rx_Buffer[Size - 1] = '\0';
										if(atoi(Rx_Buffer) == 0)	PID_DisableImproved(&PIDPos);
										else PID_EnableImproved(&PIDPos);
										break;

								default:
										break;
						}
				}
        
        // Reset buffer
        memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == Encoder_A || GPIO_Pin == Encoder_B) {
        ProcessEncoder(&Encoder, Encoder_A, Encoder_B);
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
