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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define QUEUE_SIZE 10
#define WINDOW_SIZE 10
#define CAMERA_LIDAR_HZ_RATIO 3 // Lidar 10hz -> Camera 30hz
#define PWM_WIDTH 1000 // 1000dus = 10000us = 10ms
#define INITIALIZE_TOLERANCE_SEC 5
#define RX_BUFFER_SIZE 2
#define OVER_OVERFLOW_VALUE 5965232
#define CONVERGE_DELAY_THD 50 // [dus] // 50dus -> 0.5ms

//======================================================================================================================================
/* Class Sliding Window for LidarInfo */
typedef struct{
	int32_t lidar_time_arr[WINDOW_SIZE];
	int32_t interval_arr[WINDOW_SIZE - 1];
	int lidar_time_size;
	int interval_size;
	int32_t sum_of_interval;
}LidarInfo;

void pushbackLiDAR(LidarInfo* lidar__, int32_t nVar)
{
	// lidar time
	if(lidar__->lidar_time_size >= WINDOW_SIZE)
	{
		for(int i = 0; i < WINDOW_SIZE - 1; i++)
		{
			lidar__->lidar_time_arr[i] = lidar__->lidar_time_arr[i+1];
		}
		lidar__->lidar_time_arr[WINDOW_SIZE - 1] = nVar;
	}
	else
	{
		lidar__->lidar_time_arr[lidar__->lidar_time_size] = nVar;
		lidar__->lidar_time_size++;
	}

	// interval
	if(lidar__->interval_size >= WINDOW_SIZE - 1)
	{
		lidar__->sum_of_interval -= lidar__->interval_arr[0];
		for(int i = 0; i < WINDOW_SIZE - 2; i++)
		{
			lidar__->interval_arr[i] = lidar__->interval_arr[i+1];
		}
		lidar__->interval_arr[WINDOW_SIZE - 2] = (lidar__->lidar_time_arr[WINDOW_SIZE - 1] - lidar__->lidar_time_arr[WINDOW_SIZE - 2]);
		lidar__->sum_of_interval += lidar__->interval_arr[WINDOW_SIZE - 2];
	}
	else
	{
		if(lidar__->lidar_time_size > 1)
		{
			lidar__->interval_arr[lidar__->interval_size] = lidar__->lidar_time_arr[lidar__->interval_size + 1] - lidar__->lidar_time_arr[lidar__->interval_size];
			lidar__->sum_of_interval += lidar__->interval_arr[lidar__->interval_size];
			lidar__->interval_size++;
		}
	}
}

void compensateLiDAR(LidarInfo* lidar__, int32_t nTime)
{
	for(int i = 0; i < WINDOW_SIZE; i++)
	{
		lidar__->lidar_time_arr[i] += nTime;
	}
}

//======================================================================================================================================
/* Class Queue */
typedef struct queue
{
	int32_t buffer_[QUEUE_SIZE];
	int head;
	int tail;
}queue;

void queue_push(queue* queue_, int32_t var)
{
	queue_->buffer_[queue_->head] = var;
	queue_->head++;

	if(queue_->head >= QUEUE_SIZE)
	{
		queue_->head = 0;
	}
}
int32_t queue_pop(queue* queue_)
{
	int32_t pop_data = queue_->buffer_[queue_->tail];
	queue_->tail++;

	if(queue_->tail >= QUEUE_SIZE)
	{
		queue_->tail = 0;
	}
	return pop_data;
}
int queue_is_empty(queue* queue_)
{
	if(queue_->head == queue_->tail)return 1;
	else return 0;
}
int32_t get_queue(queue* queue_)
{
	return queue_->buffer_[queue_->tail];
}

//======================================================================================================================================

uint8_t rx_buffer[RX_BUFFER_SIZE];
int32_t rx_delay_buffer[RX_BUFFER_SIZE];
int32_t rx_delay_data;
int32_t time_lidar_ros_offset = 0;
int32_t delay_gain = 0;

LidarInfo* lidar;

int32_t curr_time = 0;
int32_t curr_cycle_time = 0;
int32_t prev_cycle_time = 0;
int32_t overflow_count = 0;
int32_t tmp_curr_time = 0;

int initialize_stack = 0;
int32_t tmp_lidar_time_for_init = 0;

int32_t last_lidar_time = 0;
int32_t predicted_lidar_time = 0;
int is_converge = 0;

queue* q_camera_rising_edge_time;
queue* q_camera_faling_edge_time;

int32_t for_debugging = 0;
int32_t for_debugging2 = 0;

void DWT_Init(void); //
int32_t DWT_GetDecaMicroseconds(void); // 10us clock timer
void reInitialize(void);

// 0.5ms period
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// check LiDAR Time
	if(tmp_lidar_time_for_init == lidar->lidar_time_arr[WINDOW_SIZE - 1])initialize_stack++;
	else initialize_stack = 0;
	if (initialize_stack >= INITIALIZE_TOLERANCE_SEC*2000)reInitialize();
	tmp_lidar_time_for_init = lidar->lidar_time_arr[WINDOW_SIZE - 1];
//	int estimation_flag = last_lidar_time <= lidar.lidar_time_arr[WINDOW_SIZE - 1] ? 1 : 0;

	// output rising edge
	tmp_curr_time = 0;
	int32_t tmp_DWT_time = DWT_GetDecaMicroseconds();
	if(prev_cycle_time > tmp_DWT_time)tmp_curr_time += OVER_OVERFLOW_VALUE; // overflow check
	tmp_curr_time += overflow_count*OVER_OVERFLOW_VALUE + tmp_DWT_time; // get current time

	// Pseudo PWM
	if(!queue_is_empty(q_camera_rising_edge_time))
	{
		int32_t rising_edge_time = get_queue(q_camera_rising_edge_time);
		int32_t faling_edge_time = get_queue(q_camera_faling_edge_time);
		if(tmp_curr_time >= rising_edge_time && tmp_curr_time <= faling_edge_time)
		{
			// GPIO On
			HAL_GPIO_WritePin(GPIOC, CAMERA_PWM_Pin, GPIO_PIN_SET);
		}
		if(tmp_curr_time >= faling_edge_time)
		{
			// GPIO Off
			HAL_GPIO_WritePin(GPIOC, CAMERA_PWM_Pin, GPIO_PIN_RESET);
			queue_pop(q_camera_rising_edge_time);
			queue_pop(q_camera_faling_edge_time);
		}
	}
}

void reInitialize(void)
{
	initialize_stack = 0;

	curr_time = 0;
	curr_cycle_time = 0;
	prev_cycle_time = 0;
	overflow_count = 0;

	for(int i = 0; i < WINDOW_SIZE; i++)lidar->lidar_time_arr[i] = 0;
	for(int i = 0; i < WINDOW_SIZE - 1; i++)lidar->interval_arr[i] = 0;
	lidar->lidar_time_size = 0;
	lidar->interval_size = 0;
	lidar->sum_of_interval = 0;

	last_lidar_time = 0;
	predicted_lidar_time = 0;

	for(int i = 0; i < QUEUE_SIZE; i++)q_camera_rising_edge_time->buffer_[i] = 0;
	q_camera_rising_edge_time->head = 0;
	q_camera_rising_edge_time->tail = 0;
	for(int i = 0; i < QUEUE_SIZE; i++)q_camera_faling_edge_time->buffer_[i] = 0;
	q_camera_faling_edge_time->head = 0;
	q_camera_faling_edge_time->tail = 0;

	rx_delay_data = 0;
	time_lidar_ros_offset = 0;
	delay_gain = 0;
	is_converge = 0;
	HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_RESET);
}


// UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance==USART2)
	{
		// get uart data
		HAL_UART_Receive_IT(&huart2, rx_delay_buffer, RX_BUFFER_SIZE*4);
		time_lidar_ros_offset = rx_delay_buffer[0];
		rx_delay_data = rx_delay_buffer[1];

		// get current time
		curr_cycle_time = DWT_GetDecaMicroseconds();
		if(curr_cycle_time < prev_cycle_time)overflow_count++; // overflow check
//		curr_time = overflow_count*5969399 + curr_cycle_time;
//		curr_time = overflow_count*5965232 + curr_cycle_time;
		curr_time = overflow_count*OVER_OVERFLOW_VALUE + curr_cycle_time;

		// led toggle for debugging
		if(is_converge == 0) HAL_GPIO_TogglePin(GPIOA, LED_2_Pin);
		else if(is_converge == 1) HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_SET);

		// gain
		if(lidar->lidar_time_size == WINDOW_SIZE)
		{
			if(rx_delay_data > 0)
			{
				delay_gain += 10; // 10dus -> 100us -> 0.1ms
				compensateLiDAR(lidar, 10);
			}
			else if(rx_delay_data < 0)
			{
				delay_gain -= 10;;
				compensateLiDAR(lidar, -10);
			}
//			delay_gain += rx_delay_data;
//			compensateLiDAR(lidar, rx_delay_data);

			// check converge
			if(rx_delay_data < CONVERGE_DELAY_THD && rx_delay_data > -CONVERGE_DELAY_THD && rx_delay_data != 0)
			{
				is_converge = 1;
			}
//			if(is_converge == 1 && (rx_delay_data > CONVERGE_DELAY_THD || rx_delay_data < -CONVERGE_DELAY_THD))
//			{
//				is_converge = 0;
//			}
		}

		// Compensate LiDAR ros time and send time differrence
		curr_time -= time_lidar_ros_offset;

		// Compensate Serial Communication time (8byte -> 8.333ms -> 8333us -> 833.3dus)
		curr_time -= 833;

		// compensate delay
		curr_time += delay_gain;
		for_debugging2 = delay_gain;

		// time push
		pushbackLiDAR(lidar, curr_time);

		// estimation LiDAR Time
		last_lidar_time = lidar->lidar_time_arr[WINDOW_SIZE - 1];
		int32_t predicted_lidar_interval = lidar->sum_of_interval/(WINDOW_SIZE - 1);

		predicted_lidar_time = last_lidar_time + 2*predicted_lidar_interval;

		int32_t real_predicted_lidar_time = predicted_lidar_time;

		// if not converge, LiDAR Hz == Camera Hz
		if(is_converge == 0)
		{
			// Calculate Camera Capture Time
			int32_t predicted_camera_interval = predicted_lidar_interval;

			int32_t rising_edge_time = real_predicted_lidar_time;
			int32_t faling_edge_time = rising_edge_time + PWM_WIDTH;

			queue_push(q_camera_rising_edge_time, rising_edge_time);
			queue_push(q_camera_faling_edge_time, faling_edge_time);
		}
		// if converge
		else if(is_converge == 1)
		{
			// Calculate Camera Capture Time
			int32_t predicted_camera_interval = predicted_lidar_interval/CAMERA_LIDAR_HZ_RATIO;

			//
			for(int i = 0; i < CAMERA_LIDAR_HZ_RATIO; i++)
			{
				int32_t rising_edge_time = real_predicted_lidar_time - (CAMERA_LIDAR_HZ_RATIO - i - 1)*predicted_camera_interval;
				int32_t faling_edge_time = rising_edge_time + PWM_WIDTH;

				queue_push(q_camera_rising_edge_time, rising_edge_time);
				queue_push(q_camera_faling_edge_time, faling_edge_time);
			}
		}

		prev_cycle_time = curr_cycle_time;
	}
}

// clock
int32_t DWT_GetDecaMicroseconds(void)
{
	// if run, SystemCoreClock is 72MHz
    return DWT->CYCCNT / (SystemCoreClock / 100000); // 10us unit
}
void DWT_Init(void)
{
	// Activate DWT cycle counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0; // Initialize Cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Activate Cycle counter
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, rx_delay_buffer, RX_BUFFER_SIZE*4);
  DWT_Init();
  lidar = (LidarInfo*)malloc(sizeof(LidarInfo));
  q_camera_rising_edge_time = (queue*)malloc(sizeof(queue));
  q_camera_faling_edge_time = (queue*)malloc(sizeof(queue));
  reInitialize();
  /* @@@@@@@@@@@@@@@@@@@@@@@
  @@@  TIMER 2 - 2000Hz  @@@
  @@@@@@@@@@@@@@@@@@@@@@@ */
  HAL_TIM_Base_Start_IT(&htim2);

  /* @@@@@@@@@@@@@@@@@@@@@@@@
  @@@  TIMER 2 - 10000Hz  @@@
  @@@@@@@@@@@@@@@@@@@@@@@@ */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_PWM_GPIO_Port, CAMERA_PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERA_PWM_Pin */
  GPIO_InitStruct.Pin = CAMERA_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CAMERA_PWM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
