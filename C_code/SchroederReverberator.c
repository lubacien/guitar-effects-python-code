/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdbool.h>
#include <math.h>
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
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile int32_t timer_value_us;

#define START_TIMER {\
  HAL_TIM_Base_Init(&htim2);\
  HAL_TIM_Base_Start(&htim2); }

#define STOP_TIMER {\
  timer_value_us = __HAL_TIM_GET_COUNTER(&htim2);\
  HAL_TIM_Base_Stop(&htim2); }

#define COS_TABLE_LEN 80
static int16_t COS_TABLE[COS_TABLE_LEN] = {
    0x7FFF, 0x7F99, 0x7E6B, 0x7C75, 0x79BB, 0x7640, 0x720B, 0x6D22, 0x678D, 0x6154, 0x5A81, 0x5320,
    0x4B3B, 0x42E0, 0x3A1B, 0x30FB, 0x278D, 0x1DE1, 0x1405, 0x0A0A, 0x0000, 0xF5F6, 0xEBFB, 0xE21F,
    0xD873, 0xCF05, 0xC5E5, 0xBD20, 0xB4C5, 0xACE0, 0xA57F, 0x9EAC, 0x9873, 0x92DE, 0x8DF5, 0x89C0,
    0x8645, 0x838B, 0x8195, 0x8067, 0x8001, 0x8067, 0x8195, 0x838B, 0x8645, 0x89C0, 0x8DF5, 0x92DE,
    0x9873, 0x9EAC, 0xA57F, 0xACE0, 0xB4C5, 0xBD20, 0xC5E5, 0xCF05, 0xD873, 0xE21F, 0xEBFB, 0xF5F6,
    0x0000, 0x0A0A, 0x1405, 0x1DE1, 0x278D, 0x30FB, 0x3A1B, 0x42E0, 0x4B3B, 0x5320, 0x5A81, 0x6154,
    0x678D, 0x6D22, 0x720B, 0x7640, 0x79BB, 0x7C75, 0x7E6B, 0x7F99};
#define SAMPLES_PER_FRAME 2   /* stereo signal */
#define FRAMES_PER_BUFFER 3200//16383  /* user-defined. We have 320kB RAM-> max is 20 000 */
//but since size of full buffer has to be an unsigned int 16 it cannot be more than 65536, therefore max
//frames per buffer is 65536/4 -1 = 16384 -1
#define HALF_BUFFER_SIZE SAMPLES_PER_FRAME*FRAMES_PER_BUFFER
#define FULL_BUFFER_SIZE 2*HALF_BUFFER_SIZE
#define QUARTER_BUFFER_SIZE HALF_BUFFER_SIZE/2
#define SAMPLING_FREQ ((uint16_t) 32000)
#define DELAY_TIMEC1 ((float) 0.03) //0.5 //less than 15ms delay is perceived in the spectral domain.
#define DELAY_TIMEC2 ((float) 0.037)
#define DELAY_TIMEC3 ((float) 0.042)
#define DELAY_TIMEC4 ((float) 0.045)

#define DELAY_TIME3 ((float) 0.005)
#define DELAY_TIME4 ((float) 0.0017)


#define M_DELAY_COMB1 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIMEC1/2)*2)
#define M_DELAY_COMB2 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIMEC2/2)*2)
#define M_DELAY_COMB3 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIMEC3/2)*2)
#define M_DELAY_COMB4 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIMEC4/2)*2)
#define M_DELAY3 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIME3))
#define M_DELAY4 ((int16_t) floor(SAMPLES_PER_FRAME * SAMPLING_FREQ * DELAY_TIME4))

#define maxnum 0x2FFFF

#define GAIN1 ((float) 0.9)
#define GAIN2 ((float) 0.9)
#define GAINC3 ((float) 0.9)
#define GAINC4 ((float) 0.9)
#define GAIN3 ((float) 0.7)
#define GAIN4 ((float) 0.7)

// we need M_DELAY < HALF_BUFFER_SIZE, which means the maximum delay time for frames_per_buffer = 20000
// is 0.625
//maxdelaytime = maxframesperbuffer/samp_freq around 0.5


//sqrt(1-pow(GAIN,2.0))//this ensures good normalisation ( or sqrt(1-g^2))
int16_t dma_in[FULL_BUFFER_SIZE];
int16_t dma_out[FULL_BUFFER_SIZE];
int16_t buffer[HALF_BUFFER_SIZE];
int16_t buffer3[HALF_BUFFER_SIZE];
int16_t buffer4[HALF_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Process(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp);

void Process3(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp);
void Process4(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {


	Process(dma_in, dma_out, HALF_BUFFER_SIZE, 1);
	//Process3(dma_out, dma_out, HALF_BUFFER_SIZE, 1);
	Process4(dma_out, dma_out, HALF_BUFFER_SIZE, 1);

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {

START_TIMER
  Process(dma_in + HALF_BUFFER_SIZE, dma_out + HALF_BUFFER_SIZE, HALF_BUFFER_SIZE,0);
  //Process3(dma_out + HALF_BUFFER_SIZE, dma_out + HALF_BUFFER_SIZE, HALF_BUFFER_SIZE,0);
  Process4(dma_out + HALF_BUFFER_SIZE, dma_out + HALF_BUFFER_SIZE, HALF_BUFFER_SIZE,0);

  STOP_TIMER
}
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
  MX_I2S1_Init();
  MX_I2S2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*) dma_out, FULL_BUFFER_SIZE);
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) dma_in, FULL_BUFFER_SIZE);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void inline Process(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp) {

	static int16_t x_prev = 0;
    static int16_t ynd1=0;
    static int16_t xnd1=0;
    static int16_t ynd2=0;
    static int16_t xnd2=0;
    static int16_t ynd3=0;
    static int16_t xnd3=0;
    static int16_t ynd4=0;
    static int16_t xnd4=0;

    static int16_t buffercomb1[HALF_BUFFER_SIZE];
    static int16_t buffercomb2[HALF_BUFFER_SIZE];
    static int16_t buffercomb3[HALF_BUFFER_SIZE];
    static int16_t buffercomb4[HALF_BUFFER_SIZE];

    static float y=0;

    static int16_t y16 =0;
	for (uint16_t j = 0; j < size; j += 2) {
	    int16_t j2 = j/2;

		y = (int16_t)(*(pIn+j) - x_prev);
	    x_prev = *(pIn+j);

		if(!halfcomp){
			ynd1 = buffercomb1[j2-M_DELAY_COMB1/2+QUARTER_BUFFER_SIZE];
			xnd1 = *(pIn+j-M_DELAY_COMB1);
			buffercomb1[j2+QUARTER_BUFFER_SIZE] = xnd1 + ynd1*GAIN1;

			ynd2 = buffercomb2[j2-M_DELAY_COMB2/2+QUARTER_BUFFER_SIZE];
			xnd2 = *(pIn+j-M_DELAY_COMB2);
			buffercomb2[j2+QUARTER_BUFFER_SIZE] = xnd2 + ynd2*GAIN2;

			ynd3 = buffercomb3[j2-M_DELAY_COMB3/2+QUARTER_BUFFER_SIZE];
			xnd3 = *(pIn+j-M_DELAY_COMB3);
			buffercomb3[j2+QUARTER_BUFFER_SIZE] = xnd3 + ynd3*GAIN3;

			ynd4 = buffercomb4[j2-M_DELAY_COMB4/2+QUARTER_BUFFER_SIZE];
			xnd4 = *(pIn+j-M_DELAY_COMB4);
			buffercomb4[j2+QUARTER_BUFFER_SIZE] = xnd4 + ynd4*GAIN4;

			y16 = buffercomb1[j2+QUARTER_BUFFER_SIZE] +buffercomb2[j2+QUARTER_BUFFER_SIZE]+buffercomb3[j2+QUARTER_BUFFER_SIZE]+buffercomb4[j2+QUARTER_BUFFER_SIZE];
			// faster than xnd1 + ynd1*GAIN1 + xnd2 + ynd2*GAIN2 + xnd3 + ynd3*GAINC3 + xnd4 + ynd4*GAINC4; //gain is 0.5 -> we do /2, but the DC notch attenuates already
			buffer[j2+QUARTER_BUFFER_SIZE] = y16;
		}
		else
		{

			if (j >= M_DELAY_COMB1){
				ynd1 = buffercomb1[j2-M_DELAY_COMB1/2];
				xnd1 = *(pIn+j-M_DELAY_COMB1);
				buffercomb1[j2] = xnd1 + ynd1*GAIN1;
			}
			else{
				ynd1 = buffercomb1[j2-M_DELAY_COMB1/2+HALF_BUFFER_SIZE];
				xnd1 = *(pIn+j-M_DELAY_COMB1+FULL_BUFFER_SIZE);
				buffercomb1[j2] = xnd1 + ynd1*GAIN1;
			}


			if (j >= M_DELAY_COMB2){
				ynd2 = buffercomb2[j2-M_DELAY_COMB2/2];
				xnd2 = *(pIn+j-M_DELAY_COMB2);
				buffercomb2[j2] = xnd2 + ynd2*GAIN2;
			}
			else{
				ynd2 = buffercomb2[j2-M_DELAY_COMB2/2+HALF_BUFFER_SIZE];
				xnd2 = *(pIn+j-M_DELAY_COMB2+FULL_BUFFER_SIZE);
				buffercomb2[j2] = xnd2 + ynd2*GAIN2;
			}


			if (j >= M_DELAY_COMB3){
				ynd3 = buffercomb3[j2-M_DELAY_COMB3/2];
				xnd3 = *(pIn+j-M_DELAY_COMB3);
				buffercomb3[j2] = xnd3 + ynd3*GAIN3;
			}
			else{
				ynd3 = buffercomb3[j2-M_DELAY_COMB3/2+HALF_BUFFER_SIZE];
				xnd3 = *(pIn+j-M_DELAY_COMB3+FULL_BUFFER_SIZE);
				buffercomb3[j2] = xnd3 + ynd3*GAIN3;
			}

			if (j >= M_DELAY_COMB4){
				ynd4 = buffercomb4[j2-M_DELAY_COMB4/2];
				xnd4 = *(pIn+j-M_DELAY_COMB4);
				buffercomb4[j2] = xnd4 + ynd4*GAIN4;
			}
			else{
				ynd4 = buffercomb4[j2-M_DELAY_COMB4/2+HALF_BUFFER_SIZE];
				xnd4 = *(pIn+j-M_DELAY_COMB4+FULL_BUFFER_SIZE);
				buffercomb4[j2] = xnd4 + ynd4*GAIN4;
			}

			y16 = buffercomb1[j2] +buffercomb2[j2]+buffercomb3[j2]+buffercomb4[j2];
			buffer[j2] = y16;
		}
	}
}

void inline Process3(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp) {

    static int16_t ynd=0;
    static int16_t xnd=0;
    static int16_t y=0;

	for (uint16_t j = 0; j < size; j += 2) {
		int16_t j2 = j/2;

		if(!halfcomp){
			y = buffer[j2+ QUARTER_BUFFER_SIZE];
			ynd = buffer3[j2-M_DELAY3/2+QUARTER_BUFFER_SIZE];
			xnd = buffer[j2-M_DELAY3/2+QUARTER_BUFFER_SIZE];
			y = xnd - y*GAIN3 + ynd*GAIN3;
			buffer3[j2+QUARTER_BUFFER_SIZE] = y;
		}
		else
		{
			y = buffer[j2];

			if (j >= M_DELAY3){
				ynd = buffer3[j2-M_DELAY3/2];
				xnd = buffer[j2-M_DELAY3/2];
				y = xnd - y*GAIN3 + ynd*GAIN3;
				buffer3[j2] = y;
			}

			else{
				ynd = buffer3[j2-M_DELAY3/2+HALF_BUFFER_SIZE];
				xnd = buffer[j2-M_DELAY3/2+HALF_BUFFER_SIZE];
				y = xnd - y*GAIN3 + ynd*GAIN3;
				buffer3[j2] = y;
			}
		}
	}
}
void inline Process4(int16_t *pIn, int16_t *pOut, uint16_t size, bool halfcomp) {
    static int16_t ynd=0;
    static int16_t xnd=0;
    static int16_t xn=0;
    static int16_t y16 = 0;
    static float y=0;

	for (uint16_t j = 0; j < size; j += 2) {
		int16_t j2 = j/2;

		if(!halfcomp){
			xn = buffer[j2+ QUARTER_BUFFER_SIZE];

			ynd = buffer4[j2-M_DELAY4/2+QUARTER_BUFFER_SIZE];
			xnd = buffer[j2-M_DELAY4/2+QUARTER_BUFFER_SIZE];
			y = xnd - xn*GAIN4 + ynd*GAIN4;
			int16_t y16 = (y/maxnum) *0xFFF;
			buffer4[j2+QUARTER_BUFFER_SIZE] = y16;
		}
		else
		{
			xn = buffer[j2];

			if (j >= M_DELAY4){
				ynd = buffer4[j2-M_DELAY4/2];
				xnd = buffer[j2-M_DELAY4/2];
				y = xnd - xn*GAIN4 + ynd*GAIN4;
				int16_t y16 = (y/maxnum) *0xFFF;
				buffer4[j2] = y16;
			}

			else{
				ynd = buffer4[j2-M_DELAY4/2+HALF_BUFFER_SIZE];
				xnd = buffer[j2-M_DELAY4/2+HALF_BUFFER_SIZE];
				y = xnd - xn*GAIN4 + ynd*GAIN4;
				int16_t y16 = (y/maxnum) *0xFFF;
				buffer4[j2] = y16;
			}
		}
	}

	if(!halfcomp){
		for (uint16_t k = 0; k < size/2; k += 1) {
			*(pOut+2*k) = buffer4[k+QUARTER_BUFFER_SIZE];
			*(pOut+2*k+1) = buffer4[k+QUARTER_BUFFER_SIZE];
		}
	}
	else{
		for (uint16_t k = 0; k < size/2; k += 1) {
			*(pOut+2*k) = buffer4[k];
			*(pOut+2*k+1) = buffer4[k];
		}
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
