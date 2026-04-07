/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define L 1 //left
#define R 2 //right
#define F 0 //front
#define None -1 //none
#define NS -1 //not specified

#define BATCH_SIZE 10

struct Timer_S {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
};

struct Timers_S {
  struct Timer_S left_plus;
  struct Timer_S left_minus;
  struct Timer_S right_plus;
  struct Timer_S right_minus;
};

volatile uint32_t gQtr_L_val = 0, gQtr_R_val = 0;
volatile uint32_t gSharp_L_val = 0, gSharp_M_val = 0, gSharp_R_val = 0;
volatile uint32_t gAvg_sharp_L = 0, gAvg_sharp_M = 0, gAvg_sharp_R = 0;

volatile int32_t gSharp_L_vals[BATCH_SIZE], gSharp_M_vals[BATCH_SIZE], gSharp_R_vals[BATCH_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmit_string(char str[])
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
}


void transmit_int(char name[], uint32_t value)
{
  char buffer[32];
  int len = sprintf(buffer, "%s: %u  ", name, value);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 10);
}

void flush(){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    volatile uint32_t trash = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1); // <--- IMPORTANT: Stop it so we can restart cleanly
}

volatile uint32_t get_reading(void){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    volatile uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

void init(struct Timers_S timers)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    
    HAL_TIM_PWM_Start(timers.left_minus.htim, timers.left_minus.channel);
    HAL_TIM_PWM_Start(timers.right_minus.htim, timers.right_minus.channel);
    HAL_TIM_PWM_Start(timers.left_plus.htim, timers.left_plus.channel);
    HAL_TIM_PWM_Start(timers.right_plus.htim, timers.right_plus.channel);
}

void reset(struct Timers_S timers)
{
    HAL_TIM_PWM_Stop(timers.left_minus.htim, timers.left_minus.channel);
    HAL_TIM_PWM_Stop(timers.right_minus.htim, timers.right_minus.channel);
    HAL_TIM_PWM_Stop(timers.left_plus.htim, timers.left_plus.channel);
    HAL_TIM_PWM_Stop(timers.right_plus.htim, timers.right_plus.channel);

    HAL_ADC_Stop(&hadc1);
}

bool isBtn_on(void)
{
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin);
    
    transmit_int("mono button", pin_state);
    transmit_int("pin set", GPIO_PIN_SET);
    
    if (pin_state != GPIO_PIN_SET) 
    {
        return false;
    }
    return true;
}

void get_pololu_readings(void){
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;

    sConfig.Channel = ADC_CHANNEL_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    flush();
    gQtr_L_val = get_reading();

    sConfig.Channel = ADC_CHANNEL_6;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    flush();
    gQtr_R_val = get_reading();
}

void get_sharp_readings(void){
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;

    sConfig.Channel = ADC_CHANNEL_15;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

    flush();
    
    gSharp_L_val = get_reading();

    sConfig.Channel = ADC_CHANNEL_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

    flush();

    gSharp_M_val = get_reading();

    sConfig.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

    flush();

    gSharp_R_val = get_reading();
}

void get_avg_sharp_readings()
{
  for(int i = 0; i < BATCH_SIZE; ++i)
  {
      get_sharp_readings();

      gSharp_L_vals[i] = gSharp_L_val;
      gSharp_M_vals[i] = gSharp_M_val;
      gSharp_R_vals[i] = gSharp_R_val;
  }

  gAvg_sharp_L = 0;
  gAvg_sharp_M = 0; 
  gAvg_sharp_R = 0;

  for(int i = 0; i < BATCH_SIZE; ++i)
  {
    gAvg_sharp_L += gSharp_L_vals[i];
    gAvg_sharp_M += gSharp_M_vals[i];
    gAvg_sharp_R += gSharp_R_vals[i];
  }

  gAvg_sharp_L /= BATCH_SIZE;
  gAvg_sharp_M /= BATCH_SIZE;
  gAvg_sharp_R /= BATCH_SIZE;
}

void transmit_sharp_readings(void)
{
    transmit_int("sharp1", gAvg_sharp_L);
    transmit_int("sharp2", gAvg_sharp_M);
    transmit_int("sharp3", gAvg_sharp_R);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof('\n'), 10);
}

void transmit_qtr_readings(void)
{
  transmit_int("qtr1", gQtr_L_val);
  transmit_int("qtr2", gQtr_R_val);
  HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof('\n'), 10);
}

bool isLine_found()
{
    if(gQtr_L_val < 1500 || gQtr_R_val < 1500)
    {
      return true;
    }
    return false;
}

int get_enemy_position(void)
{
  if(gAvg_sharp_L < 1000 && gAvg_sharp_M >= 1000 && gAvg_sharp_R < 1000) 
  {
    return F;
  }
  else if(gAvg_sharp_L >= 1000 && gAvg_sharp_R >= 1000) 
  {
    return F;
  }
  else if(gAvg_sharp_R >= 1000) 
  {
    return R;
  }
  else if(gAvg_sharp_L >= 1000) 
  {
    return L;
  }
  else 
  {
    return None;
  }
}

void set_all_motors(struct Timers_S timers, uint32_t left_plus, uint32_t right_plus, uint32_t left_minus, uint32_t right_minus)
{
  __HAL_TIM_SET_COMPARE(timers.left_plus.htim, timers.left_plus.channel, left_plus);
  __HAL_TIM_SET_COMPARE(timers.right_plus.htim, timers.right_plus.channel, right_plus);
  __HAL_TIM_SET_COMPARE(timers.left_minus.htim, timers.left_minus.channel, left_minus);
  __HAL_TIM_SET_COMPARE(timers.right_minus.htim, timers.right_minus.channel, right_minus);
}

void stop_motors(struct Timers_S timers)
{
  set_all_motors(timers, 0, 0, 0, 0);
}

void move_time(struct Timers_S timers, int time, int left_plus, int right_plus, int left_minus, int right_minus)
{
  set_all_motors(timers, left_plus, right_plus, left_minus, right_minus);
  HAL_Delay(time);
  stop_motors(timers);
}

void move_enemy(struct Timers_S timers, int enemy_pos, int left_plus, int right_plus, int left_minus, int right_minus)
{
  set_all_motors(timers, left_plus, right_plus, left_minus, right_minus);

  do
  {
      get_avg_sharp_readings();
  } 
  while(get_enemy_position() == enemy_pos);

  stop_motors(timers);
}

void go_forward(struct Timers_S timers, bool isEnemy_detected, int enemy_pos, int forward_time)
{
  if(isEnemy_detected)
  {
    move_enemy(timers, enemy_pos, 100, 100, 0, 0);
  }
  else
  { 
    move_time(timers, forward_time, 100, 100, 0, 0);
  }
}

void turn_right_forward(struct Timers_S timers, bool isEnemy_detected, int enemy_pos, int turn_time)
{
  if(isEnemy_detected)
  {
    move_enemy(timers, enemy_pos, 100, 0, 0, 0);
  }
  else
  {
    move_time(timers, turn_time, 100, 0, 0, 0);
  }
}

void turn_left_forward(struct Timers_S timers, bool isEnemy_detected, int enemy_pos, int turn_time)
{
  if(isEnemy_detected)
  {
    move_enemy(timers, enemy_pos, 0, 100, 0, 0);
  }
  else
  {
    move_time(timers, turn_time, 0, 100, 0, 0);
  }
}

void turn_right_backward(struct Timers_S timers, bool isEnemy_detected, int enemy_pos, int turn_time)
{
  if(isEnemy_detected)
  {
    move_enemy(timers, enemy_pos, 0, 0, 100, 0);
  }
  else
  {
    move_time(timers, turn_time, 0, 0, 100, 0);
  }
}

void turn_left_backward(struct Timers_S timers, bool isEnemy_detected, int enemy_pos, int turn_time)
{
  if(isEnemy_detected)
  {
    move_enemy(timers, enemy_pos, 0, 0, 0, 100);
  }
  else
  {
    move_time(timers, turn_time, 0, 0, 0, 100);
  }
}

void go_back(struct Timers_S timers, int back_time)
{
  move_time(timers, back_time, 0, 0, 100, 100);
}

void test_motor(TIM_HandleTypeDef *pTim_1, uint32_t channel_1, uint32_t power, char motor[], char direction[1])
{
  char name[80];
  strcpy(name, "starting ");
  strcat(name, motor);
  strcat(name, " motor ");
  strcat(name, direction);
  strcat(name, "\n");
  puts(name);
  transmit_string(name);

  __HAL_TIM_SET_COMPARE(pTim_1, channel_1, power);

  HAL_Delay(5000);

  __HAL_TIM_SET_COMPARE(pTim_1, channel_1, 0);

  
  memset(name,0,sizeof(name));
  strcpy(name, "stopping ");
  strcat(name, motor);
  strcat(name, " motor ");
  strcat(name, direction);
  strcat(name, "\n");
  puts(name);
  transmit_string(name);

  HAL_Delay(1000);
}

void kill_enemy(struct Timers_S timers, int enemy_pos)
{
  switch (enemy_pos)
  {
    case F:
      transmit_string("enemy is in front \r\n");
      go_forward(timers, true, enemy_pos, NS);
      break;

    case R:
      transmit_string("enemy is on the right \r\n");
      turn_right_forward(timers, true, enemy_pos, NS);
      break;

    case L:
      transmit_string("enemy is on the left \r\n");
      turn_left_forward(timers, true, enemy_pos, NS);
      break;
  }
}

void search_for_enemy(struct Timers_S timers)
{
  turn_left_forward(timers, true, None, NS);
}

void test(struct Timer_S timers[])
{
  transmit_string("MOTORS TEST");
  test_motor(timers[0].htim, timers[0].channel, 100, "left", "+");
  test_motor(timers[1].htim, timers[1].channel, 100, "left", "-");
  test_motor(timers[2].htim, timers[2].channel, 100, "right", "+");
  test_motor(timers[3].htim, timers[3].channel, 100, "right", "-");

  transmit_string("QTR SENSORS TEST");
  for(int i = 0; i < 5; ++i)
  {
    get_pololu_readings();
    transmit_qtr_readings();
    HAL_Delay(100);
  }

  transmit_string("SHARP SENSORS TEST");
  for(int i = 0; i < 5; ++i)
  {
    get_avg_sharp_readings();
    transmit_sharp_readings();
    HAL_Delay(100);
  }

  transmit_string("BUTTON TEST");
  while(!isBtn_on()) 
  {
    HAL_Delay(20);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  struct Timers_S timers = {
    {&htim1, TIM_CHANNEL_2},
    {&htim3, TIM_CHANNEL_2},
    {&htim1, TIM_CHANNEL_3},
    {&htim4, TIM_CHANNEL_1}
  };

  reset(timers);
  init(timers);

  //test(timers);

  while(!isBtn_on()) 
  {
    HAL_Delay(50);
  }

  transmit_string("------------START------------\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;
  __HAL_TIM_SET_COMPARE(timers.left_plus.htim, timers.left_plus.channel, 60);
  __HAL_TIM_SET_COMPARE(timers.right_plus.htim, timers.right_plus.channel, 60);

  while (1)
  {
      if(i > 100 && isBtn_on())
      {
        transmit_string("------------STOP------------\r\n");
        break;
      }

      get_pololu_readings();
      get_avg_sharp_readings();

      int enemy_pos = get_enemy_position();

      //change for opposite colors
      if (enemy_pos == None && isLine_found())
      {
          stop_motors(timers);
          transmit_string("line is found, turning left \r\n");
          go_back(timers, 100);
          turn_left_backward(timers, false, None, 300);
          transmit_string("stopped turning \r\n");
          transmit_string("go forward\r\n");
          __HAL_TIM_SET_COMPARE(timers.left_plus.htim, timers.left_plus.channel, 60);
          __HAL_TIM_SET_COMPARE(timers.right_plus.htim, timers.right_plus.channel, 60);
      }

      transmit_sharp_readings();
      transmit_qtr_readings();
      
      
      if(enemy_pos != None)
      {
        transmit_string("there is enemy \r\n");
        kill_enemy(timers, enemy_pos);
        transmit_string("after killing enemy \r\n");
        __HAL_TIM_SET_COMPARE(timers.left_plus.htim, timers.left_plus.channel, 60);
        __HAL_TIM_SET_COMPARE(timers.right_plus.htim, timers.right_plus.channel, 60);
      }
      
      HAL_Delay(5);
      i++;
      if(i == 1000) break;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  transmit_string("end of while \r\n");
  reset(timers);
  
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D4_Pin */
  GPIO_InitStruct.Pin = D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
