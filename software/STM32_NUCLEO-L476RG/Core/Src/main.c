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
#include "movement.h"
#include "robot_test.h"
#include "sensors.h"
#include "time_utils.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_BUTTON_POLL_DELAY_MS    100U
#define LINE_ESCAPE_STOP_DURATION_MS  5U
#define LINE_ESCAPE_TURN_DURATION_MS  120U
#define MOVE_BEFORE_TURN_MS           300U

#define LINE_THRESHOLD  1500U
#define ENEMY_THRESHOLD 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum enemyPosition_E
{
    FRONT = 0,
    LEFT  = 1,
    RIGHT = 2,
    NONE  = -1
};
enum linePosition_E
{
    LINE_NOT_FOUND,
    LINE_FOUND_LEFT,
    LINE_FOUND_RIGHT,
    LINE_FOUND_BOTH
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool checkIfStopBattle(void)
{
    if (sensorsButtonOn())
    {
        uartWrite("------------STOP------------\r\n");
        return true;
    }
    return false;
}

static enum linePosition_E getLinePosition(void)
{
    bool isLeft_found  = false;
    bool isRight_found = false;

    if (gLine_color_mode == WHITE_LINE)
    {
        isLeft_found  = gQtrLeft.value < LINE_THRESHOLD;
        isRight_found = gQtrRight.value < LINE_THRESHOLD;
    }
    else
    {
        isLeft_found  = gQtrLeft.value > LINE_THRESHOLD;
        isRight_found = gQtrRight.value > LINE_THRESHOLD;
    }

    if (isLeft_found && isRight_found)
        return LINE_FOUND_BOTH;
    if (isLeft_found)
        return LINE_FOUND_LEFT;
    if (isRight_found)
        return LINE_FOUND_RIGHT;
    return LINE_NOT_FOUND;
}

bool isLineFound(void)
{
    return getLinePosition() != LINE_NOT_FOUND;
}

bool escapeLine(const struct motors_S* pMotors, uint32_t turn_duration_ms)
{
    enum linePosition_E line_position = getLinePosition();

    while (line_position != LINE_NOT_FOUND)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        movementStop(pMotors);
        HAL_Delay(LINE_ESCAPE_STOP_DURATION_MS);
        movementDriveFor(pMotors, MOVEMENT_BACKWARD, MAX_SPEED, MOVE_BEFORE_TURN_MS);

        if (checkIfStopBattle())
        {
            movementStop(pMotors);
            return true;
        }

        if (line_position == LINE_FOUND_LEFT)
        {
            uartWrite("line found on the left, turning right\r\n");
            movementDriveFor(pMotors, MOVEMENT_ROTATE_RIGHT, MAX_SPEED, turn_duration_ms);
        }
        else
        {
            uartWrite("line found on the right, turning left\r\n");
            movementDriveFor(pMotors, MOVEMENT_ROTATE_LEFT, MAX_SPEED, turn_duration_ms);
        }

        movementStop(pMotors);
        uartWrite("stopped turning\r\n");
        sensorsReadQtrSensors();
        line_position = getLinePosition();
    }

    uartWrite("go forward\r\n");
    movementDriveContinuously(pMotors, MOVEMENT_FORWARD, MEDIUM_SPEED);
    return false;
}

enum enemyPosition_E getEnemyPosition(void)
{
    if ((gSharpLeft.avg_value < ENEMY_THRESHOLD && gSharpMiddle.avg_value >= ENEMY_THRESHOLD &&
         gSharpRight.avg_value < ENEMY_THRESHOLD))
        return FRONT;
    if (gSharpLeft.avg_value >= ENEMY_THRESHOLD && gSharpRight.avg_value >= ENEMY_THRESHOLD)
        return FRONT;
    if (gSharpRight.avg_value >= ENEMY_THRESHOLD)
        return RIGHT;
    if (gSharpLeft.avg_value >= ENEMY_THRESHOLD)
        return LEFT;
    return NONE;
}

static bool killEnemy(const struct motors_S* pMotors, enum enemyPosition_E enemy_pos)
{
    enum enemyPosition_E movement_enemy_pos = NONE;
    while (enemy_pos != NONE)
    {
        if (enemy_pos != movement_enemy_pos)
        {
            movement_enemy_pos = enemy_pos;
            switch (enemy_pos)
            {
                case FRONT:
                    movementDriveContinuously(pMotors, MOVEMENT_FORWARD, MAX_SPEED);
                    uartWrite("enemy is in front \r\n");
                    break;

                case RIGHT:
                    movementDriveContinuously(pMotors, MOVEMENT_FORWARD_RIGHT, MAX_SPEED);
                    uartWrite("enemy is on the right \r\n");
                    break;

                case LEFT:
                    movementDriveContinuously(pMotors, MOVEMENT_FORWARD_LEFT, MAX_SPEED);
                    uartWrite("enemy is on the left \r\n");
                    break;

                default:
                    uartWrite("enemy is not detected \r\n");
                    break;
            }
        }

        sensorsReadSharpSensorsAverage();

        enum enemyPosition_E new_enemy_pos = getEnemyPosition();
        if (new_enemy_pos != enemy_pos)
            enemy_pos = new_enemy_pos;

        if (checkIfStopBattle())
        {
            movementStop(pMotors);
            return true;
        }
    }
    movementStop(pMotors);
    return false;
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

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
     */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    sensorsInit();
    sensorsSetLineColorMode(BLACK_LINE);
    movementInit(&gMotors);

    while (!sensorsButtonOn())
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(START_BUTTON_POLL_DELAY_MS);
        uartWrite("Waiting for start button press\r\n");
    }

    while (sensorsButtonOn())
        HAL_Delay(START_BUTTON_POLL_DELAY_MS);

    // robotTestGeneralTest(&gMotors);
    // return 0;

    uartWrite("------------START------------\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    movementDriveContinuously(&gMotors, MOVEMENT_FORWARD, MEDIUM_SPEED);
    while (1)
    {
        if (checkIfStopBattle())
            break;

        sensorsReadSharpSensorsAverage();
        enum enemyPosition_E enemy_pos = getEnemyPosition();

        sensorsReadQtrSensors();
        if (isLineFound() && enemy_pos == NONE)
            if (escapeLine(&gMotors, LINE_ESCAPE_TURN_DURATION_MS))
                break;

        if (checkIfStopBattle())
            break;

        if (enemy_pos != NONE)
        {
            if (killEnemy(&gMotors, enemy_pos))
                break;
            movementDriveContinuously(&gMotors, MOVEMENT_FORWARD, MEDIUM_SPEED);
            uartWrite("after killing enemy \r\n");
        }

        sensorsReadQtrSensors();
        if (isLineFound())
            if (escapeLine(&gMotors, LINE_ESCAPE_TURN_DURATION_MS))
                break;
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    uartWrite("end of while \r\n");
    movementReset(&gMotors);

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rccOscInitStruct = {0};
    RCC_ClkInitTypeDef rccClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    rccOscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    rccOscInitStruct.HSIState            = RCC_HSI_ON;
    rccOscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rccOscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    rccOscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    rccOscInitStruct.PLL.PLLM            = 1;
    rccOscInitStruct.PLL.PLLN            = 10;
    rccOscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    rccOscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    rccOscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&rccOscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    rccClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rccClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rccClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rccClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    rccClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&rccClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

    ADC_MultiModeTypeDef   multimode = {0};
    ADC_ChannelConfTypeDef config    = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
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
    config.Channel      = ADC_CHANNEL_5;
    config.Rank         = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    config.SingleDiff   = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
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

    TIM_ClockConfigTypeDef         clockSourceConfig   = {0};
    TIM_MasterConfigTypeDef        masterConfig        = {0};
    TIM_OC_InitTypeDef             configOc            = {0};
    TIM_BreakDeadTimeConfigTypeDef breakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 7;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 99;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    clockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &clockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    masterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    masterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    masterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &masterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    configOc.OCMode       = TIM_OCMODE_PWM1;
    configOc.Pulse        = 0;
    configOc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    configOc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    configOc.OCFastMode   = TIM_OCFAST_DISABLE;
    configOc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    configOc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &configOc, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &configOc, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    breakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    breakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    breakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    breakDeadTimeConfig.DeadTime         = 0;
    breakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    breakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    breakDeadTimeConfig.BreakFilter      = 0;
    breakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    breakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    breakDeadTimeConfig.Break2Filter     = 0;
    breakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &breakDeadTimeConfig) != HAL_OK)
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

    TIM_ClockConfigTypeDef  clockSourceConfig = {0};
    TIM_MasterConfigTypeDef masterConfig      = {0};
    TIM_OC_InitTypeDef      configOc          = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 7;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 99;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    clockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &clockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    masterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    masterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &masterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    configOc.OCMode     = TIM_OCMODE_PWM1;
    configOc.Pulse      = 0;
    configOc.OCPolarity = TIM_OCPOLARITY_HIGH;
    configOc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &configOc, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &configOc, TIM_CHANNEL_2) != HAL_OK)
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
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
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
    GPIO_InitTypeDef gpioInitStruct = {0};
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
    gpioInitStruct.Pin  = B1_Pin;
    gpioInitStruct.Mode = GPIO_MODE_IT_FALLING;
    gpioInitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &gpioInitStruct);

    /*Configure GPIO pin : LD2_Pin */
    gpioInitStruct.Pin   = LD2_Pin;
    gpioInitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpioInitStruct.Pull  = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &gpioInitStruct);

    /*Configure GPIO pin : D4_Pin */
    gpioInitStruct.Pin  = D4_Pin;
    gpioInitStruct.Mode = GPIO_MODE_INPUT;
    gpioInitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(D4_GPIO_Port, &gpioInitStruct);

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
    uartWrite("HAL initialization error\r\n");
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
void assert_failed(uint8_t* pFile, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
