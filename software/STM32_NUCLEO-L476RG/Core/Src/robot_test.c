#include "robot_test.h"

#include "sensors.h"
#include "time_utils.h"
#include "uart.h"

static void robotTestSingleMotor(const struct timer_S* pMotor,
                                 uint32_t              power,
                                 uint32_t              run_ms,
                                 uint32_t              pause_ms)
{
    uartWrite("starting ");
    uartWrite(pMotor->pLabel);
    uartWrite("\r\n");

    __HAL_TIM_SET_COMPARE(pMotor->pHtim, pMotor->channel, power);
    HAL_Delay(run_ms);
    __HAL_TIM_SET_COMPARE(pMotor->pHtim, pMotor->channel, 0);

    uartWrite("stopping ");
    uartWrite(pMotor->pLabel);
    uartWrite("\r\n");
    HAL_Delay(pause_ms);
}

void robotTestAllMotors(const struct motors_S* pMotors,
                        uint32_t               power,
                        uint32_t               run_ms,
                        uint32_t               pause_ms)
{
    robotTestSingleMotor(&pMotors->leftForward, power, run_ms, pause_ms);
    robotTestSingleMotor(&pMotors->leftBackward, power, run_ms, pause_ms);
    robotTestSingleMotor(&pMotors->rightForward, power, run_ms, pause_ms);
    robotTestSingleMotor(&pMotors->rightBackward, power, run_ms, pause_ms);
}

void robotTestQtrSensors(void)
{
    sensorsReadQtrSensors();
    uartWriteQtr(gQtrLeft.value, gQtrRight.value);
}

void robotTestSharpSensors(void)
{
    sensorsReadSharpSensorsAverage();
    uartWriteSharp(gSharpLeft.avg_value, gSharpMiddle.avg_value, gSharpRight.avg_value);
}

void robotTestButton(void)
{
    uartWrite(sensorsButtonOn() ? "button on\r\n" : "button off\r\n");
}

void robotTestAllMovements(const struct motors_S* pMotors, uint32_t speed, uint32_t duration_ms)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementDriveFor(pMotors, MOVEMENT_FORWARD, speed, duration_ms);
    movementDriveFor(pMotors, MOVEMENT_FORWARD_RIGHT, speed, duration_ms);
    movementDriveFor(pMotors, MOVEMENT_FORWARD_LEFT, speed, duration_ms);
    movementDriveFor(pMotors, MOVEMENT_BACKWARD_RIGHT, speed, duration_ms);
    movementDriveFor(pMotors, MOVEMENT_BACKWARD_LEFT, speed, duration_ms);
    movementDriveFor(pMotors, MOVEMENT_BACKWARD, speed, duration_ms);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementDriveContinuously(pMotors, MOVEMENT_FORWARD, speed);
    HAL_Delay(duration_ms);
    movementDriveContinuously(pMotors, MOVEMENT_FORWARD_RIGHT, speed);
    HAL_Delay(duration_ms);
    movementDriveContinuously(pMotors, MOVEMENT_FORWARD_LEFT, speed);
    HAL_Delay(duration_ms);
    movementDriveContinuously(pMotors, MOVEMENT_BACKWARD_RIGHT, speed);
    HAL_Delay(duration_ms);
    movementDriveContinuously(pMotors, MOVEMENT_BACKWARD_LEFT, speed);
    HAL_Delay(duration_ms);
    movementDriveContinuously(pMotors, MOVEMENT_BACKWARD, speed);
    HAL_Delay(duration_ms);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementStop(pMotors);
}

void robotTestGeneralTest(const struct motors_S* pMotors)
{
    const uint32_t motor_test_power        = MAX_SPEED;
    const uint32_t motor_test_run_ms       = 5000U;
    const uint32_t motor_test_pause_ms     = 1000U;
    const uint32_t sensor_test_run_ms      = 10000U;
    const uint32_t button_poll_interval_ms = 100U;
    const uint32_t sensor_read_interval_ms = 5U;

    while (!sensorsButtonOn())
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(button_poll_interval_ms);
    }

    robotTestAllMotors(pMotors, motor_test_power, motor_test_run_ms, motor_test_pause_ms);

    uint32_t sensor_test_start = HAL_GetTick();
    while (timeElapsedMs(sensor_test_start) < sensor_test_run_ms)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        robotTestQtrSensors();
        robotTestSharpSensors();
        HAL_Delay(sensor_read_interval_ms);
    }
}
