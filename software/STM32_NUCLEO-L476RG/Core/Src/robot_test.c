#include "robot_test.h"

#ifdef ROBOT_TEST_ENABLED

#include "sensors.h"
#include "time_utils.h"
#include "uart.h"

static void robotTestSingleMotor(enum movementDirection_E direction,
                                 const char*              pLabel,
                                 uint32_t                 power,
                                 uint32_t                 runMs,
                                 uint32_t                 pauseMs)
{
    uartWrite("starting ");
    uartWrite(pLabel);
    uartWrite("\r\n");

    movementDriveFor(direction, power, runMs);

    uartWrite("stopping ");
    uartWrite(pLabel);
    uartWrite("\r\n");
    HAL_Delay(pauseMs);
}

void robotTestAllMotors(uint32_t power, uint32_t runMs, uint32_t pauseMs)
{
    robotTestSingleMotor(MOVEMENT_FORWARD_RIGHT, "left motor forward", power, runMs, pauseMs);
    robotTestSingleMotor(MOVEMENT_BACKWARD_RIGHT, "left motor backward", power, runMs, pauseMs);
    robotTestSingleMotor(MOVEMENT_FORWARD_LEFT, "right motor forward", power, runMs, pauseMs);
    robotTestSingleMotor(MOVEMENT_BACKWARD_LEFT, "right motor backward", power, runMs, pauseMs);
}

void robotTestQtrSensors(void)
{
    sensorsReadQtrSensors();
    uartWriteQtr(sensorsGetQtrValue(SENSOR_LEFT), sensorsGetQtrValue(SENSOR_RIGHT));
}

void robotTestSharpSensors(void)
{
    sensorsReadSharpSensorsAverage();
    uartWriteSharp(sensorsGetSharpAverageValue(SENSOR_LEFT),
                   sensorsGetSharpAverageValue(SENSOR_MIDDLE),
                   sensorsGetSharpAverageValue(SENSOR_RIGHT));
}

void robotTestButton(void)
{
    uartWrite(sensorsButtonOn() ? "button on\r\n" : "button off\r\n");
}

void robotTestAllMovements(uint32_t speed, uint32_t durationMs)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementDriveFor(MOVEMENT_FORWARD, speed, durationMs);
    movementDriveFor(MOVEMENT_FORWARD_RIGHT, speed, durationMs);
    movementDriveFor(MOVEMENT_FORWARD_LEFT, speed, durationMs);
    movementDriveFor(MOVEMENT_BACKWARD_RIGHT, speed, durationMs);
    movementDriveFor(MOVEMENT_BACKWARD_LEFT, speed, durationMs);
    movementDriveFor(MOVEMENT_BACKWARD, speed, durationMs);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementDriveContinuously(MOVEMENT_FORWARD, speed);
    HAL_Delay(durationMs);
    movementDriveContinuously(MOVEMENT_FORWARD_RIGHT, speed);
    HAL_Delay(durationMs);
    movementDriveContinuously(MOVEMENT_FORWARD_LEFT, speed);
    HAL_Delay(durationMs);
    movementDriveContinuously(MOVEMENT_BACKWARD_RIGHT, speed);
    HAL_Delay(durationMs);
    movementDriveContinuously(MOVEMENT_BACKWARD_LEFT, speed);
    HAL_Delay(durationMs);
    movementDriveContinuously(MOVEMENT_BACKWARD, speed);
    HAL_Delay(durationMs);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    movementStop();
}

void robotTestGeneralTest(void)
{
    const uint32_t motorTestPower       = MAX_SPEED;
    const uint32_t motorTestRunMs       = 5000U;
    const uint32_t motorTestPauseMs     = 1000U;
    const uint32_t sensorTestRunMs      = 10000U;
    const uint32_t buttonPollIntervalMs = 100U;
    const uint32_t sensorReadIntervalMs = 5U;

    while (!sensorsButtonOn())
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(buttonPollIntervalMs);
    }

    robotTestAllMotors(motorTestPower, motorTestRunMs, motorTestPauseMs);

    uint32_t sensorTestStart = HAL_GetTick();
    while (timeElapsedMs(sensorTestStart) < sensorTestRunMs)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        robotTestQtrSensors();
        robotTestSharpSensors();
        HAL_Delay(sensorReadIntervalMs);
    }
}

#endif /* ROBOT_TEST_ENABLED */
