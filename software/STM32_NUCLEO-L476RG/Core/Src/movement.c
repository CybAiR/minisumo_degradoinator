#include "movement.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

const struct motors_S gMotors = {{&htim1, TIM_CHANNEL_1, "left motor forward"},
                                 {&htim1, TIM_CHANNEL_3, "left motor backward"},
                                 {&htim3, TIM_CHANNEL_2, "right motor forward"},
                                 {&htim3, TIM_CHANNEL_1, "right motor backward"}};

static void movementSetMotors(const struct motors_S* pMotors,
                              uint32_t               left_forward,
                              uint32_t               right_forward,
                              uint32_t               left_backward,
                              uint32_t               right_backward)
{
    __HAL_TIM_SET_COMPARE(pMotors->leftForward.pHtim, pMotors->leftForward.channel, left_forward);
    __HAL_TIM_SET_COMPARE(
        pMotors->rightForward.pHtim, pMotors->rightForward.channel, right_forward);
    __HAL_TIM_SET_COMPARE(
        pMotors->leftBackward.pHtim, pMotors->leftBackward.channel, left_backward);
    __HAL_TIM_SET_COMPARE(
        pMotors->rightBackward.pHtim, pMotors->rightBackward.channel, right_backward);
}

void movementInit(const struct motors_S* pMotors)
{
    movementStop(pMotors);
    HAL_TIM_PWM_Start(pMotors->leftBackward.pHtim, pMotors->leftBackward.channel);
    HAL_TIM_PWM_Start(pMotors->rightBackward.pHtim, pMotors->rightBackward.channel);
    HAL_TIM_PWM_Start(pMotors->leftForward.pHtim, pMotors->leftForward.channel);
    HAL_TIM_PWM_Start(pMotors->rightForward.pHtim, pMotors->rightForward.channel);
}

void movementReset(const struct motors_S* pMotors)
{
    movementStop(pMotors);
    HAL_TIM_PWM_Stop(pMotors->leftBackward.pHtim, pMotors->leftBackward.channel);
    HAL_TIM_PWM_Stop(pMotors->rightBackward.pHtim, pMotors->rightBackward.channel);
    HAL_TIM_PWM_Stop(pMotors->leftForward.pHtim, pMotors->leftForward.channel);
    HAL_TIM_PWM_Stop(pMotors->rightForward.pHtim, pMotors->rightForward.channel);
}

void movementStop(const struct motors_S* pMotors)
{
    movementSetMotors(pMotors, 0, 0, 0, 0);
}

void movementDriveContinuously(const struct motors_S*   pMotors,
                               enum movementDirection_E direction,
                               uint32_t                 speed)
{
    switch (direction)
    {
        case MOVEMENT_FORWARD:
            movementSetMotors(pMotors, speed, speed, 0, 0);
            break;
        case MOVEMENT_BACKWARD:
            movementSetMotors(pMotors, 0, 0, speed, speed);
            break;
        case MOVEMENT_FORWARD_LEFT:
            movementSetMotors(pMotors, 0, speed, 0, 0);
            break;
        case MOVEMENT_FORWARD_RIGHT:
            movementSetMotors(pMotors, speed, 0, 0, 0);
            break;
        case MOVEMENT_BACKWARD_LEFT:
            movementSetMotors(pMotors, 0, 0, 0, speed);
            break;
        case MOVEMENT_BACKWARD_RIGHT:
            movementSetMotors(pMotors, 0, 0, speed, 0);
            break;
        case MOVEMENT_ROTATE_LEFT:
            movementSetMotors(pMotors, 0, speed, speed, 0);
            break;
        case MOVEMENT_ROTATE_RIGHT:
            movementSetMotors(pMotors, speed, 0, 0, speed);
            break;
        default:
            movementStop(pMotors);
            break;
    }
}

void movementDriveFor(const struct motors_S*   pMotors,
                      enum movementDirection_E direction,
                      uint32_t                 speed,
                      uint32_t                 duration_ms)
{
    movementDriveContinuously(pMotors, direction, speed);
    HAL_Delay(duration_ms);
    movementStop(pMotors);
}
