#include "movement.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

struct timer_S
{
    TIM_HandleTypeDef* pHtim;
    uint32_t           channel;
};

struct motors_S
{
    struct timer_S leftForward;
    struct timer_S leftBackward;
    struct timer_S rightForward;
    struct timer_S rightBackward;
};

static const struct motors_S sMotors = {{&htim1, TIM_CHANNEL_1},
                                        {&htim1, TIM_CHANNEL_3},
                                        {&htim3, TIM_CHANNEL_2},
                                        {&htim3, TIM_CHANNEL_1}};

static void movementSetMotors(uint32_t leftForward,
                              uint32_t rightForward,
                              uint32_t leftBackward,
                              uint32_t rightBackward)
{
    __HAL_TIM_SET_COMPARE(sMotors.leftForward.pHtim, sMotors.leftForward.channel, leftForward);
    __HAL_TIM_SET_COMPARE(sMotors.rightForward.pHtim, sMotors.rightForward.channel, rightForward);
    __HAL_TIM_SET_COMPARE(sMotors.leftBackward.pHtim, sMotors.leftBackward.channel, leftBackward);
    __HAL_TIM_SET_COMPARE(
        sMotors.rightBackward.pHtim, sMotors.rightBackward.channel, rightBackward);
}

void movementInit(void)
{
    movementStop();
    HAL_TIM_PWM_Start(sMotors.leftBackward.pHtim, sMotors.leftBackward.channel);
    HAL_TIM_PWM_Start(sMotors.rightBackward.pHtim, sMotors.rightBackward.channel);
    HAL_TIM_PWM_Start(sMotors.leftForward.pHtim, sMotors.leftForward.channel);
    HAL_TIM_PWM_Start(sMotors.rightForward.pHtim, sMotors.rightForward.channel);
}

void movementReset(void)
{
    movementStop();
    HAL_TIM_PWM_Stop(sMotors.leftBackward.pHtim, sMotors.leftBackward.channel);
    HAL_TIM_PWM_Stop(sMotors.rightBackward.pHtim, sMotors.rightBackward.channel);
    HAL_TIM_PWM_Stop(sMotors.leftForward.pHtim, sMotors.leftForward.channel);
    HAL_TIM_PWM_Stop(sMotors.rightForward.pHtim, sMotors.rightForward.channel);
}

void movementStop(void)
{
    movementSetMotors(0, 0, 0, 0);
}

void movementDriveContinuously(enum movementDirection_E direction, uint32_t speed)
{
    switch (direction)
    {
        case MOVEMENT_FORWARD:
            movementSetMotors(speed, speed, 0, 0);
            break;
        case MOVEMENT_BACKWARD:
            movementSetMotors(0, 0, speed, speed);
            break;
        case MOVEMENT_FORWARD_LEFT:
            movementSetMotors(0, speed, 0, 0);
            break;
        case MOVEMENT_FORWARD_RIGHT:
            movementSetMotors(speed, 0, 0, 0);
            break;
        case MOVEMENT_BACKWARD_LEFT:
            movementSetMotors(0, 0, 0, speed);
            break;
        case MOVEMENT_BACKWARD_RIGHT:
            movementSetMotors(0, 0, speed, 0);
            break;
        case MOVEMENT_ROTATE_LEFT:
            movementSetMotors(0, speed, speed, 0);
            break;
        case MOVEMENT_ROTATE_RIGHT:
            movementSetMotors(speed, 0, 0, speed);
            break;
        default:
            movementStop();
            break;
    }
}

void movementDriveFor(enum movementDirection_E direction, uint32_t speed, uint32_t durationMs)
{
    movementDriveContinuously(direction, speed);
    HAL_Delay(durationMs);
    movementStop();
}
