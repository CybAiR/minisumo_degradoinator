#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "main.h"

#define MEDIUM_SPEED 60U
#define MAX_SPEED    100U

struct timer_S
{
    TIM_HandleTypeDef* pHtim;
    uint32_t           channel;
    const char*        pLabel;
};

struct motors_S
{
    struct timer_S leftForward;
    struct timer_S leftBackward;
    struct timer_S rightForward;
    struct timer_S rightBackward;
};

enum movementDirection_E
{
    MOVEMENT_FORWARD,
    MOVEMENT_BACKWARD,
    MOVEMENT_FORWARD_LEFT,
    MOVEMENT_FORWARD_RIGHT,
    MOVEMENT_BACKWARD_LEFT,
    MOVEMENT_BACKWARD_RIGHT,
    MOVEMENT_ROTATE_LEFT,
    MOVEMENT_ROTATE_RIGHT
};

extern const struct motors_S gMotors;

void movementInit(const struct motors_S* pMotors);
void movementReset(const struct motors_S* pMotors);
void movementStop(const struct motors_S* pMotors);

void movementDriveContinuously(const struct motors_S*   pMotors,
                               enum movementDirection_E direction,
                               uint32_t                 speed);
void movementDriveFor(const struct motors_S*   pMotors,
                      enum movementDirection_E direction,
                      uint32_t                 speed,
                      uint32_t                 duration_ms);

#endif
