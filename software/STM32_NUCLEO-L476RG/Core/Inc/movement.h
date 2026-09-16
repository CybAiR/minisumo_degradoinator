#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "main.h"

#define MEDIUM_SPEED 60U
#define MAX_SPEED    100U

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

void movementInit(void);
void movementReset(void);
void movementStop(void);

void movementDriveContinuously(enum movementDirection_E direction, uint32_t speed);
void movementDriveFor(enum movementDirection_E direction, uint32_t speed, uint32_t durationMs);

#endif
