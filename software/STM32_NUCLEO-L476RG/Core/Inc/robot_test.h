#ifndef ROBOT_TEST_H
#define ROBOT_TEST_H

/* Define ROBOT_TEST_ENABLED to compile the hardware diagnostic routines. */
#ifdef ROBOT_TEST_ENABLED

#include "movement.h"

void robotTestAllMotors(uint32_t power, uint32_t runMs, uint32_t pauseMs);
void robotTestQtrSensors(void);
void robotTestSharpSensors(void);
void robotTestButton(void);
void robotTestAllMovements(uint32_t speed, uint32_t durationMs);
void robotTestGeneralTest(void);

#endif /* ROBOT_TEST_ENABLED */

#endif
