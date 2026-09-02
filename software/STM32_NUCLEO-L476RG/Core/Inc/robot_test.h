#ifndef ROBOT_TEST_H
#define ROBOT_TEST_H

#include "movement.h"

void robotTestAllMotors(const struct motors_S* pMotors,
                        uint32_t               power,
                        uint32_t               run_ms,
                        uint32_t               pause_ms);
void robotTestQtrSensors(void);
void robotTestSharpSensors(void);
void robotTestButton(void);
void robotTestAllMovements(const struct motors_S* pMotors, uint32_t speed, uint32_t duration_ms);
void robotTestGeneralTest(const struct motors_S* pMotors);

#endif
