#ifndef ROBOT_TEST_H
#define ROBOT_TEST_H

#include "movement.h"

void RobotTest_all_motors(const struct Motors_S *motors, uint32_t power, uint32_t run_ms, uint32_t pause_ms);
void RobotTest_qtr_sensors(void);
void RobotTest_sharp_sensors(void);
void RobotTest_button(void);
void RobotTest_all_movements(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void RobotTest_general_test(const struct Motors_S *motors);

#endif
