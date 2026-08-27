#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "main.h"

#define MEDIUM_SPEED 60U
#define MAX_SPEED    100U

struct Timer_S {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  const char *label;
};

struct Motors_S {
  struct Timer_S left_forward;
  struct Timer_S left_backward;
  struct Timer_S right_forward;
  struct Timer_S right_backward;
};

extern const struct Motors_S g_motors;

void Movement_init(const struct Motors_S *motors);
void Movement_reset(const struct Motors_S *motors);
void Movement_stop(const struct Motors_S *motors);

void Movement_go_forward_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_go_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void Movement_turn_right_forward_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_turn_right_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void Movement_turn_left_forward_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_turn_left_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void Movement_turn_right_backward_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_turn_right_backward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void Movement_turn_left_backward_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_turn_left_backward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);
void Movement_go_back_continuously(const struct Motors_S *motors, uint32_t speed);
void Movement_go_back_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms);

#endif
