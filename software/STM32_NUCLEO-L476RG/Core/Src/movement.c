#include "movement.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

const struct Motors_S g_motors = {
  {&htim1, TIM_CHANNEL_1, "left motor forward"},
  {&htim1, TIM_CHANNEL_3, "left motor backward"},
  {&htim3, TIM_CHANNEL_2, "right motor forward"},
  {&htim3, TIM_CHANNEL_1, "right motor backward"}
};

static void Movement_set_motors(const struct Motors_S *motors, uint32_t left_forward, uint32_t right_forward, uint32_t left_backward, uint32_t right_backward)
{
  __HAL_TIM_SET_COMPARE(motors->left_forward.htim, motors->left_forward.channel, left_forward);
  __HAL_TIM_SET_COMPARE(motors->right_forward.htim, motors->right_forward.channel, right_forward);
  __HAL_TIM_SET_COMPARE(motors->left_backward.htim, motors->left_backward.channel, left_backward);
  __HAL_TIM_SET_COMPARE(motors->right_backward.htim, motors->right_backward.channel, right_backward);
}

static void Movement_for(const struct Motors_S *motors, uint32_t duration_ms, uint32_t left_forward, uint32_t right_forward, uint32_t left_backward, uint32_t right_backward)
{
  Movement_set_motors(motors, left_forward, right_forward, left_backward, right_backward);
  HAL_Delay(duration_ms);
  Movement_stop(motors);
}

void Movement_init(const struct Motors_S *motors)
{
  Movement_stop(motors);
  HAL_TIM_PWM_Start(motors->left_backward.htim, motors->left_backward.channel);
  HAL_TIM_PWM_Start(motors->right_backward.htim, motors->right_backward.channel);
  HAL_TIM_PWM_Start(motors->left_forward.htim, motors->left_forward.channel);
  HAL_TIM_PWM_Start(motors->right_forward.htim, motors->right_forward.channel);
}

void Movement_reset(const struct Motors_S *motors)
{
  Movement_stop(motors);
  HAL_TIM_PWM_Stop(motors->left_backward.htim, motors->left_backward.channel);
  HAL_TIM_PWM_Stop(motors->right_backward.htim, motors->right_backward.channel);
  HAL_TIM_PWM_Stop(motors->left_forward.htim, motors->left_forward.channel);
  HAL_TIM_PWM_Stop(motors->right_forward.htim, motors->right_forward.channel);
}

void Movement_stop(const struct Motors_S *motors)
{
  Movement_set_motors(motors, 0, 0, 0, 0);
}

void Movement_go_forward_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, speed, speed, 0, 0);
}

void Movement_go_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, speed, speed, 0, 0);
}

void Movement_turn_right_forward_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, speed, 0, 0, 0);
}

void Movement_turn_right_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, speed, 0, 0, 0);
}

void Movement_turn_left_forward_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, 0, speed, 0, 0);
}

void Movement_turn_left_forward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, 0, speed, 0, 0);
}

void Movement_turn_right_backward_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, 0, 0, speed, 0);
}

void Movement_turn_right_backward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, 0, 0, speed, 0);
}

void Movement_turn_left_backward_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, 0, 0, 0, speed);
}

void Movement_turn_left_backward_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, 0, 0, 0, speed);
}

void Movement_go_back_continuously(const struct Motors_S *motors, uint32_t speed)
{
  Movement_set_motors(motors, 0, 0, speed, speed);
}

void Movement_go_back_for(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  Movement_for(motors, duration_ms, 0, 0, speed, speed);
}
