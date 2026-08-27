#include "robot_test.h"

#include "sensors.h"
#include "time_utils.h"
#include "uart.h"

static void RobotTest_single_motor(const struct Timer_S *motor, uint32_t power, uint32_t run_ms, uint32_t pause_ms)
{
  UART_write("starting ");
  UART_write(motor->label);
  UART_write("\r\n");

  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, power);
  HAL_Delay(run_ms);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

  UART_write("stopping ");
  UART_write(motor->label);
  UART_write("\r\n");
  HAL_Delay(pause_ms);
}

void RobotTest_all_motors(const struct Motors_S *motors, uint32_t power, uint32_t run_ms, uint32_t pause_ms)
{
    RobotTest_single_motor(&motors->left_forward, power, run_ms, pause_ms);
    RobotTest_single_motor(&motors->left_backward, power, run_ms, pause_ms);
    RobotTest_single_motor(&motors->right_forward, power, run_ms, pause_ms);
    RobotTest_single_motor(&motors->right_backward, power, run_ms, pause_ms);
}

void RobotTest_qtr_sensors(void)
{
  Sensors_read_qtr_sensors();
  UART_write_qtr(g_qtr_left.value, g_qtr_right.value);
}

void RobotTest_sharp_sensors(void)
{
  Sensors_read_sharp_sensors_average();
  UART_write_sharp(g_sharp_left.avg_value, g_sharp_middle.avg_value, g_sharp_right.avg_value);
}

void RobotTest_button(void)
{
  UART_write(Sensors_button_on() ? "button on\r\n" : "button off\r\n");
}

void RobotTest_all_movements(const struct Motors_S *motors, uint32_t speed, uint32_t duration_ms)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  Movement_go_forward_for(motors, speed, duration_ms);
  Movement_turn_right_forward_for(motors, speed, duration_ms);
  Movement_turn_left_forward_for(motors, speed, duration_ms);
  Movement_turn_right_backward_for(motors, speed, duration_ms);
  Movement_turn_left_backward_for(motors, speed, duration_ms);
  Movement_go_back_for(motors, speed, duration_ms);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  Movement_go_forward_continuously(motors, speed);
  HAL_Delay(duration_ms);
  Movement_turn_right_forward_continuously(motors, speed);
  HAL_Delay(duration_ms);
  Movement_turn_left_forward_continuously(motors, speed);
  HAL_Delay(duration_ms);
  Movement_turn_right_backward_continuously(motors, speed);
  HAL_Delay(duration_ms);
  Movement_turn_left_backward_continuously(motors, speed);
  HAL_Delay(duration_ms);
  Movement_go_back_continuously(motors, speed);
  HAL_Delay(duration_ms);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  Movement_stop(motors);
}

void RobotTest_general_test(const struct Motors_S *motors)
{
  const uint32_t motor_test_power = MAX_SPEED;
  const uint32_t motor_test_run_ms = 5000U;
  const uint32_t motor_test_pause_ms = 1000U;
  const uint32_t sensor_test_run_ms = 10000U;
  const uint32_t button_poll_interval_ms = 100U;
  const uint32_t sensor_read_interval_ms = 5U;

  while (!Sensors_button_on())
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(button_poll_interval_ms);
  }

  RobotTest_all_motors(motors, motor_test_power, motor_test_run_ms, motor_test_pause_ms);

  uint32_t sensor_test_start = HAL_GetTick();
  while (Time_elapsed_ms(sensor_test_start) < sensor_test_run_ms)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    RobotTest_qtr_sensors();
    RobotTest_sharp_sensors();
    HAL_Delay(sensor_read_interval_ms);
  }
}
