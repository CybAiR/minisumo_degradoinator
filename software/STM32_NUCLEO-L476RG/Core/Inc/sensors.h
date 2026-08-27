#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include <stdbool.h>

enum AnalogSensorType {
  SENSOR_TYPE_QTR,
  SENSOR_TYPE_SHARP
};

enum AnalogSensorName {
  SENSOR_LEFT,
  SENSOR_MIDDLE,
  SENSOR_RIGHT,
  SENSOR_COUNT
};

struct AnalogSensor_S {
  uint32_t channel;
  uint32_t value;
  uint32_t avg_value;
  enum AnalogSensorType type;
  enum AnalogSensorName name;
};

enum LineColorMode {
  BLACK_LINE,
  WHITE_LINE
};

void Sensors_init(void);
bool Sensors_button_on(void);
void Sensors_read_qtr_sensors(void);
void Sensors_read_sharp_sensors_average(void);
void Sensors_set_line_color_mode(enum LineColorMode mode);

extern enum LineColorMode g_line_color_mode;

extern struct AnalogSensor_S g_qtr_left, g_qtr_right;
extern struct AnalogSensor_S g_sharp_left, g_sharp_middle, g_sharp_right;

#endif
