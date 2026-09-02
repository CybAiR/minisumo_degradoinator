#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include <stdbool.h>

enum analogSensorType_E
{
    SENSOR_TYPE_QTR,
    SENSOR_TYPE_SHARP
};

enum analogSensorName_E
{
    SENSOR_LEFT,
    SENSOR_MIDDLE,
    SENSOR_RIGHT,
    SENSOR_COUNT
};

struct analogSensor_S
{
    uint32_t                channel;
    uint32_t                value;
    uint32_t                avg_value;
    enum analogSensorType_E type;
    enum analogSensorName_E name;
};

enum lineColorMode_E
{
    BLACK_LINE,
    WHITE_LINE
};

void sensorsInit(void);
bool sensorsButtonOn(void);
void sensorsReadQtrSensors(void);
void sensorsReadSharpSensorsAverage(void);
void sensorsSetLineColorMode(enum lineColorMode_E mode);

extern enum lineColorMode_E gLine_color_mode;

extern struct analogSensor_S gQtrLeft, gQtrRight;
extern struct analogSensor_S gSharpLeft, gSharpMiddle, gSharpRight;

#endif
