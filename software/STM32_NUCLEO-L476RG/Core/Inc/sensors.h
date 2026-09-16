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
    uint32_t                averageValue;
    enum analogSensorType_E type;
    enum analogSensorName_E name;
};

enum lineColorMode_E
{
    BLACK_LINE,
    WHITE_LINE
};

void                 sensorsInit(void);
bool                 sensorsButtonOn(void);
void                 sensorsReadQtrSensors(void);
void                 sensorsReadSharpSensorsAverage(void);
void                 sensorsSetLineColorMode(enum lineColorMode_E mode);
enum lineColorMode_E sensorsGetLineColorMode(void);

/* Return cached readings without sampling; unsupported sensor names return zero. */
uint32_t sensorsGetQtrValue(enum analogSensorName_E name);
uint32_t sensorsGetSharpAverageValue(enum analogSensorName_E name);

#endif
