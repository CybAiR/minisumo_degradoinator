#include "sensors.h"
#include "time_utils.h"
#include "uart.h"

#define SHARP_AVERAGE_SAMPLE_COUNT  5U
#define ADC_CONVERSION_TIMEOUT_MS   100U
#define BUTTON_DEBOUNCE_DURATION_MS 30U

extern ADC_HandleTypeDef hadc1;

static enum lineColorMode_E  sLineColorMode = WHITE_LINE;
static struct analogSensor_S sQtrLeft       = {0};
static struct analogSensor_S sQtrRight      = {0};
static struct analogSensor_S sSharpLeft     = {0};
static struct analogSensor_S sSharpMiddle   = {0};
static struct analogSensor_S sSharpRight    = {0};

static bool     sIsButtonState        = false;
static bool     sIsButtonCandidate    = false;
static uint32_t sButtonCandidateSince = 0U;

void sensorsInit(void)
{
    sQtrLeft = (struct analogSensor_S){
        .channel = ADC_CHANNEL_5, .type = SENSOR_TYPE_QTR, .name = SENSOR_LEFT};
    sQtrRight = (struct analogSensor_S){
        .channel = ADC_CHANNEL_9, .type = SENSOR_TYPE_QTR, .name = SENSOR_RIGHT};

    sSharpLeft = (struct analogSensor_S){
        .channel = ADC_CHANNEL_15, .type = SENSOR_TYPE_SHARP, .name = SENSOR_LEFT};
    sSharpMiddle = (struct analogSensor_S){
        .channel = ADC_CHANNEL_2, .type = SENSOR_TYPE_SHARP, .name = SENSOR_MIDDLE};
    sSharpRight = (struct analogSensor_S){
        .channel = ADC_CHANNEL_1, .type = SENSOR_TYPE_SHARP, .name = SENSOR_RIGHT};

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
        uartWrite("ADC calibration error\r\n");
}

static void sensorsReadAdc(uint32_t* pValue)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        uartWrite("ADC start error\r\n");

    if (HAL_ADC_PollForConversion(&hadc1, ADC_CONVERSION_TIMEOUT_MS) != HAL_OK)
        uartWrite("ADC conversion error\r\n");

    *pValue = HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
        uartWrite("ADC stop error\r\n");
}

static void sensorsSelectAdcChannel(const struct analogSensor_S* pSensor)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel                = pSensor->channel;
    config.Rank                   = ADC_REGULAR_RANK_1;
    config.SingleDiff             = ADC_SINGLE_ENDED;
    config.OffsetNumber           = ADC_OFFSET_NONE;

    if (pSensor->type == SENSOR_TYPE_SHARP)
        config.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    else
        config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
        uartWrite("ADC channel configuration error\r\n");
}

static void sensorsRead(struct analogSensor_S* pSensor)
{
    uint32_t discardedValue = 0U;
    uint32_t value          = 0U;

    sensorsSelectAdcChannel(pSensor);
    sensorsReadAdc(&discardedValue);
    sensorsReadAdc(&value);

    pSensor->value = value;
}

static void sensorsReadAverage(struct analogSensor_S* pSensor)
{
    uint32_t sum = 0;

    for (uint32_t sample = 0; sample < SHARP_AVERAGE_SAMPLE_COUNT; ++sample)
    {
        sensorsRead(pSensor);
        sum += pSensor->value;
    }

    pSensor->averageValue = sum / SHARP_AVERAGE_SAMPLE_COUNT;
}

void sensorsReadQtrSensors(void)
{
    sensorsRead(&sQtrLeft);
    sensorsRead(&sQtrRight);
}

void sensorsReadSharpSensorsAverage(void)
{
    sensorsReadAverage(&sSharpLeft);
    sensorsReadAverage(&sSharpMiddle);
    sensorsReadAverage(&sSharpRight);
}

void sensorsSetLineColorMode(enum lineColorMode_E mode)
{
    sLineColorMode = mode;
}

enum lineColorMode_E sensorsGetLineColorMode(void)
{
    return sLineColorMode;
}

uint32_t sensorsGetQtrValue(enum analogSensorName_E name)
{
    switch (name)
    {
        case SENSOR_LEFT:
            return sQtrLeft.value;
        case SENSOR_RIGHT:
            return sQtrRight.value;
        default:
            return 0U;
    }
}

uint32_t sensorsGetSharpAverageValue(enum analogSensorName_E name)
{
    switch (name)
    {
        case SENSOR_LEFT:
            return sSharpLeft.averageValue;
        case SENSOR_MIDDLE:
            return sSharpMiddle.averageValue;
        case SENSOR_RIGHT:
            return sSharpRight.averageValue;
        default:
            return 0U;
    }
}

bool sensorsButtonOn(void)
{
    bool isRawState = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin) == GPIO_PIN_SET;

    if (isRawState != sIsButtonCandidate)
    {
        sIsButtonCandidate    = isRawState;
        sButtonCandidateSince = HAL_GetTick();
    }

    if (sIsButtonState != sIsButtonCandidate &&
        timeElapsedMs(sButtonCandidateSince) >= BUTTON_DEBOUNCE_DURATION_MS)
        sIsButtonState = sIsButtonCandidate;

    return sIsButtonState;
}
