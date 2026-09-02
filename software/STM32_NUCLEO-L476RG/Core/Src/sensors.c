#include "sensors.h"
#include "time_utils.h"
#include "uart.h"

#define SHARP_AVERAGE_SAMPLE_COUNT  5U
#define ADC_CONVERSION_TIMEOUT_MS   100U
#define BUTTON_DEBOUNCE_DURATION_MS 30U

extern ADC_HandleTypeDef hadc1;

enum lineColorMode_E  gLine_color_mode = WHITE_LINE;
struct analogSensor_S gQtrLeft         = {0};
struct analogSensor_S gQtrRight        = {0};
struct analogSensor_S gSharpLeft       = {0};
struct analogSensor_S gSharpMiddle     = {0};
struct analogSensor_S gSharpRight      = {0};

static bool     sIsButton_state_         = false;
static bool     sIsButton_candidate_     = false;
static uint32_t sButton_candidate_since_ = 0U;

void sensorsInit(void)
{
    gQtrLeft = (struct analogSensor_S){
        .channel = ADC_CHANNEL_5, .type = SENSOR_TYPE_QTR, .name = SENSOR_LEFT};
    gQtrRight = (struct analogSensor_S){
        .channel = ADC_CHANNEL_9, .type = SENSOR_TYPE_QTR, .name = SENSOR_RIGHT};

    gSharpLeft = (struct analogSensor_S){
        .channel = ADC_CHANNEL_15, .type = SENSOR_TYPE_SHARP, .name = SENSOR_LEFT};
    gSharpMiddle = (struct analogSensor_S){
        .channel = ADC_CHANNEL_2, .type = SENSOR_TYPE_SHARP, .name = SENSOR_MIDDLE};
    gSharpRight = (struct analogSensor_S){
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
    uint32_t discarded_value = 0U;
    uint32_t value           = 0U;

    sensorsSelectAdcChannel(pSensor);
    sensorsReadAdc(&discarded_value);
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

    pSensor->avg_value = sum / SHARP_AVERAGE_SAMPLE_COUNT;
}

void sensorsReadQtrSensors(void)
{
    sensorsRead(&gQtrLeft);
    sensorsRead(&gQtrRight);
}

void sensorsReadSharpSensorsAverage(void)
{
    sensorsReadAverage(&gSharpLeft);
    sensorsReadAverage(&gSharpMiddle);
    sensorsReadAverage(&gSharpRight);
}

void sensorsSetLineColorMode(enum lineColorMode_E mode)
{
    gLine_color_mode = mode;
}

bool sensorsButtonOn(void)
{
    bool isRaw_state = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin) == GPIO_PIN_SET;

    if (isRaw_state != sIsButton_candidate_)
    {
        sIsButton_candidate_     = isRaw_state;
        sButton_candidate_since_ = HAL_GetTick();
    }

    if (sIsButton_state_ != sIsButton_candidate_ &&
        timeElapsedMs(sButton_candidate_since_) >= BUTTON_DEBOUNCE_DURATION_MS)
        sIsButton_state_ = sIsButton_candidate_;

    return sIsButton_state_;
}
