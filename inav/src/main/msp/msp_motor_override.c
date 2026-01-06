/*
 * MSP motor override utility - stores per-motor values for a short time window
 */

#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "flight/mixer.h" // MAX_SUPPORTED_MOTORS

#include "msp_motor_override.h"

#define MSP_MOTOR_OVERRIDE_TIMEOUT_US (100 * 1000) // 100 ms

static uint16_t valuesBuf[MAX_SUPPORTED_MOTORS];
static uint8_t valueCount = 0;
static timeUs_t lastUpdateUs = 0;

void mspMotorOverrideReset(void)
{
    valueCount = 0;
    lastUpdateUs = 0;
    // not strictly needed to clear buffer
}

void mspMotorOverrideUpdate(const uint16_t *values, uint8_t count, timeUs_t nowUs)
{
    if (!values || count == 0) {
        valueCount = 0;
        lastUpdateUs = 0;
        return;
    }
    if (count > MAX_SUPPORTED_MOTORS) {
        count = MAX_SUPPORTED_MOTORS;
    }
    memcpy(valuesBuf, values, count * sizeof(valuesBuf[0]));
    valueCount = count;
    lastUpdateUs = nowUs;
}

bool mspMotorOverrideIsFresh(timeUs_t nowUs)
{
    return valueCount > 0 && cmpTimeUs(nowUs, lastUpdateUs) <= MSP_MOTOR_OVERRIDE_TIMEOUT_US;
}

uint8_t mspMotorOverrideCount(void)
{
    return valueCount;
}

bool mspMotorOverrideGetValue(uint8_t index, uint16_t *outUs)
{
    if (index >= valueCount || !outUs) return false;
    *outUs = valuesBuf[index];
    return true;
}
