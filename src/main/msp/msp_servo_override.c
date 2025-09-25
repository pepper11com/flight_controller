/*
 * MSP servo override utility - stores per-servo values for a short time window
 */

#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "flight/servos.h" // MAX_SUPPORTED_SERVOS, servoParams

#include "msp_servo_override.h"

#define MSP_SERVO_OVERRIDE_TIMEOUT_US (100 * 1000) // 100 ms

static uint16_t valuesBuf[MAX_SUPPORTED_SERVOS];
static uint8_t valueCount = 0;
static timeUs_t lastUpdateUs = 0;

void mspServoOverrideReset(void)
{
    valueCount = 0;
    lastUpdateUs = 0;
}

void mspServoOverrideUpdate(const uint16_t *values, uint8_t count, timeUs_t nowUs)
{
    if (!values || count == 0) {
        valueCount = 0;
        lastUpdateUs = 0;
        return;
    }
    if (count > MAX_SUPPORTED_SERVOS) {
        count = MAX_SUPPORTED_SERVOS;
    }
    memcpy(valuesBuf, values, count * sizeof(valuesBuf[0]));
    valueCount = count;
    lastUpdateUs = nowUs;
}

bool mspServoOverrideIsFresh(timeUs_t nowUs)
{
    return valueCount > 0 && cmpTimeUs(nowUs, lastUpdateUs) <= MSP_SERVO_OVERRIDE_TIMEOUT_US;
}

uint8_t mspServoOverrideCount(void)
{
    return valueCount;
}

bool mspServoOverrideGetValue(uint8_t index, uint16_t *outUs)
{
    if (index >= valueCount || !outUs) return false;
    *outUs = valuesBuf[index];
    return true;
}
