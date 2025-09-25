/*
 * MSP motor override utility
 */
#pragma once

#include <stdint.h>
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

void mspMotorOverrideReset(void);
void mspMotorOverrideUpdate(const uint16_t *values, uint8_t count, timeUs_t nowUs);
bool mspMotorOverrideIsFresh(timeUs_t nowUs);
uint8_t mspMotorOverrideCount(void);
bool mspMotorOverrideGetValue(uint8_t index, uint16_t *outUs);

#ifdef __cplusplus
}
#endif
