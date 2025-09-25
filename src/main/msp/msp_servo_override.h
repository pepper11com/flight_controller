#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

void mspServoOverrideReset(void);
void mspServoOverrideUpdate(const uint16_t *values, uint8_t count, timeUs_t nowUs);
bool mspServoOverrideIsFresh(timeUs_t nowUs);
uint8_t mspServoOverrideCount(void);
bool mspServoOverrideGetValue(uint8_t index, uint16_t *outUs);

#ifdef __cplusplus
}
#endif
