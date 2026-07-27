#ifndef BSP_EIGHT_TRACKING_H_
#define BSP_EIGHT_TRACKING_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_PATTERN_ALL_WHITE = 0,
    LINE_PATTERN_ALL_BLACK,
    LINE_PATTERN_VALID,
    LINE_PATTERN_DISCRETE_INVALID,
    LINE_PATTERN_TOO_WIDE
} LinePattern;

typedef struct {
    uint8_t raw;
    uint8_t black_mask;
    uint8_t active_count;
    int16_t error;
    LinePattern pattern;
    bool contiguous;
    bool valid_line;
    bool straight_line;
    bool start_line_valid;
} LineObservation;

void EightTracking_Init(void);
void EightTracking_Read(LineObservation *observation);
uint8_t EightTracking_ReadRaw(void);

#endif /* BSP_EIGHT_TRACKING_H_ */
