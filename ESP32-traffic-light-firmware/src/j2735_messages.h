#pragma once

#include <stddef.h>
#include <stdint.h>

#define J2735_MAP_MESSAGE_CAPACITY 3072
#define J2735_SPAT_MESSAGE_CAPACITY 512
#define J2735_SIGNAL_GROUP_COUNT 2
#define J2735_MINUTE_OF_YEAR_UNAVAILABLE 527040
#define J2735_DSECOND_UNAVAILABLE 65535
#define J2735_TIME_MARK_UNKNOWN 36111

typedef struct {
    const char *state;
    uint16_t end_time;
} j2735_movement_update_t;

typedef struct {
    uint8_t revision;
    uint32_t minute_of_year;
    uint16_t millisecond_of_minute;
    const char *intersection_status;
    j2735_movement_update_t movements[J2735_SIGNAL_GROUP_COUNT];
} j2735_spat_update_t;

/*
 * Return the number of JSON bytes written, or -1 for invalid arguments or
 * a buffer that is too small. The resulting JSON is NUL-terminated.
 */
int j2735_format_map_message(char *output, size_t capacity);
int j2735_format_spat_message(
    char *output,
    size_t capacity,
    const j2735_spat_update_t *update);
