#include <stdio.h>
#include <string.h>

#include "../src/j2735_messages.h"

int main(void)
{
    char map_message[J2735_MAP_MESSAGE_CAPACITY];
    char spat_message[J2735_SPAT_MESSAGE_CAPACITY];
    j2735_spat_update_t update = {
        .revision = 7,
        .minute_of_year = 12345,
        .millisecond_of_minute = 23456,
        .intersection_status = "1000",
        .movements = {
            {
                .state = "protected-Movement-Allowed",
                .end_time = 1200,
            },
            {
                .state = "stop-And-Remain",
                .end_time = 1330,
            },
        },
    };

    if (j2735_format_map_message(map_message, sizeof(map_message)) < 0 ||
        j2735_format_spat_message(spat_message, sizeof(spat_message), &update) < 0) {
        return 1;
    }
    if (strstr(spat_message, "\"status\":\"1000\"") == NULL) {
        return 1;
    }

    puts(map_message);
    puts(spat_message);
    return 0;
}
