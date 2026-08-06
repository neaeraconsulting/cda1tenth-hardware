#include "j2735_messages.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "j2735_config.h"

typedef struct {
    uint8_t lane_id;
    int16_t first_x_cm;
    int16_t first_y_cm;
    int16_t next_x_cm;
    int16_t next_y_cm;
    uint8_t connecting_lane;
    uint8_t signal_group;
} map_lane_t;

static const map_lane_t MAP_LANES[] = {
    {
        .lane_id = 1,
        .first_x_cm = -J2735_HALF_LANE_OFFSET_CM,
        .first_y_cm = J2735_STOP_BAR_OFFSET_CM,
        .next_x_cm = 0,
        .next_y_cm = J2735_APPROACH_EXTENSION_CM,
        .connecting_lane = 4,
        .signal_group = 1,
    },
    {
        .lane_id = 2,
        .first_x_cm = J2735_HALF_LANE_OFFSET_CM,
        .first_y_cm = J2735_STOP_BAR_OFFSET_CM,
        .next_x_cm = 0,
        .next_y_cm = J2735_APPROACH_EXTENSION_CM,
    },
    {
        .lane_id = 3,
        .first_x_cm = J2735_HALF_LANE_OFFSET_CM,
        .first_y_cm = -J2735_STOP_BAR_OFFSET_CM,
        .next_x_cm = 0,
        .next_y_cm = -J2735_APPROACH_EXTENSION_CM,
        .connecting_lane = 2,
        .signal_group = 1,
    },
    {
        .lane_id = 4,
        .first_x_cm = -J2735_HALF_LANE_OFFSET_CM,
        .first_y_cm = -J2735_STOP_BAR_OFFSET_CM,
        .next_x_cm = 0,
        .next_y_cm = -J2735_APPROACH_EXTENSION_CM,
    },
    {
        .lane_id = 5,
        .first_x_cm = J2735_STOP_BAR_OFFSET_CM,
        .first_y_cm = J2735_HALF_LANE_OFFSET_CM,
        .next_x_cm = J2735_APPROACH_EXTENSION_CM,
        .next_y_cm = 0,
        .connecting_lane = 8,
        .signal_group = 2,
    },
    {
        .lane_id = 6,
        .first_x_cm = J2735_STOP_BAR_OFFSET_CM,
        .first_y_cm = -J2735_HALF_LANE_OFFSET_CM,
        .next_x_cm = J2735_APPROACH_EXTENSION_CM,
        .next_y_cm = 0,
    },
    {
        .lane_id = 7,
        .first_x_cm = -J2735_STOP_BAR_OFFSET_CM,
        .first_y_cm = -J2735_HALF_LANE_OFFSET_CM,
        .next_x_cm = -J2735_APPROACH_EXTENSION_CM,
        .next_y_cm = 0,
        .connecting_lane = 6,
        .signal_group = 2,
    },
    {
        .lane_id = 8,
        .first_x_cm = -J2735_STOP_BAR_OFFSET_CM,
        .first_y_cm = J2735_HALF_LANE_OFFSET_CM,
        .next_x_cm = -J2735_APPROACH_EXTENSION_CM,
        .next_y_cm = 0,
    },
};

static bool append_json(
    char *output,
    size_t capacity,
    size_t *used,
    const char *format,
    ...)
{
    if (*used >= capacity) {
        return false;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(output + *used, capacity - *used, format, args);
    va_end(args);

    if (written < 0 || (size_t)written >= capacity - *used) {
        output[0] = '\0';
        return false;
    }

    *used += (size_t)written;
    return true;
}

int j2735_format_map_message(char *output, size_t capacity)
{
    if (output == NULL || capacity == 0) {
        return -1;
    }

    size_t used = 0;
    if (!append_json(
            output,
            capacity,
            &used,
            "{\"messageId\":18,\"value\":{\"MapData\":{"
            "\"msgIssueRevision\":%d,\"layerType\":\"intersectionData\","
            "\"intersections\":[{\"id\":{\"id\":%d},\"revision\":%d,"
            "\"refPoint\":{\"lat\":%ld,\"long\":%ld},\"laneWidth\":%d,"
            "\"laneSet\":[",
            J2735_MAP_ISSUE_REVISION,
            J2735_INTERSECTION_ID,
            J2735_INTERSECTION_REVISION,
            (long)J2735_REFERENCE_LATITUDE,
            (long)J2735_REFERENCE_LONGITUDE,
            J2735_LANE_WIDTH_CM)) {
        return -1;
    }

    for (size_t i = 0; i < sizeof(MAP_LANES) / sizeof(MAP_LANES[0]); ++i) {
        const map_lane_t *lane = &MAP_LANES[i];
        bool ingress = lane->signal_group != 0;
        if (i > 0 && !append_json(output, capacity, &used, ",")) {
            return -1;
        }

        if (!append_json(
                output,
                capacity,
                &used,
                "{\"laneID\":%u,\"%s\":%u,"
                "\"laneAttributes\":{\"directionalUse\":\"%s\","
                "\"sharedWith\":\"0000\",\"laneType\":{\"vehicle\":\"00\"}},"
                "\"maneuvers\":\"8000\",\"nodeList\":{\"nodes\":["
                "{\"delta\":{\"node-XY1\":{\"x\":%d,\"y\":%d}}},"
                "{\"delta\":{\"node-XY1\":{\"x\":%d,\"y\":%d}}}]}",
                lane->lane_id,
                ingress ? "ingressApproach" : "egressApproach",
                (lane->lane_id + 1) / 2,
                ingress ? "80" : "40",
                lane->first_x_cm,
                lane->first_y_cm,
                lane->next_x_cm,
                lane->next_y_cm)) {
            return -1;
        }

        if (ingress &&
            !append_json(
                output,
                capacity,
                &used,
                ",\"connectsTo\":[{\"connectingLane\":{\"lane\":%u,"
                "\"maneuver\":\"8000\"},\"signalGroup\":%u}]",
                lane->connecting_lane,
                lane->signal_group)) {
            return -1;
        }

        if (!append_json(output, capacity, &used, "}")) {
            return -1;
        }
    }

    if (!append_json(output, capacity, &used, "]}]}}}")) {
        return -1;
    }
    return (int)used;
}

int j2735_format_spat_message(
    char *output,
    size_t capacity,
    const j2735_spat_update_t *update)
{
    if (output == NULL || capacity == 0 || update == NULL) {
        return -1;
    }

    size_t used = 0;
    const char *intersection_status =
        update->intersection_status != NULL ?
        update->intersection_status :
        "0000";
    if (!append_json(
            output,
            capacity,
            &used,
            "{\"messageId\":19,\"value\":{\"SPAT\":{"
            "\"intersections\":[{\"id\":{\"id\":%d},"
            "\"revision\":%u,\"status\":\"%s\","
            "\"moy\":%lu,\"timeStamp\":%u,\"states\":[",
            J2735_INTERSECTION_ID,
            update->revision,
            intersection_status,
            (unsigned long)update->minute_of_year,
            update->millisecond_of_minute)) {
        return -1;
    }

    for (size_t i = 0; i < J2735_SIGNAL_GROUP_COUNT; ++i) {
        if (update->movements[i].state == NULL) {
            output[0] = '\0';
            return -1;
        }
        if (i > 0 && !append_json(output, capacity, &used, ",")) {
            return -1;
        }

        if (!append_json(
                output,
                capacity,
                &used,
                "{\"signalGroup\":%u,"
                "\"state-time-speed\":[{\"eventState\":\"%s\",\"timing\":{"
                "\"minEndTime\":%u}}]}",
                (unsigned)(i + 1),
                update->movements[i].state,
                update->movements[i].end_time)) {
            return -1;
        }
    }

    if (!append_json(output, capacity, &used, "]}]}}}")) {
        return -1;
    }
    return (int)used;
}
