#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define J2735_SSM_MESSAGE_CAPACITY 768
#define V2X_ENTITY_ID_HEX_LENGTH 8
#define V2X_ENTITY_ID_CAPACITY (V2X_ENTITY_ID_HEX_LENGTH + 1)
#define V2X_ROLE_CAPACITY 16

typedef enum {
    V2X_SRM_PRIORITY_REQUEST,
    V2X_SRM_PRIORITY_REQUEST_UPDATE,
    V2X_SRM_PRIORITY_CANCELLATION,
} v2x_srm_request_type_t;

typedef struct {
    uint8_t sequence_number;
    uint16_t second;
    uint8_t request_id;
    v2x_srm_request_type_t request_type;
    uint8_t inbound_lane;
    uint8_t signal_group;
    bool has_duration;
    uint16_t duration_deciseconds;
    char entity_id[V2X_ENTITY_ID_CAPACITY];
    char role[V2X_ROLE_CAPACITY];
} v2x_srm_request_t;

typedef enum {
    V2X_SSM_REQUESTED,
    V2X_SSM_PROCESSING,
    V2X_SSM_GRANTED,
    V2X_SSM_REJECTED,
    V2X_SSM_MAX_PRESENCE,
} v2x_ssm_status_t;

typedef struct {
    uint32_t minute_of_year;
    uint16_t second;
    uint8_t sequence_number;
    uint16_t intersection_id;
    const v2x_srm_request_t *request;
    v2x_ssm_status_t status;
} v2x_ssm_update_t;

/*
 * Parse the schema-shaped JSON representation of a J2735 SRM. This project
 * accepts one request for its own intersection and lane-based access points.
 */
bool v2x_parse_srm_message(
    const char *json,
    size_t length,
    uint16_t expected_intersection_id,
    v2x_srm_request_t *request,
    char *error,
    size_t error_capacity);

/*
 * Return the number of JSON bytes written, or -1 for invalid arguments or a
 * buffer that is too small. The resulting JSON is NUL-terminated.
 */
int v2x_format_ssm_message(
    char *output,
    size_t capacity,
    const v2x_ssm_update_t *update);
