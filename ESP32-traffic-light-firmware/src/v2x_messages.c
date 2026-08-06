#include "v2x_messages.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"

#define J2735_SRM_MESSAGE_ID 29
#define J2735_SSM_MESSAGE_ID 30
#define J2735_MSG_COUNT_MAX 127
#define J2735_DSECOND_MAX 65535

static bool fail(char *error, size_t capacity, const char *message)
{
    if (error != NULL && capacity > 0) {
        snprintf(error, capacity, "%s", message);
    }
    return false;
}

static bool read_integer(
    const cJSON *object,
    const char *name,
    int minimum,
    int maximum,
    int *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (!cJSON_IsNumber(item) ||
        item->valuedouble != (double)item->valueint ||
        item->valueint < minimum ||
        item->valueint > maximum) {
        return false;
    }

    *value = item->valueint;
    return true;
}

static bool read_optional_integer(
    const cJSON *object,
    const char *name,
    int minimum,
    int maximum,
    int *value,
    bool *present)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL) {
        *present = false;
        return true;
    }
    if (!cJSON_IsNumber(item) ||
        item->valuedouble != (double)item->valueint ||
        item->valueint < minimum ||
        item->valueint > maximum) {
        return false;
    }

    *present = true;
    *value = item->valueint;
    return true;
}

static bool parse_entity_id(
    const cJSON *requestor,
    char output[V2X_ENTITY_ID_CAPACITY])
{
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(requestor, "id");
    const cJSON *entity_id = cJSON_IsObject(id) ?
        cJSON_GetObjectItemCaseSensitive(id, "entityID") :
        NULL;
    if (!cJSON_IsString(entity_id) ||
        strlen(entity_id->valuestring) != V2X_ENTITY_ID_HEX_LENGTH) {
        return false;
    }

    for (size_t i = 0; i < V2X_ENTITY_ID_HEX_LENGTH; ++i) {
        unsigned char character = (unsigned char)entity_id->valuestring[i];
        if (!isxdigit(character)) {
            return false;
        }
        output[i] = (char)toupper(character);
    }
    output[V2X_ENTITY_ID_HEX_LENGTH] = '\0';
    return true;
}

static bool parse_emergency_role(
    const cJSON *requestor,
    char output[V2X_ROLE_CAPACITY])
{
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(requestor, "type");
    const cJSON *role = cJSON_IsObject(type) ?
        cJSON_GetObjectItemCaseSensitive(type, "role") :
        NULL;
    if (!cJSON_IsString(role)) {
        return false;
    }

    const char *value = role->valuestring;
    if (strcmp(value, "emergency") != 0 &&
        strcmp(value, "police") != 0 &&
        strcmp(value, "fire") != 0 &&
        strcmp(value, "ambulance") != 0) {
        return false;
    }

    snprintf(output, V2X_ROLE_CAPACITY, "%s", value);
    return true;
}

static bool parse_request_type(
    const cJSON *request,
    v2x_srm_request_type_t *request_type)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(request, "requestType");
    if (!cJSON_IsString(item)) {
        return false;
    }

    if (strcmp(item->valuestring, "priorityRequest") == 0) {
        *request_type = V2X_SRM_PRIORITY_REQUEST;
    } else if (strcmp(item->valuestring, "priorityRequestUpdate") == 0) {
        *request_type = V2X_SRM_PRIORITY_REQUEST_UPDATE;
    } else if (strcmp(item->valuestring, "priorityCancellation") == 0) {
        *request_type = V2X_SRM_PRIORITY_CANCELLATION;
    } else {
        return false;
    }
    return true;
}

static uint8_t signal_group_for_inbound_lane(uint8_t lane)
{
    if (lane == 1 || lane == 3) {
        return 1;
    }
    if (lane == 5 || lane == 7) {
        return 2;
    }
    return 0;
}

static bool parse_matching_package(
    const cJSON *package,
    uint16_t expected_intersection_id,
    v2x_srm_request_t *result,
    bool *matches)
{
    *matches = false;
    if (!cJSON_IsObject(package)) {
        return false;
    }

    const cJSON *request = cJSON_GetObjectItemCaseSensitive(package, "request");
    const cJSON *id = cJSON_IsObject(request) ?
        cJSON_GetObjectItemCaseSensitive(request, "id") :
        NULL;
    int intersection_id;
    if (!cJSON_IsObject(request) ||
        !cJSON_IsObject(id) ||
        !read_integer(id, "id", 0, UINT16_MAX, &intersection_id)) {
        return false;
    }
    if ((uint16_t)intersection_id != expected_intersection_id) {
        return true;
    }

    int request_id;
    if (!read_integer(request, "requestID", 0, UINT8_MAX, &request_id) ||
        !parse_request_type(request, &result->request_type)) {
        return false;
    }
    result->request_id = (uint8_t)request_id;

    int duration = 0;
    if (!read_optional_integer(
            package,
            "duration",
            0,
            J2735_DSECOND_MAX,
            &duration,
            &result->has_duration)) {
        return false;
    }
    result->duration_deciseconds = (uint16_t)duration;

    const cJSON *access_point =
        cJSON_GetObjectItemCaseSensitive(request, "inBoundLane");
    int inbound_lane;
    if (result->request_type == V2X_SRM_PRIORITY_CANCELLATION &&
        access_point == NULL) {
        result->inbound_lane = 0;
        result->signal_group = 0;
    } else if (!cJSON_IsObject(access_point) ||
               !read_integer(access_point, "lane", 0, UINT8_MAX, &inbound_lane)) {
        return false;
    } else {
        result->inbound_lane = (uint8_t)inbound_lane;
        result->signal_group =
            signal_group_for_inbound_lane(result->inbound_lane);
        if (result->signal_group == 0 &&
            result->request_type != V2X_SRM_PRIORITY_CANCELLATION) {
            return false;
        }
    }

    *matches = true;
    return true;
}

bool v2x_parse_srm_message(
    const char *json,
    size_t length,
    uint16_t expected_intersection_id,
    v2x_srm_request_t *request,
    char *error,
    size_t error_capacity)
{
    if (json == NULL || length == 0 || request == NULL) {
        return fail(error, error_capacity, "invalid parser arguments");
    }

    const char *parse_end = NULL;
    cJSON *root =
        cJSON_ParseWithLengthOpts(json, length, &parse_end, false);
    if (root == NULL) {
        return fail(error, error_capacity, "invalid JSON");
    }

    const char *limit = json + length;
    while (parse_end < limit && isspace((unsigned char)*parse_end)) {
        ++parse_end;
    }
    if (parse_end != limit || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return fail(error, error_capacity, "JSON must contain one object");
    }

    v2x_srm_request_t result = {0};
    int message_id;
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "value");
    const cJSON *message = cJSON_IsObject(value) ?
        cJSON_GetObjectItemCaseSensitive(value, "SignalRequestMessage") :
        NULL;
    if (!read_integer(root, "messageId", 0, INT16_MAX, &message_id) ||
        message_id != J2735_SRM_MESSAGE_ID ||
        !cJSON_IsObject(message)) {
        cJSON_Delete(root);
        return fail(error, error_capacity, "expected J2735 SRM messageId 29");
    }

    int second;
    if (!read_integer(message, "second", 0, J2735_DSECOND_MAX, &second)) {
        cJSON_Delete(root);
        return fail(error, error_capacity, "SRM second is missing or invalid");
    }
    result.second = (uint16_t)second;

    int sequence_number = 0;
    bool has_sequence_number;
    if (!read_optional_integer(
            message,
            "sequenceNumber",
            0,
            J2735_MSG_COUNT_MAX,
            &sequence_number,
            &has_sequence_number)) {
        cJSON_Delete(root);
        return fail(error, error_capacity, "SRM sequenceNumber is invalid");
    }
    result.sequence_number = has_sequence_number ?
        (uint8_t)sequence_number :
        0;

    const cJSON *requestor =
        cJSON_GetObjectItemCaseSensitive(message, "requestor");
    if (!cJSON_IsObject(requestor) ||
        !parse_entity_id(requestor, result.entity_id) ||
        !parse_emergency_role(requestor, result.role)) {
        cJSON_Delete(root);
        return fail(
            error,
            error_capacity,
            "requestor needs a 4-byte entityID and emergency vehicle role");
    }

    const cJSON *requests =
        cJSON_GetObjectItemCaseSensitive(message, "requests");
    if (!cJSON_IsArray(requests) || cJSON_GetArraySize(requests) == 0) {
        cJSON_Delete(root);
        return fail(error, error_capacity, "SRM requests must not be empty");
    }

    bool found = false;
    const cJSON *package;
    cJSON_ArrayForEach(package, requests) {
        v2x_srm_request_t candidate = result;
        bool matches;
        if (!parse_matching_package(
                package,
                expected_intersection_id,
                &candidate,
                &matches)) {
            cJSON_Delete(root);
            return fail(error, error_capacity, "SRM request package is invalid");
        }
        if (!matches) {
            continue;
        }
        if (found) {
            cJSON_Delete(root);
            return fail(
                error,
                error_capacity,
                "only one request per intersection is supported");
        }
        result = candidate;
        found = true;
    }

    cJSON_Delete(root);
    if (!found) {
        return fail(error, error_capacity, "SRM does not target this intersection");
    }

    *request = result;
    if (error != NULL && error_capacity > 0) {
        error[0] = '\0';
    }
    return true;
}

static const char *ssm_status_name(v2x_ssm_status_t status)
{
    switch (status) {
    case V2X_SSM_REQUESTED:
        return "requested";
    case V2X_SSM_PROCESSING:
        return "processing";
    case V2X_SSM_GRANTED:
        return "granted";
    case V2X_SSM_REJECTED:
        return "rejected";
    case V2X_SSM_MAX_PRESENCE:
        return "maxPresence";
    default:
        return NULL;
    }
}

int v2x_format_ssm_message(
    char *output,
    size_t capacity,
    const v2x_ssm_update_t *update)
{
    if (output == NULL ||
        capacity == 0 ||
        update == NULL ||
        update->request == NULL) {
        return -1;
    }

    const char *status = ssm_status_name(update->status);
    if (status == NULL) {
        output[0] = '\0';
        return -1;
    }

    const v2x_srm_request_t *request = update->request;
    int written = snprintf(
        output,
        capacity,
        "{\"messageId\":%d,\"value\":{\"SignalStatusMessage\":{"
        "\"timeStamp\":%lu,\"second\":%u,\"sequenceNumber\":%u,"
        "\"status\":[{\"sequenceNumber\":%u,\"id\":{\"id\":%u},"
        "\"sigStatus\":[{\"requester\":{\"id\":{\"entityID\":\"%s\"},"
        "\"request\":%u,\"sequenceNumber\":%u,\"role\":\"%s\"},"
        "\"inboundOn\":{\"lane\":%u},\"duration\":%u,"
        "\"status\":\"%s\"}]}]}}}",
        J2735_SSM_MESSAGE_ID,
        (unsigned long)update->minute_of_year,
        update->second,
        update->sequence_number,
        update->sequence_number,
        update->intersection_id,
        request->entity_id,
        request->request_id,
        request->sequence_number,
        request->role,
        request->inbound_lane,
        request->duration_deciseconds,
        status);
    if (written < 0 || (size_t)written >= capacity) {
        output[0] = '\0';
        return -1;
    }
    return written;
}
