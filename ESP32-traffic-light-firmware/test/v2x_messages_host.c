#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../src/v2x_messages.h"

static const char VALID_SRM[] =
    "{\"messageId\":29,\"value\":{\"SignalRequestMessage\":{"
    "\"second\":12000,\"sequenceNumber\":9,"
    "\"requests\":[{\"request\":{\"id\":{\"id\":1},\"requestID\":7,"
    "\"requestType\":\"priorityRequest\",\"inBoundLane\":{\"lane\":1}},"
    "\"duration\":100}],"
    "\"requestor\":{\"id\":{\"entityID\":\"01ab03cd\"},"
    "\"type\":{\"role\":\"fire\"}}}}}";

static void test_valid_request(void)
{
    v2x_srm_request_t request;
    char error[128];
    assert(v2x_parse_srm_message(
        VALID_SRM,
        strlen(VALID_SRM),
        1,
        &request,
        error,
        sizeof(error)));
    assert(request.sequence_number == 9);
    assert(request.request_id == 7);
    assert(request.request_type == V2X_SRM_PRIORITY_REQUEST);
    assert(request.inbound_lane == 1);
    assert(request.signal_group == 1);
    assert(request.has_duration);
    assert(request.duration_deciseconds == 100);
    assert(strcmp(request.entity_id, "01AB03CD") == 0);
    assert(strcmp(request.role, "fire") == 0);
}

static void test_cancellation_without_lane(void)
{
    static const char message[] =
        "{\"messageId\":29,\"value\":{\"SignalRequestMessage\":{"
        "\"second\":1,\"requests\":[{\"request\":{\"id\":{\"id\":1},"
        "\"requestID\":7,\"requestType\":\"priorityCancellation\"}}],"
        "\"requestor\":{\"id\":{\"entityID\":\"01020304\"},"
        "\"type\":{\"role\":\"ambulance\"}}}}}";
    v2x_srm_request_t request;
    assert(v2x_parse_srm_message(
        message,
        strlen(message),
        1,
        &request,
        NULL,
        0));
    assert(request.request_type == V2X_SRM_PRIORITY_CANCELLATION);
    assert(request.inbound_lane == 0);
    assert(request.signal_group == 0);
}

static void test_rejects_invalid_requests(void)
{
    char message[sizeof(VALID_SRM)];
    v2x_srm_request_t request;

    memcpy(message, VALID_SRM, sizeof(message));
    char *role = strstr(message, "\"fire\"");
    assert(role != NULL);
    memcpy(role, "\"bus \"", 6);
    assert(!v2x_parse_srm_message(
        message,
        strlen(message),
        1,
        &request,
        NULL,
        0));

    assert(!v2x_parse_srm_message(
        VALID_SRM,
        strlen(VALID_SRM),
        2,
        &request,
        NULL,
        0));
}

static void test_ssm_format(void)
{
    v2x_srm_request_t request;
    char error[128];
    char output[J2735_SSM_MESSAGE_CAPACITY];
    assert(v2x_parse_srm_message(
        VALID_SRM,
        strlen(VALID_SRM),
        1,
        &request,
        error,
        sizeof(error)));

    v2x_ssm_update_t update = {
        .minute_of_year = 12345,
        .second = 23456,
        .sequence_number = 10,
        .intersection_id = 1,
        .request = &request,
        .status = V2X_SSM_GRANTED,
    };
    assert(v2x_format_ssm_message(output, sizeof(output), &update) > 0);
    assert(strstr(output, "\"messageId\":30") != NULL);
    assert(strstr(output, "\"entityID\":\"01AB03CD\"") != NULL);
    assert(strstr(output, "\"status\":\"granted\"") != NULL);
}

int main(void)
{
    test_valid_request();
    test_cancellation_without_lane();
    test_rejects_invalid_requests();
    test_ssm_format();
    puts("v2x message tests passed");
    return 0;
}
