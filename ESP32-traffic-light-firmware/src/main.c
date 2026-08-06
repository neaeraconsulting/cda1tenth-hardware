#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "j2735_config.h"
#include "j2735_messages.h"
#include "led_strip.h"
#include "mqtt_config.h"
#include "mqtt_publisher.h"

#define STATUS_LED_PIN GPIO_NUM_8
#define TRAFFIC_LIGHT_PIN GPIO_NUM_3
#define SIGNAL_COUNT 4
#define PIXELS_PER_SIGNAL 3
#define TRAFFIC_LIGHT_PIXEL_COUNT (SIGNAL_COUNT * PIXELS_PER_SIGNAL)
#define BRIGHTNESS 64
#define GREEN_DURATION_MS 10000
#define YELLOW_DURATION_MS 2000
#define ALL_RED_DURATION_MS 1000
#define MIN_GREEN_BEFORE_PREEMPT_MS 3000
#define MIN_PREEMPT_GREEN_MS 5000
#define PREEMPT_DEFAULT_LEASE_MS 10000
#define PREEMPT_MIN_LEASE_MS 2000
#define PREEMPT_MAX_LEASE_MS 30000
#define SPAT_PUBLISH_INTERVAL_MS 100
#define SSM_PUBLISH_INTERVAL_MS 100
#define STATUS_TOGGLE_INTERVAL_US 500000
#define LED_STRIP_RESOLUTION_HZ (10 * 1000 * 1000)
#define INTERSECTION_STATUS_NORMAL "0000"
#define INTERSECTION_STATUS_PREEMPT_ACTIVE "1000"

static const char *TAG = "traffic_light";

typedef enum {
    NS_GREEN,
    NS_YELLOW,
    ALL_RED_BEFORE_EW,
    EW_GREEN,
    EW_YELLOW,
    ALL_RED_BEFORE_NS,
} intersection_phase_t;

typedef enum {
    SIGNAL_RED,
    SIGNAL_YELLOW,
    SIGNAL_GREEN,
} signal_color_t;

typedef struct {
    bool synchronized;
    uint32_t minute_of_year;
    uint16_t millisecond_of_minute;
    uint32_t millisecond_of_hour;
} spat_time_t;

typedef struct {
    bool present;
    bool granted;
    bool clear_pending;
    int64_t expires_us;
    int64_t granted_us;
    v2x_ssm_status_t response_status;
    v2x_srm_request_t request;
} preemption_state_t;

static led_strip_handle_t configure_strip(gpio_num_t gpio, uint32_t pixel_count)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio,
        .max_leds = pixel_count,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = LED_STRIP_RESOLUTION_HZ,
        .flags.with_dma = false,
    };
    led_strip_handle_t strip = NULL;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    return strip;
}

static uint32_t phase_duration_ms(intersection_phase_t phase)
{
    return (phase == NS_YELLOW || phase == EW_YELLOW) ? YELLOW_DURATION_MS :
           (phase == NS_GREEN || phase == EW_GREEN) ? GREEN_DURATION_MS :
           ALL_RED_DURATION_MS;
}

static intersection_phase_t next_automatic_phase(intersection_phase_t phase)
{
    return phase == ALL_RED_BEFORE_NS ? NS_GREEN : phase + 1;
}

static intersection_phase_t next_phase(
    intersection_phase_t phase,
    mqtt_light_command_t command)
{
    if (command == MQTT_LIGHT_COMMAND_TRAFFIC) {
        return next_automatic_phase(phase);
    }

    intersection_phase_t target_green =
        command == MQTT_LIGHT_COMMAND_NORTH_SOUTH ? NS_GREEN : EW_GREEN;
    if (phase == target_green) {
        return target_green;
    }
    if (phase == NS_GREEN) {
        return NS_YELLOW;
    }
    if (phase == EW_GREEN) {
        return EW_YELLOW;
    }
    if (phase == NS_YELLOW || phase == EW_YELLOW) {
        return command == MQTT_LIGHT_COMMAND_NORTH_SOUTH ?
            ALL_RED_BEFORE_NS :
            ALL_RED_BEFORE_EW;
    }
    return target_green;
}

static bool green_phase(intersection_phase_t phase)
{
    return phase == NS_GREEN || phase == EW_GREEN;
}

static intersection_phase_t preemption_target_green(uint8_t signal_group)
{
    return signal_group == 1 ? NS_GREEN : EW_GREEN;
}

static intersection_phase_t yellow_for_green(intersection_phase_t green)
{
    return green == NS_GREEN ? NS_YELLOW : EW_YELLOW;
}

static intersection_phase_t all_red_before_green(intersection_phase_t green)
{
    return green == NS_GREEN ? ALL_RED_BEFORE_NS : ALL_RED_BEFORE_EW;
}

static const char *phase_status(intersection_phase_t phase)
{
    switch (phase) {
    case NS_GREEN:
        return "north-south-green";
    case NS_YELLOW:
        return "north-south-yellow";
    case EW_GREEN:
        return "east-west-green";
    case EW_YELLOW:
        return "east-west-yellow";
    default:
        return "all-red";
    }
}

static signal_color_t signal_color(intersection_phase_t phase, uint8_t signal_group)
{
    bool ns_movement = signal_group == 1;

    if ((ns_movement && phase == NS_GREEN) || (!ns_movement && phase == EW_GREEN)) {
        return SIGNAL_GREEN;
    }
    if ((ns_movement && phase == NS_YELLOW) || (!ns_movement && phase == EW_YELLOW)) {
        return SIGNAL_YELLOW;
    }
    return SIGNAL_RED;
}

static const char *signal_state(intersection_phase_t phase, uint8_t signal_group)
{
    static const char *const states[] = {
        "stop-And-Remain",
        "protected-clearance",
        "protected-Movement-Allowed",
    };
    return states[signal_color(phase, signal_group)];
}

static uint32_t state_remaining_ms(intersection_phase_t phase, uint8_t signal_group, uint32_t phase_remaining_ms)
{
    signal_color_t color = signal_color(phase, signal_group);
    uint32_t remaining_ms = phase_remaining_ms;

    phase = next_automatic_phase(phase);
    while (signal_color(phase, signal_group) == color) {
        remaining_ms += phase_duration_ms(phase);
        phase = next_automatic_phase(phase);
    }
    return remaining_ms;
}

static spat_time_t get_spat_time(void)
{
    struct timeval wall_time;
    gettimeofday(&wall_time, NULL);

    spat_time_t result = {
        .synchronized = false,
        .minute_of_year = J2735_MINUTE_OF_YEAR_UNAVAILABLE,
        .millisecond_of_minute = J2735_DSECOND_UNAVAILABLE,
        .millisecond_of_hour = 0,
    };
    if (wall_time.tv_sec >= 1704067200) {
        struct tm utc;
        gmtime_r(&wall_time.tv_sec, &utc);
        result.synchronized = true;
        result.minute_of_year = utc.tm_yday * 1440 + utc.tm_hour * 60 + utc.tm_min;
        result.millisecond_of_minute = utc.tm_sec * 1000 + wall_time.tv_usec / 1000;
        result.millisecond_of_hour =
            utc.tm_min * 60000 + utc.tm_sec * 1000 + wall_time.tv_usec / 1000;
    }
    return result;
}

static uint16_t end_time_mark(const spat_time_t *now, uint32_t remaining_ms)
{
    if (!now->synchronized) {
        return J2735_TIME_MARK_UNKNOWN;
    }

    uint64_t end_ms = (uint64_t)now->millisecond_of_hour + remaining_ms;
    return (uint16_t)(((end_ms + 99) / 100) % 36000);
}

static void publish_spat(
    intersection_phase_t phase,
    int64_t phase_started_us,
    int64_t now_us,
    bool preempt_active)
{
    static char message[J2735_SPAT_MESSAGE_CAPACITY];
    static uint8_t revision;
    int64_t phase_duration_us = (int64_t)phase_duration_ms(phase) * 1000;
    int64_t remaining_us = phase_duration_us - (now_us - phase_started_us);
    uint32_t phase_remaining_ms =
        remaining_us > 0 ? (uint32_t)((remaining_us + 999) / 1000) : 0;
    spat_time_t now = get_spat_time();
    j2735_spat_update_t update = {
        .revision = revision,
        .minute_of_year = now.minute_of_year,
        .millisecond_of_minute = now.millisecond_of_minute,
        .intersection_status = preempt_active ?
            INTERSECTION_STATUS_PREEMPT_ACTIVE :
            INTERSECTION_STATUS_NORMAL,
    };
    for (uint8_t group = 1; group <= J2735_SIGNAL_GROUP_COUNT; ++group) {
        update.movements[group - 1].state = signal_state(phase, group);
        update.movements[group - 1].end_time = preempt_active ?
            J2735_TIME_MARK_UNKNOWN :
            end_time_mark(
                &now,
                state_remaining_ms(
                    phase,
                    group,
                    phase_remaining_ms));
    }

    if (j2735_format_spat_message(message, sizeof(message), &update) < 0) {
        ESP_LOGE(TAG, "SPAT JSON buffer is too small");
        return;
    }
    revision = (revision + 1) & 0x7f;
    mqtt_publisher_publish_spat(message);
}

static void publish_ssm_response(
    const v2x_srm_request_t *request,
    v2x_ssm_status_t status)
{
    static char message[J2735_SSM_MESSAGE_CAPACITY];
    static uint8_t sequence_number;
    spat_time_t now = get_spat_time();
    v2x_ssm_update_t update = {
        .minute_of_year = now.minute_of_year,
        .second = now.millisecond_of_minute,
        .sequence_number = sequence_number,
        .intersection_id = J2735_INTERSECTION_ID,
        .request = request,
        .status = status,
    };

    if (v2x_format_ssm_message(message, sizeof(message), &update) < 0) {
        ESP_LOGE(TAG, "SSM JSON buffer is too small");
        return;
    }
    sequence_number = (sequence_number + 1) & 0x7f;
    mqtt_publisher_publish_ssm(message);
}

static void set_signal(led_strip_handle_t strip, uint8_t signal, signal_color_t color)
{
    uint32_t first_pixel = signal * PIXELS_PER_SIGNAL;
    uint8_t red = color == SIGNAL_RED ? BRIGHTNESS : 0;
    uint8_t yellow = color == SIGNAL_YELLOW ? BRIGHTNESS : 0;
    uint8_t green = color == SIGNAL_GREEN ? BRIGHTNESS : 0;
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, first_pixel, red, 0, 0));
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, first_pixel + 1, yellow, yellow, 0));
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, first_pixel + 2, 0, green, 0));
}

static void show_intersection_phase(led_strip_handle_t strip, intersection_phase_t phase)
{
    // Signals 0 and 2 face each other; signals 1 and 3 are the cross street.
    signal_color_t north_south = signal_color(phase, 1);
    signal_color_t east_west = signal_color(phase, 2);
    set_signal(strip, 0, north_south);
    set_signal(strip, 1, east_west);
    set_signal(strip, 2, north_south);
    set_signal(strip, 3, east_west);
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}

static bool same_request(
    const v2x_srm_request_t *left,
    const v2x_srm_request_t *right)
{
    return left->request_id == right->request_id &&
           strcmp(left->entity_id, right->entity_id) == 0;
}

static uint32_t request_lease_ms(const v2x_srm_request_t *request)
{
    uint32_t lease_ms =
        request->has_duration && request->duration_deciseconds > 0 ?
        (uint32_t)request->duration_deciseconds * 100 :
        PREEMPT_DEFAULT_LEASE_MS;

    if (lease_ms < PREEMPT_MIN_LEASE_MS) {
        return PREEMPT_MIN_LEASE_MS;
    }
    if (lease_ms > PREEMPT_MAX_LEASE_MS) {
        return PREEMPT_MAX_LEASE_MS;
    }
    return lease_ms;
}

static void reject_srm(const v2x_srm_request_t *request, const char *reason)
{
    ESP_LOGW(
        TAG,
        "Rejected SRM entity=%s request=%u: %s",
        request->entity_id,
        request->request_id,
        reason);
    publish_ssm_response(request, V2X_SSM_REJECTED);
}

static void handle_srm(
    preemption_state_t *preemption,
    const v2x_srm_request_t *request,
    intersection_phase_t phase,
    int64_t now_us)
{
    if (strcasecmp(request->entity_id, V2X_AUTHORIZED_ENTITY_ID) != 0) {
        reject_srm(request, "entityID is not authorized");
        return;
    }

    bool matches =
        preemption->present &&
        same_request(&preemption->request, request);
    if (request->request_type == V2X_SRM_PRIORITY_CANCELLATION) {
        if (!matches) {
            reject_srm(request, "no matching active request");
            return;
        }

        preemption->clear_pending = true;
        preemption->response_status = V2X_SSM_PROCESSING;
        publish_ssm_response(&preemption->request, preemption->response_status);
        ESP_LOGI(
            TAG,
            "Accepted SRM cancellation entity=%s request=%u",
            request->entity_id,
            request->request_id);
        if (!preemption->granted) {
            preemption->present = false;
        }
        return;
    }

    if (request->request_type == V2X_SRM_PRIORITY_REQUEST_UPDATE && !matches) {
        reject_srm(request, "update has no matching active request");
        return;
    }
    if (preemption->present && !matches) {
        reject_srm(request, "another emergency request is active");
        return;
    }
    if (matches &&
        request->signal_group != preemption->request.signal_group) {
        reject_srm(request, "an update cannot change the requested movement");
        return;
    }

    bool already_granted = matches && preemption->granted;
    uint32_t lease_ms = request_lease_ms(request);
    preemption->present = true;
    preemption->clear_pending = false;
    preemption->request = *request;
    preemption->request.has_duration = true;
    preemption->request.duration_deciseconds = (uint16_t)(lease_ms / 100);
    preemption->expires_us = now_us + (int64_t)lease_ms * 1000;
    intersection_phase_t target =
        preemption_target_green(request->signal_group);
    preemption->granted = already_granted || phase == target;
    if (!already_granted && phase == target) {
        preemption->granted_us = now_us;
    }
    preemption->response_status =
        preemption->granted ?
        V2X_SSM_GRANTED :
        V2X_SSM_PROCESSING;

    ESP_LOGI(
        TAG,
        "%s SRM entity=%s request=%u lane=%u group=%u lease=%lu ms",
        matches ? "Updated" : "Accepted",
        request->entity_id,
        request->request_id,
        request->inbound_lane,
        request->signal_group,
        (unsigned long)lease_ms);
    publish_ssm_response(&preemption->request, preemption->response_status);
}

/*
 * Advance a preemption without ever skipping yellow or all-red. Returns true
 * when it changed the physical phase.
 */
static bool advance_preemption(
    preemption_state_t *preemption,
    intersection_phase_t *phase,
    int64_t *phase_started_us,
    int64_t now_us)
{
    if (!preemption->present) {
        return false;
    }

    if (!preemption->clear_pending && now_us >= preemption->expires_us) {
        preemption->clear_pending = true;
        preemption->response_status = V2X_SSM_MAX_PRESENCE;
        ESP_LOGW(
            TAG,
            "SRM lease expired entity=%s request=%u",
            preemption->request.entity_id,
            preemption->request.request_id);
        publish_ssm_response(
            &preemption->request,
            preemption->response_status);
        if (!preemption->granted) {
            preemption->present = false;
            return false;
        }
    }

    intersection_phase_t target =
        preemption_target_green(preemption->request.signal_group);
    if (preemption->granted) {
        if (!preemption->clear_pending ||
            now_us - preemption->granted_us <
                (int64_t)MIN_PREEMPT_GREEN_MS * 1000) {
            return false;
        }

        *phase = yellow_for_green(target);
        *phase_started_us = now_us;
        ESP_LOGI(
            TAG,
            "Emergency service ended; entering clearance for group %u",
            preemption->request.signal_group);
        preemption->present = false;
        return true;
    }

    if (preemption->clear_pending) {
        preemption->present = false;
        return false;
    }

    if (*phase == target) {
        preemption->granted = true;
        preemption->granted_us = now_us;
        preemption->response_status = V2X_SSM_GRANTED;
        publish_ssm_response(
            &preemption->request,
            preemption->response_status);
        return false;
    }

    int64_t elapsed_us = now_us - *phase_started_us;
    uint32_t required_ms = green_phase(*phase) ?
        MIN_GREEN_BEFORE_PREEMPT_MS :
        phase_duration_ms(*phase);
    if (elapsed_us < (int64_t)required_ms * 1000) {
        return false;
    }

    if (green_phase(*phase)) {
        *phase = yellow_for_green(*phase);
    } else if (*phase == NS_YELLOW || *phase == EW_YELLOW) {
        *phase = all_red_before_green(target);
    } else {
        *phase = target;
        preemption->granted = true;
        preemption->granted_us = now_us;
        preemption->response_status = V2X_SSM_GRANTED;
        publish_ssm_response(
            &preemption->request,
            preemption->response_status);
    }
    *phase_started_us = now_us;
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "CDA1Tenth traffic light firmware starting for ESP32-C6-DevKitC-1");
    ESP_LOGI(TAG, "Traffic light strip: GPIO %d, pixels: %d", TRAFFIC_LIGHT_PIN, TRAFFIC_LIGHT_PIXEL_COUNT);
    ESP_LOGI(TAG, "Board RGB status LED: GPIO %d", STATUS_LED_PIN);

    led_strip_handle_t traffic_strip = configure_strip(TRAFFIC_LIGHT_PIN, TRAFFIC_LIGHT_PIXEL_COUNT);
    led_strip_handle_t status_strip = configure_strip(STATUS_LED_PIN, 1);
    ESP_ERROR_CHECK(led_strip_clear(traffic_strip));
    ESP_ERROR_CHECK(led_strip_clear(status_strip));

    intersection_phase_t phase = NS_GREEN;
    mqtt_light_command_t control_mode = MQTT_LIGHT_COMMAND_TRAFFIC;
    show_intersection_phase(traffic_strip, phase);
    int64_t phase_started_us = esp_timer_get_time();
    int64_t last_status_toggle_us = phase_started_us;
    int64_t last_spat_publish_us = phase_started_us - (int64_t)SPAT_PUBLISH_INTERVAL_MS * 1000;
    int64_t last_ssm_publish_us = phase_started_us;
    bool status_led_on = false;
    preemption_state_t preemption = {0};
    static char map_message[J2735_MAP_MESSAGE_CAPACITY];
    bool map_message_ready = j2735_format_map_message(map_message, sizeof(map_message)) >= 0;
    if (!map_message_ready) {
        ESP_LOGE(TAG, "MAP JSON buffer is too small");
    }
    mqtt_publisher_init(map_message_ready ? map_message : NULL);
    mqtt_publisher_set_status(phase_status(phase));

    while (true) {
        int64_t now_us = esp_timer_get_time();
        mqtt_light_command_t command;
        if (mqtt_publisher_take_command(&command)) {
            control_mode = command;
            if (command == MQTT_LIGHT_COMMAND_TRAFFIC) {
                if (!preemption.present) {
                    phase_started_us = now_us;
                }
                ESP_LOGI(TAG, "MQTT command: automatic traffic cycle");
            } else if (command == MQTT_LIGHT_COMMAND_NORTH_SOUTH) {
                ESP_LOGI(TAG, "MQTT command: request north/south");
            } else {
                ESP_LOGI(TAG, "MQTT command: request east/west");
            }
        }

        v2x_srm_request_t srm;
        while (mqtt_publisher_take_srm(&srm)) {
            handle_srm(&preemption, &srm, phase, now_us);
        }

        bool phase_changed = advance_preemption(
            &preemption,
            &phase,
            &phase_started_us,
            now_us);
        if (!phase_changed &&
            !preemption.present &&
            now_us - phase_started_us >=
                (int64_t)phase_duration_ms(phase) * 1000) {
            phase = next_phase(phase, control_mode);
            phase_started_us = now_us;
            phase_changed = true;
        }
        if (phase_changed) {
            show_intersection_phase(traffic_strip, phase);
            mqtt_publisher_set_status(phase_status(phase));
        }

        if (now_us - last_spat_publish_us >= (int64_t)SPAT_PUBLISH_INTERVAL_MS * 1000) {
            last_spat_publish_us = now_us;
            publish_spat(
                phase,
                phase_started_us,
                now_us,
                preemption.present);
        }

        if (preemption.present &&
            now_us - last_ssm_publish_us >=
                (int64_t)SSM_PUBLISH_INTERVAL_MS * 1000) {
            last_ssm_publish_us = now_us;
            publish_ssm_response(
                &preemption.request,
                preemption.response_status);
        }

        if (now_us - last_status_toggle_us >= STATUS_TOGGLE_INTERVAL_US) {
            last_status_toggle_us = now_us;
            status_led_on = !status_led_on;
            ESP_ERROR_CHECK(led_strip_set_pixel(status_strip, 0, 0, 0, status_led_on ? BRIGHTNESS : 0));
            ESP_ERROR_CHECK(led_strip_refresh(status_strip));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
