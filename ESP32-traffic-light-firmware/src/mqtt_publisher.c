#include "mqtt_publisher.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

#include "j2735_config.h"
#include "mqtt_config.h"

static const char *TAG = "mqtt_publisher";

static esp_mqtt_client_handle_t mqtt_client;
static QueueHandle_t command_queue;
static QueueHandle_t srm_queue;
static const char *retained_map_message;
static const char *retained_status_message;
static volatile bool mqtt_connected;
static bool time_sync_started;
static bool spat_enqueue_failed;

static bool text_equals(const char *value, int length, const char *expected)
{
    size_t expected_length = strlen(expected);
    return length >= 0 &&
           (size_t)length == expected_length &&
           memcmp(value, expected, expected_length) == 0;
}

static void enqueue_command(const esp_mqtt_event_t *event)
{
    if (command_queue == NULL ||
        event->current_data_offset != 0 ||
        event->data_len != event->total_data_len) {
        return;
    }
    if (event->retain) {
        ESP_LOGW(TAG, "Ignoring retained light command");
        return;
    }

    mqtt_light_command_t command;
    if (text_equals(event->data, event->data_len, "traffic")) {
        command = MQTT_LIGHT_COMMAND_TRAFFIC;
    } else if (event->data_len == 1 &&
               event->data[0] >= '1' &&
               event->data[0] <= '4') {
        command = (event->data[0] - '0') % 2 ?
            MQTT_LIGHT_COMMAND_NORTH_SOUTH :
            MQTT_LIGHT_COMMAND_EAST_WEST;
    } else {
        ESP_LOGW(TAG, "Ignoring unsupported light command: %.*s", event->data_len, event->data);
        return;
    }

    xQueueOverwrite(command_queue, &command);
}

static void enqueue_srm(const esp_mqtt_event_t *event)
{
    if (srm_queue == NULL ||
        event->current_data_offset != 0 ||
        event->data_len != event->total_data_len) {
        return;
    }
    if (event->retain) {
        ESP_LOGW(TAG, "Ignoring retained SRM");
        return;
    }

    v2x_srm_request_t request;
    char error[128];
    if (!v2x_parse_srm_message(
            event->data,
            (size_t)event->data_len,
            J2735_INTERSECTION_ID,
            &request,
            error,
            sizeof(error))) {
        ESP_LOGW(TAG, "Ignoring invalid SRM: %s", error);
        return;
    }

    if (xQueueSend(srm_queue, &request, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Dropping SRM because the receive queue is full");
    }
}

static int enqueue_message(
    const char *topic,
    const char *message,
    int qos,
    bool retain)
{
    if (!mqtt_connected || message == NULL) {
        return 0;
    }

    return esp_mqtt_client_enqueue(
        mqtt_client,
        topic,
        message,
        0,
        qos,
        retain ? 1 : 0,
        true);
}

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_args;
    (void)base;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;
        ESP_LOGI(TAG, "Connected to MQTT broker %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
        if (esp_mqtt_client_subscribe(mqtt_client, MQTT_COMMAND_TOPIC, 0) < 0) {
            ESP_LOGE(TAG, "Unable to subscribe to %s", MQTT_COMMAND_TOPIC);
        } else {
            ESP_LOGI(TAG, "Subscribed to light commands on %s", MQTT_COMMAND_TOPIC);
        }
        if (esp_mqtt_client_subscribe(mqtt_client, MQTT_SRM_TOPIC, 0) < 0) {
            ESP_LOGE(TAG, "Unable to subscribe to %s", MQTT_SRM_TOPIC);
        } else {
            ESP_LOGI(TAG, "Subscribed to J2735 SRMs on %s", MQTT_SRM_TOPIC);
        }
        if (enqueue_message(MQTT_MAP_TOPIC, retained_map_message, 1, true) < 0) {
            ESP_LOGE(TAG, "Unable to enqueue retained MAP");
        }
        if (enqueue_message(MQTT_STATUS_TOPIC, retained_status_message, 0, true) < 0) {
            ESP_LOGW(TAG, "Unable to enqueue retained light status");
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        break;
    case MQTT_EVENT_ERROR:
        mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT transport error");
        break;
    case MQTT_EVENT_DATA: {
        esp_mqtt_event_handle_t event = event_data;
        if (event->topic != NULL &&
            text_equals(event->topic, event->topic_len, MQTT_COMMAND_TOPIC)) {
            enqueue_command(event);
        } else if (event->topic != NULL &&
                   text_equals(event->topic, event->topic_len, MQTT_SRM_TOPIC)) {
            enqueue_srm(event);
        }
        break;
    }
    default:
        break;
    }
}

static void start_time_sync(void)
{
    if (time_sync_started) {
        return;
    }

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_SERVER);
    esp_err_t error = esp_netif_sntp_init(&config);
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Unable to start SNTP: %s", esp_err_to_name(error));
        return;
    }

    time_sync_started = true;
    ESP_LOGI(TAG, "SNTP started with server %s", SNTP_SERVER);
}

static void wifi_event_handler(
    void *handler_args,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_args;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to Wi-Fi SSID %s", MQTT_WIFI_SSID);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        mqtt_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason %u); reconnecting", event->reason);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP " IPSTR, IP2STR(&event->ip_info.ip));
        start_time_sync();
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        error = nvs_flash_erase();
        if (error != ESP_OK) {
            return error;
        }
        error = nvs_flash_init();
    }
    return error;
}

void mqtt_publisher_init(const char *map_message)
{
    retained_map_message = map_message;
    if (MQTT_WIFI_SSID[0] == '\0') {
        ESP_LOGW(TAG, "MQTT disabled: configure src/secrets.h");
        return;
    }

    command_queue = xQueueCreate(1, sizeof(mqtt_light_command_t));
    srm_queue = xQueueCreate(8, sizeof(v2x_srm_request_t));
    if (command_queue == NULL || srm_queue == NULL) {
        ESP_LOGE(TAG, "Unable to create the MQTT receive queues");
        return;
    }

    esp_err_t error = init_nvs();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize NVS: %s", esp_err_to_name(error));
        return;
    }

    if (esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize the network event loop");
        return;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Unable to create the Wi-Fi station interface");
        return;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wifi_init) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize Wi-Fi");
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL));

    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.hostname = MQTT_BROKER_HOST,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .broker.address.port = MQTT_BROKER_PORT,
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .buffer.size = MQTT_BUFFER_SIZE,
        .buffer.out_size = MQTT_BUFFER_SIZE,
        .outbox.limit = MQTT_OUTBOX_LIMIT_BYTES,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Unable to initialize MQTT client");
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, MQTT_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, MQTT_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());
    error = esp_mqtt_client_start(mqtt_client);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Unable to start MQTT client: %s", esp_err_to_name(error));
    }

    ESP_LOGI(
        TAG,
        "MQTT enabled: broker=%s:%d SPAT=%s MAP=%s SRM=%s SSM=%s COMMAND=%s STATUS=%s",
        MQTT_BROKER_HOST,
        MQTT_BROKER_PORT,
        MQTT_SPAT_TOPIC,
        MQTT_MAP_TOPIC,
        MQTT_SRM_TOPIC,
        MQTT_SSM_TOPIC,
        MQTT_COMMAND_TOPIC,
        MQTT_STATUS_TOPIC);
}

bool mqtt_publisher_take_command(mqtt_light_command_t *command)
{
    return command != NULL &&
           command_queue != NULL &&
           xQueueReceive(command_queue, command, 0) == pdTRUE;
}

bool mqtt_publisher_take_srm(v2x_srm_request_t *request)
{
    return request != NULL &&
           srm_queue != NULL &&
           xQueueReceive(srm_queue, request, 0) == pdTRUE;
}

void mqtt_publisher_publish_spat(const char *spat_message)
{
    int message_id = enqueue_message(MQTT_SPAT_TOPIC, spat_message, 0, true);
    if (message_id < 0) {
        if (!spat_enqueue_failed) {
            ESP_LOGW(TAG, "Unable to enqueue SPAT message: %d", message_id);
        }
        spat_enqueue_failed = true;
    } else {
        spat_enqueue_failed = false;
    }
}

void mqtt_publisher_publish_ssm(const char *ssm_message)
{
    if (enqueue_message(MQTT_SSM_TOPIC, ssm_message, 0, false) < 0) {
        ESP_LOGW(TAG, "Unable to enqueue SSM");
    }
}

void mqtt_publisher_set_status(const char *status)
{
    retained_status_message = status;
    if (enqueue_message(MQTT_STATUS_TOPIC, status, 0, true) < 0) {
        ESP_LOGW(TAG, "Unable to enqueue retained light status");
    }
}
