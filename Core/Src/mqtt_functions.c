#include "mongoose.h"
#include "bmp180.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include <stdio.h>
#include "stm32h7xx_hal.h"

#define MQTT_BROKER "192.168.0.100"
#define MQTT_PORT 1883
#define MQTT_TOPIC "temperature/sensor1"

static struct mg_mgr mgr;  // Managerul pentru conexiuni
static struct mg_connection *mqtt_conn = NULL; // Pointer la conexiunea MQTT
extern struct netif gnetif;

// Prototipuri de funcții
void mqtt_init(void);

// Callback pentru evenimente MQTT
static void mqtt_event_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_MQTT_OPEN) {
        printf("Conectat la broker MQTT!\n");
    } else if (ev == MG_EV_CLOSE) {
        printf("Conexiunea MQTT s-a închis. Încercare reconectare...\n");
        mqtt_conn = NULL; // Marchează conexiunea ca invalidă
        HAL_Delay(5000);
        mqtt_init();  // Reîncearcă inițializarea
    } else if (ev == MG_EV_MQTT_MSG) {
        struct mg_mqtt_message *msg = (struct mg_mqtt_message *)ev_data;
        printf("Mesaj primit pe topic: %.*s, Payload: %.*s\n",
               (int)msg->topic.len, msg->topic.buf,
               (int)msg->data.len, msg->data.buf);
    }
}

void mqtt_init(void) {
    mg_mgr_init(&mgr);
    char url[64];
    snprintf(url, sizeof(url), "mqtt://%s:%d", MQTT_BROKER, MQTT_PORT);
    struct mg_mqtt_opts opts = {
        .qos = 1,
    };
    mqtt_conn = mg_mqtt_connect(&mgr, url, &opts, mqtt_event_handler, NULL);
    if (!mqtt_conn) {
        printf("Eroare: Nu s-a putut conecta la broker MQTT.\n");
    }
}

void mqtt_publish_message(float temperature) {
    if (!mqtt_conn) {
        printf("Eroare: MQTT nu este conectat.\n");
        return;
    }
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"temperature\": %.2f}", temperature);

    // ✅ Folosește corect structura mg_mqtt_opts
    struct mg_mqtt_opts pub_opts = {
        .topic = mg_str(MQTT_TOPIC),
        .message = mg_str(payload),
        .qos = 1
    };
    mg_mqtt_pub(mqtt_conn, &pub_opts);
}

void mqtt_poll(int ms) {
    mg_mgr_poll(&mgr, ms);
}

void mqtt_cleanup(void) {
    mg_mgr_free(&mgr);
    printf("Manager MQTT curățat.\n");
}

void print_ip_address(void) {
    printf("IP Address: %s\n", ipaddr_ntoa(&gnetif.ip_addr));
}

float read_temperature(void) {
    return BMP180_ReadTemperature();
}

uint64_t mg_millis(void) {
    return HAL_GetTick();
}

// ✅ RNG îmbunătățit fără hardware RNG
bool mg_random(void *buf, size_t len) {
    static uint32_t seed = 0x12345678;
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        p[i] = (uint8_t)(seed & 0xFF);
    }
    return true;
}
