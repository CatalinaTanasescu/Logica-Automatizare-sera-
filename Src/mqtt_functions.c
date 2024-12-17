#include "mongoose.h"
#include "bmp180.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include <stdio.h>

#define MQTT_BROKER "192.168.1.101"  // IP-ul Raspberry Pi cu EMQX
#define MQTT_TOPIC "temperature/sensor1"

static struct mg_mgr mgr;       // Managerul pentru conexiuni
extern struct netif gnetif;     // Structura netif definită de LWIP

// Callback pentru evenimente MQTT
static void mqtt_event_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_MQTT_OPEN) {
        printf("Conectat la broker MQTT!\n");
    } else if (ev == MG_EV_CLOSE) {
        printf("Conexiunea MQTT s-a închis.\n");
    }
}

// Publică temperatura pe topicul MQTT
void mqtt_publish_message(float temperature) {
    // Creează URL-ul brokerului
    char url[64];
    snprintf(url, sizeof(url), "mqtt://%s:1883", MQTT_BROKER);

    // Construiește payload-ul JSON
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"temperature\": %.2f}", temperature);

    // Opțiuni MQTT
    struct mg_mqtt_opts opts = {
        .topic = mg_str(MQTT_TOPIC),
        .message = mg_str(payload),
    };

    // Inițializează conexiunea MQTT
    struct mg_connection *c = mg_mqtt_connect(&mgr, url, &opts, mqtt_event_handler, NULL);
    if (c == NULL) {
        printf("Eroare: Nu s-a putut conecta la broker MQTT.\n");
    }
}

// Inițializează managerul MQTT
void mqtt_init(void) {
    mg_mgr_init(&mgr);
    printf("Manager MQTT inițializat.\n");
}

// Procesează evenimentele MQTT
void mqtt_poll(int ms) {
    mg_mgr_poll(&mgr, ms);
}

// Curăță managerul MQTT
void mqtt_cleanup(void) {
    mg_mgr_free(&mgr);
    printf("Manager MQTT curățat.\n");
}

// Afișează adresa IP a dispozitivului
void print_ip_address(void) {
    printf("IP Address: %s\n", ipaddr_ntoa(&gnetif.ip_addr));
}

// Citește temperatura de la BMP180
float read_temperature(void) {
    return BMP180_ReadTemperature();
}


// Funcție pentru timpul în milisecunde
uint64_t mg_millis(void) {
    return HAL_GetTick();  // Timpul curent în milisecunde
}

// Funcție pentru generare de date pseudo-aleatoare
bool mg_random(void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = (uint8_t)(HAL_GetTick() & 0xFF);  // Pseudo-random bazat pe tick counter
    }
    return true;  // Semnalează succesul
}
