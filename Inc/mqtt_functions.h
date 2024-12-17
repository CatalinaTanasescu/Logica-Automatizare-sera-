/*
 * mqtt_functions.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Marmelada
 */

#ifndef INC_MQTT_FUNCTIONS_H_
#define INC_MQTT_FUNCTIONS_H_

#include "mongoose.h"
#include <stdint.h>

// Funcție pentru inițializarea managerului MQTT
void mqtt_init(void);

// Funcție pentru publicarea unui mesaj MQTT (temperatură)
void mqtt_publish_message(float temperature);

// Funcție pentru procesarea evenimentelor MQTT
void mqtt_poll(int ms);

// Funcție pentru eliberarea managerului MQTT
void mqtt_cleanup(void);

// Funcție pentru afișarea adresei IP a STM32
void print_ip_address(void);

// Funcție pentru citirea temperaturii de la senzorul BMP180
float read_temperature(void);

#endif /* INC_MQTT_FUNCTIONS_H_ */
