/*
 * fan_control.h
 *
 *  Created on: Nov 9, 2024
 *      Author: Marmelada
 */

#ifndef INC_FAN_CONTROL_H_
#define INC_FAN_CONTROL_H_

#include "stm32h5xx_hal.h"  // Include HAL library

// Define the temperature threshold for turning on the fan
#define FAN_TEMP_THRESHOLD 25.0

// Function prototypes
void Fan_Init(void);
void Fan_Control(uint8_t state);

#endif /* INC_FAN_CONTROL_H_ */
