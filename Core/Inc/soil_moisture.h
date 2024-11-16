/*
 * soilMoisture.h
 *
 *  Created on: Nov 9, 2024
 *      Author: Marmelada
 */

#ifndef INC_SOIL_MOISTURE_H_
#define INC_SOIL_MOISTURE_H_

#include "stm32h5xx_hal.h"

// Function prototypes
void SoilMoisture_Init(void);
uint32_t SoilMoisture_ReadAnalog(void);
uint8_t SoilMoisture_ReadDigital(void);
void SoilMoisture_DisplayData(void);

#endif /* INC_SOIL_MOISTURE_H_ */
