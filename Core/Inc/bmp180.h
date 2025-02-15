/*
 * bmp180.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Marmelada
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_



#include "main.h"

// I2C address for BMP180
#define BMP180_I2C_ADDR             0x77
#define CONTROL_REGISTER_VALUE      0xF4
#define BMP180_WRITE                (BMP180_I2C_ADDR << 1)
#define BMP180_READ                 ((BMP180_I2C_ADDR << 1) | 1)
#define MAX_NO_TEMP                 4

// Structure for storing calibration data
typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} Bmp180CalibrationData;

// Prototypes for pressure functions
int32_t BMP180_ReadUncompensatedPressure(uint8_t oss);

int32_t BMP180_CalculatePressure(int32_t up, int32_t b5, uint8_t oss, Bmp180CalibrationData *calibrationData);


// Structure for circular buffer
typedef struct {
    uint16_t buff[MAX_NO_TEMP];
    uint8_t start;
    uint8_t end;
    uint8_t count;
} cbuff_t;

// Prototypes for BMP180 functions and circular buffer
void BMP180_ReadCalibrationData(Bmp180CalibrationData *data);
int16_t BMP180_ReadTemperature();
float BMP180_CalculateTemperature(uint16_t uncompensatedTemperature, Bmp180CalibrationData *calibrationData);

void cbuff_new(cbuff_t *cb, uint16_t size);
void cbuff_add(cbuff_t *cb, uint16_t elem);
uint16_t avg_temperature(cbuff_t *cb);
void SendCalibrationData(Bmp180CalibrationData *calibrationData);


#endif /* INC_BMP180_H_ */
