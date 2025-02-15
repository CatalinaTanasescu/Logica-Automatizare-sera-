#include "bmp180.h"
#include "main.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;  // External I2C handle
extern UART_HandleTypeDef huart1;

// BMP180 Functions

// Write to BMP180 register
void BMP180_WriteReg(uint8_t reg, uint8_t cmd) {
    uint8_t data[2] = { reg, cmd };
    HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, data, 2, 1000);
}

// Read a single byte from a BMP180 register
uint8_t BMP180_ReadReg(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, &reg, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, BMP180_READ, &value, 1, 1000);
    return value;
}

void SendCalibrationData(Bmp180CalibrationData *calibrationData) {
    char buffer[50];

    // Send each calibration value
    snprintf(buffer, sizeof(buffer), "Calibration Data:\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac1 = %d\r\n", calibrationData->ac1);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac2 = %d\r\n", calibrationData->ac2);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac3 = %d\r\n", calibrationData->ac3);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac4 = %u\r\n", calibrationData->ac4);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac5 = %u\r\n", calibrationData->ac5);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "ac6 = %u\r\n", calibrationData->ac6);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "b1 = %d\r\n", calibrationData->b1);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "b2 = %d\r\n", calibrationData->b2);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "mb = %d\r\n", calibrationData->mb);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "mc = %d\r\n", calibrationData->mc);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "md = %d\r\n", calibrationData->md);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
// Read calibration data from BMP180 E2PROM
void BMP180_ReadCalibrationData(Bmp180CalibrationData *data) {
    data->ac1 = (BMP180_ReadReg(0xAA) << 8) | BMP180_ReadReg(0xAB);
    data->ac2 = (BMP180_ReadReg(0xAC) << 8) | BMP180_ReadReg(0xAD);
    data->ac3 = (BMP180_ReadReg(0xAE) << 8) | BMP180_ReadReg(0xAF);
    data->ac4 = (BMP180_ReadReg(0xB0) << 8) | BMP180_ReadReg(0xB1);
    data->ac5 = (BMP180_ReadReg(0xB2) << 8) | BMP180_ReadReg(0xB3);
    data->ac6 = (BMP180_ReadReg(0xB4) << 8) | BMP180_ReadReg(0xB5);
    data->b1  = (BMP180_ReadReg(0xB6) << 8) | BMP180_ReadReg(0xB7);
    data->b2  = (BMP180_ReadReg(0xB8) << 8) | BMP180_ReadReg(0xB9);
    data->mb  = (BMP180_ReadReg(0xBa) << 8) | BMP180_ReadReg(0xBB);
    data->mc  = (BMP180_ReadReg(0xBC) << 8) | BMP180_ReadReg(0xBD);
    data->md  = (BMP180_ReadReg(0xBE) << 8) | BMP180_ReadReg(0xBF);
}

// Read raw temperature data from BMP180
int16_t BMP180_ReadTemperature() {
    BMP180_WriteReg(CONTROL_REGISTER_VALUE, 0x2E);  // Start temperature measurement
    HAL_Delay(5);  // Wait for measurement to complete
    return (BMP180_ReadReg(0xF6) << 8) | BMP180_ReadReg(0xF7);
}

// Calculate actual temperature in Celsius using calibration data
float BMP180_CalculateTemperature(uint16_t uncompensatedTemperature, Bmp180CalibrationData *calibrationData) {
    // Step 1: Calculate X1
    int32_t x1 = ((uncompensatedTemperature - calibrationData->ac6) * calibrationData->ac5) >> 15;

    // Step 2: Calculate X2
    int32_t x2 = (calibrationData->mc << 11) / (x1 + calibrationData->md);

    // Step 3: Calculate B5
    int32_t b5 = x1 + x2;

    // Step 4: Calculate actual temperature in Celsius
    float temperature = ((b5 + 8) >> 4) / 10.0;

    return temperature;  // Temperature in °C
}


// Function to start and read the uncompensated pressure (UP)
int32_t BMP180_ReadUncompensatedPressure(uint8_t oss) {
    // Start pressure measurement with the desired oversampling setting (OSS)
    BMP180_WriteReg(CONTROL_REGISTER_VALUE, 0x34 + (oss << 6));

    // Delay based on oversampling setting
    switch (oss) {
        case 0: HAL_Delay(5); break;  // Low power: 4.5 ms
        case 1: HAL_Delay(8); break;  // Standard: 7.5 ms
        case 2: HAL_Delay(14); break; // High resolution: 13.5 ms
        case 3: HAL_Delay(26); break; // Ultra-high resolution: 25.5 ms
    }

    // Read the raw pressure data from registers 0xF6 (MSB), 0xF7, and 0xF8 (XLSB)
    int32_t up = ((int32_t)BMP180_ReadReg(0xF6) << 16 | (int32_t)BMP180_ReadReg(0xF7) << 8 | (int32_t)BMP180_ReadReg(0xF8)) >> (8 - oss);
    return up;
}

// Function to calculate the actual pressure in Pascals using calibration data
int32_t BMP180_CalculatePressure(int32_t up, int32_t b5, uint8_t oss, Bmp180CalibrationData *calibrationData) {
    int32_t b6 = b5 - 4000;
    int32_t x1 = (calibrationData->b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (calibrationData->ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((calibrationData->ac1 * 4 + x3) << oss) + 2) >> 2;

    x1 = (calibrationData->ac3 * b6) >> 13;
    x2 = (calibrationData->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (calibrationData->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (50000 >> oss);

    int32_t p;
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    return p;  // Pressure in Pa
}

// Circular Buffer Functions

// Initialize circular buffer
void cbuff_new(cbuff_t *cb, uint16_t size) {
    cb->start = 0;
    cb->end = 0;
    cb->count = 0;
}

// Add new element to circular buffer
void cbuff_add(cbuff_t *cb, uint16_t elem) {
    if (cb->count == MAX_NO_TEMP) {
        cb->start = (cb->start + 1) % MAX_NO_TEMP;
    } else {
        cb->count++;
    }
    cb->buff[cb->end] = elem;
    cb->end = (cb->end + 1) % MAX_NO_TEMP;
}

// Calculate average temperature from buffer
uint16_t avg_temperature(cbuff_t *cb) {
    uint32_t sum = 0;
    for (int i = 0, idx = cb->start; i < cb->count; i++) {
        sum += cb->buff[idx];
        idx = (idx + 1) % MAX_NO_TEMP;
    }
    return cb->count ? (sum / cb->count) : 0;
}
