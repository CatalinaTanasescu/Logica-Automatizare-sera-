#include "soil_moisture.h"
#include "stm32h5xx_hal.h"
#include <stdio.h>
#include <string.h>

// External variables
extern ADC_HandleTypeDef hadc1;       // ADC handle
extern UART_HandleTypeDef huart2;    // UART handle

// Calibration values
#define NUM_SAMPLES 20               // Number of samples for averaging
#define CALIBRATION_DRY 3900         // ADC value for completely dry soil
#define CALIBRATION_WET 1800         // ADC value for completely wet soil
#define THRESHOLD_MODERATE 3000      // ADC value for moderate soil moisture

/**
 * @brief Initializes the GPIOs and ADC for soil moisture sensor
 */
void SoilMoisture_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure PC3 (ADC1_INP13) as the active ADC channel
    //sConfig.Channel = ADC_CHANNEL_13;
    //sConfig.Rank = ADC_REGULAR_RANK_1;
    //sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler(); // Ensure proper ADC configuration
    }
}

/**
 * @brief Reads and averages the analog soil moisture data from PC3
 * @retval uint32_t Averaged ADC raw value
 */
uint32_t SoilMoisture_ReadAnalog(void) {
    uint32_t total_value = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);  // Start ADC conversion
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            total_value += HAL_ADC_GetValue(&hadc1);  // Get ADC value
        }
        HAL_ADC_Stop(&hadc1);  // Stop ADC conversion
        HAL_Delay(5);  // Small delay for stabilization
    }

    return total_value / NUM_SAMPLES;  // Return averaged ADC value
}

/**
 * @brief Maps the ADC value to a percentage (0% for wet, 100% for dry)
 * @param analogValue Raw ADC value from the sensor
 * @retval int Moisture percentage (0% to 100%)
 */
int CalculateMoisturePercentage(uint32_t analogValue) {
    if (analogValue <= CALIBRATION_WET) {
        return 0;  // Completely wet
    } else if (analogValue >= CALIBRATION_DRY) {
        return 100;  // Completely dry
    } else {
        // Linearly map the ADC value to the percentage
        return 100 * (analogValue - CALIBRATION_WET) / (CALIBRATION_DRY - CALIBRATION_WET);
    }
}

/**
 * @brief Displays soil moisture data over UART and handles LED indicators
 */
void SoilMoisture_DisplayData(void) {
    uint32_t analogValue = SoilMoisture_ReadAnalog();  // Get averaged ADC value

    // Transmit raw soil moisture value over UART
    char uartBuffer[100];
    snprintf(uartBuffer, sizeof(uartBuffer), "Analog Soil Moisture: %lu\r\n", analogValue);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

    // Calculate the moisture percentage
    int moisturePercentage = CalculateMoisturePercentage(analogValue);
    snprintf(uartBuffer, sizeof(uartBuffer), "Moisture Percentage: %d%%\r\n", moisturePercentage);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

    // Update LEDs based on moisture percentage
    if (moisturePercentage > 70) {  // Dry condition
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // Blue LED OFF
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);  // Red LED OFF
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);   // Green LED ON
        snprintf(uartBuffer, sizeof(uartBuffer), "Condition: Dry\r\n");
    } else if (moisturePercentage > 30) {  // Moderate condition
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // Blue LED OFF
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);    // Red LED ON
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); // Green LED OFF
        snprintf(uartBuffer, sizeof(uartBuffer), "Condition: Moderate\r\n");
    } else {  // Wet condition
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // Blue LED ON
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);  // Red LED OFF
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); // Green LED OFF
        snprintf(uartBuffer, sizeof(uartBuffer), "Condition: Wet\r\n");
    }

    // Transmit the condition over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
}
