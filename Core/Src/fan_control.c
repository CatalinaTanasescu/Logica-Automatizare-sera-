/*
 * fan_control.c
 *
 *  Created on: Nov 9, 2024
 *      Author: Marmelada
 */


#include "fan_control.h"
#include "main.h"

// Define the GPIO ports and pins for controlling the fan
#define FAN_IN1_PORT GPIOA
#define FAN_IN1_PIN GPIO_PIN_6
#define FAN_IN2_PORT GPIOC
#define FAN_IN2_PIN GPIO_PIN_0

// Initialize the GPIO pins for fan control
void Fan_Init(void) {
    // Enable GPIO clocks (ensure these clocks are enabled in your setup)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure FAN_IN1 (PA6) as output
    GPIO_InitStruct.Pin = FAN_IN1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAN_IN1_PORT, &GPIO_InitStruct);

    // Configure FAN_IN2 (PC0) as output
    GPIO_InitStruct.Pin = FAN_IN2_PIN;
    HAL_GPIO_Init(FAN_IN2_PORT, &GPIO_InitStruct);

    // Initially, turn off the fan
    HAL_GPIO_WritePin(FAN_IN1_PORT, FAN_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FAN_IN2_PORT, FAN_IN2_PIN, GPIO_PIN_RESET);
}

// Control the fan based on the temperature
void Fan_Control(uint8_t state) {
    if (state) {
        // Turn fan ON
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // Set IN1 high
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Set IN2 low
    } else {
        // Turn fan OFF
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // Set IN1 low
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Set IN2 low
    }
}
