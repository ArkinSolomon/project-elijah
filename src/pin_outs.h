#pragma once

#include "hardware/pio.h"

#define I2C_BUS0 i2c0
#define I2C_BUS1 i2c1
#define STATUS_PIO pio0

#define I2C0_SDA_PIN 8 // Pin 11
#define I2C0_SCL_PIN 9 // Pin 12

#define I2C1_SDA_PIN 14 // Pin 19
#define I2C1_SCL_PIN 15 // Pin 20

#define TEMP_LED_PIN 0 // Pin 1
#define STATUS_LED_PIN 25 // Onboard

// #define HMC_5883L_RDY_PIN 1 // Pin 2
#define MPU_6050_INT_PIN 1 // Pin 2