#pragma once

#include "hardware/pio.h"

#define I2C_BUS i2c0
#define STATUS_PIO pio0

#define I2C_SDA_PIN 8 // Pin 11
#define I2C_SCL_PIN 9 // Pin 12

#define TEMP_LED_PIN 0 // Pin 1
#define STATUS_LED_PIN 25 // Onboard

#define HMC_5883L_RDY_PIN 1 // Pin 2