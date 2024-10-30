#pragma once

#include "hardware/pio.h"
#include "hardware/spi.h"

#define I2C_BUS0 i2c0
#define I2C_BUS1 i2c1
#define STATUS_PIO pio0

#define I2C0_SDA_PIN 8 // Pin 11
#define I2C0_SCL_PIN 9 // Pin 12

#define I2C1_SDA_PIN 14 // Pin 19
#define I2C1_SCL_PIN 15 // Pin 20

#define CORE_0_LED_PIN 27 // Pin 32
#define CORE_1_LED_PIN 26 // Pin 31
#define STATUS_LED_PIN 25 // Onboard

// #define HMC_5883L_RDY_PIN 1 // Pin 2
#define MPU_6050_INT_PIN 1 // Pin 2

#define SPI0_SCK_PIN 10 // Pin 14
#define SPI0_TX_PIN 11 // Pin 15
#define SPI0_RX_PIN 12 // Pin 16
#define SPI0_CSN_PIN 13 // Pin 17