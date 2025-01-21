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

#define CORE_0_LED_PIN 21 // Pin 27
#define CORE_1_LED_PIN 20 // Pin 26
#define STATUS_LED_PIN 25 // Onboard

#define MPU_6050_INT_PIN 1 // Pin 2

#define SPI0_SCK_PIN 18 // Pin 24
#define SPI0_TX_PIN 19 // Pin 25
#define SPI0_RX_PIN 16 // Pin 21
#define SPI0_CSN_PIN 17 // Pin 22

#define SPI1_SCK_PIN 10 // Pin 14
#define SPI1_TX_PIN 11 // Pin 15
#define SPI1_RX_PIN 12 // Pin 16
#define SPI1_CSN_PIN 13 // Pin 17

#define BAT_VOLTAGE_PIN 28 // Pin 34

#if BAT_VOLTAGE_PIN == 26
#define BAT_ADC_INPUT 0
#elif BAT_VOLTAGE_PIN == 27
#define BAT_ADC_INPUT 1
#elif BAT_VOLTAGE_PIN == 28
#define BAT_ADC_INPUT 2
#endif