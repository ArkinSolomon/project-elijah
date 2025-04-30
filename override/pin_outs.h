#pragma once

// Don't place includes here as they will be included in sd_hw_config.c

#define LED_2_PIN 2
#define LED_3_PIN 3
#define ONBOARD_LED_PIN 25
#define SPEAKER_GND_PIN 26
#define SPEAKER_PIN 27

#define PTT_ENABLE 6

#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5

#define MPU_6050_ADDR 0x68

#define SPI0_SCK_PIN 18
#define SPI0_TX_PIN 19
#define SPI0_RX_PIN 16

#define SD_CS_PIN 0
// #define BMP_CS_PIN 20
#define BMP_CS_PIN 21 // Replacement

#define SPI1_SCK_PIN 10
#define SPI1_TX_PIN 11
#define SPI1_RX_PIN 12
#define SPI1_CSN_PIN 13

#define BAT_VOLTAGE_PIN 28

void pin_init();