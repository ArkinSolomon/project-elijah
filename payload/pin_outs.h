#pragma once

#define I2C0_SDA_PIN 8 // Pin 11
#define I2C0_SCL_PIN 9 // Pin 12

#define I2C1_SDA_PIN 14 // Pin 19
#define I2C1_SCL_PIN 15 // Pin 20

#define LED_2_PIN 20
#define LED_3_PIN 21
#define ONBOARD_LED_PIN 25

#define SPEAKER_GND_PIN 26
#define SPEAKER_PIN 27

#define SPI0_SCK_PIN 18 // Pin 24
#define SPI0_TX_PIN 19 // Pin 25
#define SPI0_RX_PIN 16 // Pin 21
#define SD_CARD_CSN_PIN 17 // Pin 22
#define BACKUP_BMP_280_CSN_PIN 5

#define SPI1_SCK_PIN 10 // Pin 14
#define SPI1_TX_PIN 11 // Pin 15
#define SPI1_RX_PIN 12 // Pin 16
// #define SPI1_CSN_PIN 13 // Pin 17

#define BAT_VOLTAGE_PIN 28 // Pin 34

#define RADIO_PTT_PIN 2
#define RADIO_GND_PIN 6

void pin_init();