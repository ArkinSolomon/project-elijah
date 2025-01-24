// See https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration
#include "hw_config.h"
#include "pin_outs.h"

/* Configuration of hardware SPI object */
static spi_t spi = {
    .hw_inst = spi0,  // SPI component
    .sck_gpio = SPI0_SCK_PIN,    // GPIO number (not Pico pin number)
    .mosi_gpio = SPI0_TX_PIN,
    .miso_gpio = SPI0_RX_PIN,
    .baud_rate = 125 * 1000 * 1000 / 4  // 31,250,000 Hz
}
;

/* SPI Interface */
static sd_spi_if_t spi_if = {
    .spi = &spi,  // Pointer to the SPI driving this card
    .ss_gpio = SPI0_CSN_PIN  // The SPI slave select GPIO for this SD card
};

/* Configuration of the SD Card socket object */
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if  // Pointer to the SPI interface driving this card
};

/* ********************************************************************** */

size_t sd_get_num() { return 1; }

/**
 * @brief Get a pointer to an SD card object by its number.
 *
 * @param[in] num The number of the SD card to get.
 *
 * @return A pointer to the SD card object, or @c NULL if the number is invalid.
 */
sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num) {
        return &sd_card;
    }
    return NULL;
}
