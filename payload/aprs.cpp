#include "aprs.h"
#include "aprs_pico.h"
#include <hardware/clocks.h>

#include <cmath>
#include <cstring>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/structs/io_bank0.h>
#include <pico/time.h>

#include "pin_outs.h"

void aprs::transmitAllData(const PayloadState& state) {
	set_sys_clock_48mhz();
	audio_buffer_pool_t* audio_buffer_pool = aprs_pico_init();
	gpio_put(LED_3_PIN, true);

	gpio_put(LED_2_PIN, true);
	gpio_put(RADIO_PTT_PIN, false);
	aprs::transmitData(audio_buffer_pool, "  This is a test. ");
	aprs::transmitData(audio_buffer_pool, "  Battery Level: " + std::to_string(int(state.bat_percent)) + "% ");
	aprs::transmitData(audio_buffer_pool, "  Current Temperature: " + std::to_string(int(state.temperature)) + " F ");
	gpio_put(RADIO_PTT_PIN, true);
	gpio_put(LED_2_PIN, false);

}

void aprs::transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data) {
	aprs_pico_sendAPRS(audio_buffer_pool,
						"KF8CDC-11",	// Source call sign
						"KF8CDC-7",		// Destination call sign
						"WIDE1-1",		// APRS path #1
						"WIDE2-2",		// APRS path #2
						data.c_str(),	// Data message
						39.74747,		// Latitude  (in deg)
						-83.81279,		// Longitude (in deg)
						318,			// Altitude  (in m)
						'\\',			// APRS symbol table: Secondary
						'O',			// APRS symbol code:  Rocket
						128u);	// Volume    (0 ... 256)
	sleep_ms(500);

}