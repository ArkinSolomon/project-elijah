#pragma once

#include <string>
#include <pico/audio.h>

#include "payload_state_manager.h"

#define APRS_FRAME_FLAG 0x7E

namespace aprs
{
    void transmitAllData(PayloadState state, int apogee, tm tmLand);
    void transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data);
}
