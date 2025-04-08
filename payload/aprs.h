#pragma once

#include <string>
#include <pico/audio.h>

#include "payload_state_manager.h"

namespace aprs
{
    inline audio_buffer_pool_t* abp = nullptr;

    void transmitAllData(PayloadState state, int apogee, tm tmLand);
    void transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data);
}
