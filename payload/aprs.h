#pragma once

#include <string>
#include <pico/audio.h>

#include "payload_state_manager.h"

namespace aprs
{
    inline audio_buffer_pool_t* abp = nullptr;
    inline std::string transmit_data;

    void transmitAllData(const PayloadState& state, double apogee);
    void transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data);
}
