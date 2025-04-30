#pragma once

#include <string>
#include <pico/audio.h>

#include "payload_state_manager.h"

namespace aprs
{
    inline audio_buffer_pool_t* abp = nullptr;
    inline std::string transmit_str;

    void transmit_all_data(const PayloadState& state, double apogee);
    void transmit_data(audio_buffer_pool_t* audio_buffer_pool, const std::string& data);
    std::string get_transmit_orientation(const PayloadState& state);
}
