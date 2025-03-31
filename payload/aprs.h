#pragma once

#include <string>
#include <pico/audio.h>

#include "payload_state_manager.h"

#define APRS_FRAME_FLAG 0x7E

namespace aprs
{
    void transmitAllData(const PayloadState& state);
    void transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data);
}
