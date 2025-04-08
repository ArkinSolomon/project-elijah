#pragma once

#include <hardware/i2c.h>
#include <ctime>

enum class PayloadPersistentKey : uint8_t;
struct PayloadState;
class PayloadStateManager;
enum class PayloadFaultKey : uint8_t;

class DS1307
{
public:
    DS1307(i2c_inst_t* i2c_inst, uint8_t i2c_addr);

    bool check_clock(bool& clock_set) const;
    bool set_clock(const tm& time_inst) const;
    bool functional_check(const tm& reset_inst) const;

    bool read_clock(tm &time_inst) const;
    bool check_and_read_clock(tm& time_inst) const;

    bool read_custom_register(uint8_t addr, uint8_t* output, uint8_t size);
    bool write_custom_register(uint8_t addr, const uint8_t* data, uint8_t size);
    void reg_dump() const;
    void erase_data() const;

private:
    i2c_inst_t* i2c_inst;
    uint8_t i2c_addr;

    static constexpr uint8_t REG_SECONDS = 0x00;
    static constexpr uint8_t REG_MINUTES = 0x01;
    static constexpr uint8_t REG_HOURS = 0x02;
    static constexpr uint8_t REG_DAY = 0x03;
    static constexpr uint8_t REG_DATE = 0x04;
    static constexpr uint8_t REG_MONTH = 0x05;
    static constexpr uint8_t REG_YEAR = 0x06;
    static constexpr uint8_t REG_CTRL = 0x07;
};
