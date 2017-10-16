#pragma once

#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <cstdint>
#include <vector>
#include <tuple>


// This is an Battery Monitor which uses the ADC_ADS1115 driver
class AP_BattMonitor_ADS1115 : public AP_BattMonitor_Backend
{
public:

    /// Constructor
	AP_BattMonitor_ADS1115(AP_BattMonitor &mon,AP_BattMonitor::BattMonitor_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
	void read(void) override;

protected:

     // read word from register
     // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;
    AP_ADC_ADS1115 _adc;

    // reads the serial number if it's not already known
    // returns true if the read was successful, or the number was already known

	uint16_t _low_threshold;
	uint16_t _hi_threshold;
};
