#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include <AP_ADC/AP_ADC.h>
#include "AP_BattMonitor_ADS1115.h"
#include "AP_BattMonitor_Analog.h"

#define BATTMON_ADS1115_DEBUG 0

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_ADS1115::AP_BattMonitor_ADS1115(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state): AP_BattMonitor_Backend(mon, mon_state)
{    
    _state.healthy = true;
    _adc.init(ADS1115_I2C_BUS);
}

// read - read the voltage and current
void
AP_BattMonitor_ADS1115::read()
{
	// Just read through A0 and A1, and stop
	// Current goes on A0 - 2
	// Voltage goes on A1 - 3
	adc_report_s report[4];
	_adc.read(report, 4);

    // get voltage and converts from ADC output to volts, adds offset to correct for actual voltage
    _state.voltage = (report[3].data /AP_BATT_VOLTDIVIDER_DEFAULT) /10.0f;

	// calculate time since last current read
	uint32_t tnow = AP_HAL::micros();
	float dt = tnow - _state.last_time_micros;
    // get amperage - converts from weird 0.1 to correct ~0.004
    _state.current_amps = report[2].data * (AP_BATT_CURR_AMP_PERVOLT_DEFAULT / 1000.0f);

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
        // .0002778 is 1/3600 (conversion to hours), 1/1000 converts s to us
        // and amps to mamps
        // Change to division since compiled out and easier to read.
        _state.current_total_mah += _state.current_amps * dt * (1.0/(3600.0*1000.0));
    }

    // record time
    _state.last_time_micros = tnow;
    
#if BATTMON_ADS1115_DEBUG
    // Debug log
    static FILE *battlog;
    if (battlog == nullptr) {
        battlog = fopen("/tmp/battlog.log", "w");
    }
    if (battlog) {
        fprintf(battlog, "%f %f\n", _state.voltage, _state.current_amps);
    }
#endif
}
