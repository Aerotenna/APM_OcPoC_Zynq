#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ

#include "AP_BattMonitor_OcPoC.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_OcPoC::init(void)
{
	_xadc_coef = 28 / 4096.0f;
}

void AP_BattMonitor_OcPoC::read(void)
{
	float vbat;
	uint32_t buff[1];

	_xadc_fd = fopen("/sys/bus/iio/devices/iio:device0/in_voltage8_raw", "r");
	if (_xadc_fd != NULL) {
		fscanf(_xadc_fd, "%d", buff);
		fclose(_xadc_fd);
		vbat = buff[0] * _xadc_coef + 0.2;
		_state.voltage = vbat;
		_state.last_time_micros = AP_HAL::micros();
		_state.healthy = true;
	} else {
		_state.healthy = false;
	}
}

#endif
