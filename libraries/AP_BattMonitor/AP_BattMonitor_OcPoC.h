#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"
#include <stdio.h>

class AP_BattMonitor_OcPoC :public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_BattMonitor_OcPoC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state):
        AP_BattMonitor_Backend(mon, mon_state)
    {};

    virtual ~AP_BattMonitor_OcPoC(void) {};

    // initialise
    void init();

    // read the latest battery voltage
    void read();

private:
    FILE* _xadc_fd;
    float _xadc_coef;
};
