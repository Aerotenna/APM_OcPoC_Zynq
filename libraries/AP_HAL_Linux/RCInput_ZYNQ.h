#pragma once

/*
  This class implements RCInput on the ZYNQ / ZyboPilot platform with custom
  logic doing the edge detection of the PPM sum input
 */

#include "AP_HAL_Linux.h"

// FIXME A puppie dies when you hard code an address
#define RCIN_ZYNQ_PULSE_INPUT_BASE  0xFF200000

#define CUSTOM_PWM_0_BASE1 0x00050000

class Linux::RCInput_ZYNQ : public Linux::RCInput
{
public:
    void init();
    void _timer_tick(void);

 private:
    static const int TICK_PER_US=50;
    static const int TICK_PER_S=50000000;

    // Memory mapped keyhole register to pulse input FIFO
    void *pulse_input;

    // time spent in the low state
    uint32_t _s0_time;
};
