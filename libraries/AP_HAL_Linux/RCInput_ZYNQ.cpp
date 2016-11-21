#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"
#include "RCInput.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_ZYNQ::init()
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    if (mem_fd == -1) {
        AP_HAL::panic("Unable to open /dev/mem");
    }
    pulse_input = (uint32_t*) mmap(0, 0x200000, PROT_READ|PROT_WRITE,
                                                      MAP_SHARED, mem_fd, RCIN_ZYNQ_PULSE_INPUT_BASE);
    close(mem_fd);

    _s0_time = 0;
}

/*
  called at 1kHz to check for new pulse capture data from the PL pulse timer
 */
void RCInput_ZYNQ::_timer_tick()
{
    uint32_t v;

    // all F's means no samples available
    while( (v = *((uint32_t*)(pulse_input + CUSTOM_PWM_0_BASE1)) ) != 0xffffffff ) {
        // Hi bit indicates pin state, low bits denote pulse length
        if(!(v & 0x80000000))
            _s0_time = (v & 0x7fffffff)/TICK_PER_US;
        else
            _process_rc_pulse(_s0_time, (v & 0x7fffffff)/TICK_PER_US);
    }
}
