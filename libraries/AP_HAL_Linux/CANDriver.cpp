#include "CANDriver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <AP_HAL/AP_HAL.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

void CANDriver::init() {

    long iMode = 1;

    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter;

    /* create socket */
    _s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    /* assign can0 device */
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, "can0");
    ioctl(_s, SIOCGIFINDEX, &ifr);

    /* set socket can non-blocking mode */
    ioctl(_s, FIONBIO, &iMode);

    addr.can_ifindex = ifr.ifr_ifindex;//ifr_ifindex;

    /* bind socket with can0 */
    bind(_s, (struct sockaddr*)&addr, sizeof(addr));

    /* set receive rule */
    rfilter.can_id   = 0x155AA500|CAN_EFF_FLAG;
    rfilter.can_mask = 0x155AA500|CAN_EFF_FLAG;

    /* enable can */
    setsockopt(_s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(struct can_filter));

    return;
}

int CANDriver::can_write(uint8_t * tx_data, uint8_t len, uint32_t tx_id, uint32_t f_type)
{
    struct can_frame frame;

    frame.can_id  = tx_id | f_type;
    frame.can_dlc = len;

    for (int i=0; i<len; i++) {
        frame.data[i] = tx_data[i];
    }

    int nbytes = write(_s, &frame, sizeof(frame));

    if(nbytes != sizeof(frame)){
        return 1;
    }

    return 0;
}

int CANDriver::can_read(uint8_t * rx_data, uint8_t len, uint32_t* rx_id)
{
    int nbytes = 0;
    struct can_frame frame;

    nbytes = read(_s, &frame, sizeof(struct can_frame));

    frame.can_id &= 0x1fffffff;

    if (nbytes > 0) {
        *rx_id = frame.can_id;
        for (int i=0; i<len; i++) {
            rx_data[i] = frame.data[i];
        }
    }

    return nbytes;
}


