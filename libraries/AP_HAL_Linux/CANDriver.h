#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#define CAN_EFF_FLAG		(uint32_t)0x80000000
#define CAN_RTR_FLAG		(uint32_t)0x40000000
#define CAN_ERR_FLAG		(uint32_t)0x20000000

namespace Linux {

class CANDriver {
public:
	void init();

	int can_write(uint8_t * tx_data, uint8_t len, uint32_t tx_id, uint32_t f_type);
	int can_read(uint8_t * rx_data, uint8_t len, uint32_t* rx_id);

private :
	/* socket file description symbol */
	int _s;
};

}
