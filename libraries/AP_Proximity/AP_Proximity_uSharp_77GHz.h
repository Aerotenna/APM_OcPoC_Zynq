#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_USHARP_77GHz_TIMEOUT_MS        200                               // requests timeout after 0.2 seconds
#define USHARP_77GHz_MEASUREMENT_COEFFICIENT     24.4f //4.7f
#define PROXIMITY_USHARP_77GHz_DISTANCE_MAX      50.0f
#define PROXIMITY_USHARP_77GHz_DISTANCE_MIN      0.31f

#define USHARP_77GHZ_MAX_INDEX 255

class AP_Proximity_uSharp_77GHz : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_uSharp_77GHz(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const;
    float distance_min() const;

private:

    // read data from sensor
    bool get_reading(void);

    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t linebuf[10];
    uint8_t linebuf_len;

    // convert indices of uSharp panels to AC_Avoid appropriate indices
    uint8_t _avoid_index_from_panel[PROXIMITY_USHARP_PANELS] = {0, 2, 4, 6, 1, 3, 5, 7};
};
