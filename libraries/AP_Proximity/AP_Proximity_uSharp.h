#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_USHARP_PANELS            4                              // maximum number of sectors
#define PROXIMITY_USHARP_SECTOR_WIDTH_DEG  (360/PROXIMITY_USHARP_PANELS)  // angular width of each sector
#define PROXIMITY_USHARP_TIMEOUT_MS        200                            // requests timeout after 0.2 seconds
#define USHARP_MEASUREMENT_COEFFICIENT     2.5f

class AP_Proximity_uSharp : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_uSharp(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // get distance in meters in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance
    bool get_horizontal_distance(float angle_deg, float &distance) const;

    // update state
    void update(void);

private:

    // read data from sensor
    bool get_reading(void);
    bool convert_angle_to_sector(float angle_degrees, uint8_t &sector) const;

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    uint8_t linebuf[10];
    uint8_t linebuf_len;

    // sensor data
    uint8_t  _num_sectors = PROXIMITY_USHARP_PANELS;      // number of sectors we will search
    uint16_t _sector_middle_deg[PROXIMITY_USHARP_PANELS]; // middle angle of each sector
    uint8_t  _sector_width_deg[PROXIMITY_USHARP_PANELS];  // width (in degrees) of each sector
    float    _angle[PROXIMITY_USHARP_PANELS];             // angle to closest object within each sector
    float    _distance[PROXIMITY_USHARP_PANELS];          // distance to closest object within each sector
    bool     _distance_valid[PROXIMITY_USHARP_PANELS];    // true if a valid distance received for each sector
};
