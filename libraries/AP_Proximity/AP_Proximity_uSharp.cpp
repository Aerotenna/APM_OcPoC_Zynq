/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_uSharp.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_uSharp::AP_Proximity_uSharp(AP_Proximity &_frontend,
                                         AP_Proximity::Proximity_State &_state,
                                         AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
    }
}

// detect if a Aerotenna proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_uSharp::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

// update the state of the sensor
void AP_Proximity_uSharp::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // read uSharp
    if (get_reading()) {
        set_status(AP_Proximity::Proximity_Good);
    }else{
        set_status(AP_Proximity::Proximity_NoData);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_uSharp::distance_max() const
{
    return PROXIMITY_USHARP_DISTANCE_MAX;
}
float AP_Proximity_uSharp::distance_min() const
{
    return PROXIMITY_USHARP_DISTANCE_MIN;
}

// return last value measured by uSharp sensor
// note: reading is distance in cm
bool AP_Proximity_uSharp::get_reading(void)
{
    // read any available lines from the uLanding
    float sum[4] = {0,};
    uint16_t count[4] = {0,};
    uint8_t  index = 0;
    uint8_t  panel_index;

    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if ( c == 72 && index ==0 ) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if ( index == 1 ){
            linebuf[linebuf_len] = c;
            linebuf_len ++;
            if ( linebuf_len == 4 ){
                index = 0;

                switch (linebuf[1]) {
                case 252:
                    panel_index = 3;
                    break;
                case 253:
                    panel_index = 0;
                    break;
                case 254:
                    panel_index = 1;
                    break;
                default:
                    panel_index = 2;
                }

                sum[panel_index] += (linebuf[3] & 0x7F)*128 + (linebuf[2] & 0x7F);
                count[panel_index]++;

                linebuf_len = 0;
            }
        }

    }

    if ( count[0] == 0 && count[1] == 0 && count[2] == 0 && count[3] == 0 ) {
        // return false (i.e. set status = Proximity_NoData) if no panels return data
        return false;
    }

    for (uint8_t i=0; i<_num_sectors; i++) {
        if ( count[i] != 0 ){
            _distance[i] = (USHARP_MEASUREMENT_COEFFICIENT * sum[i] / count[i]) / 100.0f;
            _distance_valid[i] = true;
            _last_distance_received_ms = AP_HAL::millis();
        }else{
            _distance_valid[i] = false;
        }
    }

    // update boundary used for avoidance
    for (uint8_t sector=0; sector<_num_sectors; sector++) {
        update_boundary_for_sector(sector);
    }

    return true;
}
