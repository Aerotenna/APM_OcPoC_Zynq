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

    // initialize variables dependent on number of uSharp panels
    for (uint8_t i=0; i<_num_sectors; i++) {
        // calculate middle angle of each sector based on number of panels
        _sector_middle_deg[i] = i * (M_2PI / _num_sectors);

        // set width (in deg) for each sector
        _sector_width_deg[i] = PROXIMITY_USHARP_SECTOR_WIDTH_DEG;
    }
}

// detect if a Aerotenna proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_uSharp::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

// get distance in meters in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
bool AP_Proximity_uSharp::get_horizontal_distance(float angle_deg, float &distance) const
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        if (_distance_valid[sector]) {
            distance = _distance[sector] / 100.0f; // convert from cm to meters
            return true;
        }
    }
    return false;
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

// return last value measured by uSharp sensor
// note: reading is distance in cm
bool AP_Proximity_uSharp::get_reading(void)
{
    // read any available lines from the uLanding
    float sum[4] = {0,};
    uint16_t count[4] = {0,};
    uint8_t  index = 0;

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
        		switch(linebuf[1]){
        		case 252:
        			sum[3] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[3]++;
        			break;
        		case 253:
        			sum[0] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[0]++;
        			break;
        		case 254:
        			sum[1] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[1]++;
        			break;
        		default:
        			sum[2] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[2]++;
        		}

        		linebuf_len = 0;
        	}
        }

    }

    if ( count[0] == 0 && count[1] == 0 && count[2] == 0 && count[3] == 0 ) {
        _last_distance_received_ms = 0;
        return false;
    }

    for (uint8_t i=0; i<_num_sectors; i++) {
        if ( count[i] != 0 ){
        	_distance[i] = USHARP_MEASUREMENT_COEFFICIENT * sum[i] / count[i];
            _distance_valid[i] = true;
            _last_distance_received_ms = AP_HAL::millis();
        }else{
            _distance_valid[i] = false;
        }
    }

    // check for timeout
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_USHARP_TIMEOUT_MS)) {
        return false;
    }

    return true;
}

bool AP_Proximity_uSharp::convert_angle_to_sector(float angle_degrees, uint8_t &sector) const
{
    // sanity check angle
    if (angle_degrees > 360.0f || angle_degrees < -180.0f) {
        return false;
    }

    // convert to 0 ~ 360
    if (angle_degrees < 0.0f) {
        angle_degrees += 360.0f;
    }

    bool closest_found = false;
    uint8_t closest_sector;
    float closest_angle;

    // search for which sector angle_degrees falls into
    for (uint8_t i = 0; i < _num_sectors; i++) {
        float angle_diff = fabsf(wrap_180(_sector_middle_deg[i] - angle_degrees));

        // record if closest
        if (!closest_found || angle_diff < closest_angle) {
            closest_found = true;
            closest_sector = i;
            closest_angle = angle_diff;
        }

        if (fabsf(angle_diff) <= _sector_width_deg[i] / 2.0f) {
            sector = i;
            return true;
        }
    }

    // angle_degrees might have been within a gap between sectors
    if (closest_found) {
        sector = closest_sector;
        return true;
    }

    return false;
}
