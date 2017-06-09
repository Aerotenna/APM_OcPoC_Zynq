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
#include "AP_Proximity_uSharp_77GHz.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_uSharp_77GHz::AP_Proximity_uSharp_77GHz(AP_Proximity &_frontend,
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
bool AP_Proximity_uSharp_77GHz::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

// update the state of the sensor
void AP_Proximity_uSharp_77GHz::update(void)
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
float AP_Proximity_uSharp_77GHz::distance_max() const
{
    return PROXIMITY_USHARP_77GHz_DISTANCE_MAX;
}
float AP_Proximity_uSharp_77GHz::distance_min() const
{
    return PROXIMITY_USHARP_77GHz_DISTANCE_MIN;
}

// return last value measured by uSharp sensor
// note: reading is distance in cm
bool AP_Proximity_uSharp_77GHz::get_reading(void)
{
    // read any available lines from the uLanding
    uint16_t range[8];
    
    // set all range data to max index
    for (uint8_t i=0;i<8;i++) {range[i] = USHARP_77GHZ_MAX_INDEX;}
    
    uint16_t range_min = USHARP_77GHZ_MAX_INDEX;
    uint16_t peak_max  = 0;
    uint16_t  obj_cnt = 0;
    uint16_t num_detobj = 0;

    static uint8_t c = 0;
    static uint8_t c_prev = 0;
    uint8_t  index = 0;
    uint16_t offset = 0;

    int16_t nbytes = uart->available();
    //hal.console->printf("\n\nnbytes = %d;",nbytes);
    
    while (nbytes-- > 0) {

        c_prev = c;
        c = uart->read();

        //hal.console->printf("\nHello From uSharp\n");

        // ok, we have located start byte
        if ( c == 1 && c_prev == 2 && index ==0 ) {
            linebuf_len = 0;
            index       = 1;
            offset      = 1;
            //hal.console->printf("\nFound Header Byte\n");
        }
        // now it is ready to decode index information
        if (index == 1){
            //if (offset == 1) {hal.console->printf("\n");}
            //hal.console->printf("offset = %d; c = %d; ",offset,c);
            
            if ( offset == 44 ){
                num_detobj = c;
                //hal.console->printf("num_detobj = %d\n", num_detobj);
                if ( num_detobj == 0 ){
                    //index = 0;
                    break;
                }
            }
            if ( offset >= 48 ){
                //hal.console->printf("found object index");
                linebuf[linebuf_len] = c;
                linebuf_len++;
                if ( linebuf_len == 12 ){
                    //hal.console->printf("\nfound full object message; ");

                    uint16_t range_temp = linebuf[1]*256 + linebuf[0];
                    
                    uint16_t peak_temp  = linebuf[5]*256 + linebuf[4];
                    
                    //hal.console->printf("range_temp = %d\n", range_temp);

                    //if ( range_temp <= range_min ) {
                    if (peak_temp >= peak_max && range_temp >= 4) {
                        //hal.console->printf("range_temp <= range_min");range_temp < 4
                        range_min = range_temp;
                        peak_max  = peak_temp;
                    }
                    linebuf_len = 0;
                    obj_cnt++;
                    
                    if ( obj_cnt == num_detobj ){
                        range[0] = range_min;
                        //hal.console->printf("\nrange[0] = %d; ", range[0]);
                        //index = 0;
                        break;
                    }
                }
            }
            offset++;
        }
    }
 
    for (uint8_t k=0; k<_num_sectors; k++) {

        uint8_t i = _avoid_index_from_panel[k];

        if ( range[i] < USHARP_77GHZ_MAX_INDEX ) {
            _angle[i] = _sector_middle_deg[i];
            _distance[i] = (USHARP_77GHz_MEASUREMENT_COEFFICIENT * range[i] ) / 100.0f;
            //if (i==0) {hal.console->printf("_distance[0] = %f;\n", _distance[i]);}
            _distance_valid[i] = true;
        }else{
            _angle[i] = _sector_middle_deg[i];
            _distance[i] = PROXIMITY_USHARP_77GHz_DISTANCE_MAX;
            _distance_valid[i] = true;
        }
    }

    // update boundary used for avoidance
    for (uint8_t sector=0; sector<_num_sectors; sector++) {
        update_boundary_for_sector(sector);
    }

    return true;
}
