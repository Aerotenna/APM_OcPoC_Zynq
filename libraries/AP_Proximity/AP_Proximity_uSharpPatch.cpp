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
#include "AP_Proximity_uSharpPatch.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_uSharpPatch::AP_Proximity_uSharpPatch(AP_Proximity &_frontend,
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
bool AP_Proximity_uSharpPatch::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

// update the state of the sensor
void AP_Proximity_uSharpPatch::update(void)
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
float AP_Proximity_uSharpPatch::distance_max() const
{
    return PROXIMITY_USHARP_DISTANCE_MAX;
}
float AP_Proximity_uSharpPatch::distance_min() const
{
    return PROXIMITY_USHARP_DISTANCE_MIN;
}

// return last value measured by uSharp sensor
// note: reading is distance in cm
bool AP_Proximity_uSharpPatch::get_reading(void)
{
    // read any available lines from the uLanding
    float sum = 0;
    float snr = 0;
    uint16_t count = 0;
    uint8_t  index = 0;

    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if ( c == 254 && index ==0 ) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if (index == 1){
            linebuf[linebuf_len] = c;
            linebuf_len ++;
            if (linebuf_len == 6){
                index = 0;
                int expo = 0;
                float fsnr = 0.0f;
                float db   = 0.0f;
                // Checksum
                // ( VersionID + Altitude LSB + Altitude MSB + SNR ) & 0xFF == checksum
                if (((linebuf[1]+linebuf[2]+linebuf[3] +linebuf[4])&0xFF)==linebuf[5]){
                    //convert snr from 1 byte "floating point" to db
                    expo = 32 - int(linebuf[4]&0x1F);
                    if ( linebuf[4] >=  224 ){
                        fsnr = 1.75f*pow(2,expo);
                    }else if (linebuf[4] >= 192) {
                        fsnr = 1.50f*pow(2,expo);
                    }else if (linebuf[4] >= 160) {
                        fsnr = 1.25f*pow(2,expo);
                    }else{
                        fsnr = 1.0f*pow(2,expo);
                    }

                    db   = 10*log10(fsnr);

                    sum += (linebuf[3] * 256) + linebuf[2];
                    snr += db;
                    count++;
                }

                // we have decoded all the bytes in the buffer
                linebuf_len = 0;
            }
        }
    }
    // If we did not receive any information from any channel, then return
    if (count == 0) {
        return false;
    }

    float snr_average = snr/count;

    for (uint8_t i=0; i<_num_sectors; i++) {

        if ((i == 0) && (snr_average > _snr_threshold)) {
            _angle[0] = _sector_middle_deg[0];
            _distance[0] = (sum / count) / 100.0f;
            _distance_valid[0] = true;
        }else{
            _angle[i] = _sector_middle_deg[i];
            _distance[i] = PROXIMITY_USHARP_DISTANCE_MAX;
            _distance_valid[i] = true;
        }
    }

    // update boundary used for avoidance
    for (uint8_t sector=0; sector<_num_sectors; sector++) {
        update_boundary_for_sector(sector);
    }

    return true;
}
