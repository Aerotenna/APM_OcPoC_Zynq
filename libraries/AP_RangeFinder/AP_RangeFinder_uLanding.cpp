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
#include "AP_RangeFinder_uLanding.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_uLanding::AP_RangeFinder_uLanding(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state,
                                                             AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0));
    }
}

/*
   detect if a uLanding rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_uLanding::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0) != nullptr;
}

// update ulanding filter parameters
void AP_RangeFinder_uLanding::set_ulanding_params(int filter_length)
{
    // set to default if the input parameter is less than 1 (protect against
    // divide by zero)
    _max_sum_count = filter_length < 1 ? DEFAULT_FILT_BUFFER : filter_length;

    // set to max filter length if the parameter exceeds defined max
    _max_sum_count = filter_length > MAX_FILT_BUFFER ? MAX_FILT_BUFFER : filter_length;
}


// read - return last value measured by sensor
bool AP_RangeFinder_uLanding::get_reading(uint16_t &reading_cm, uint16_t &voltage_mv)
{
    if (uart == nullptr) {
        return false;
    }

/***************************************************************************************
*
*  This is following the 'sampled Gaussian kernal' at
*  https://en.wikipedia.org/wiki/Scale_space_implementation#The_discrete_Gaussian_kernel
*  using an implementation similar to the python
*  gaussian_filter1d() method
*
***************************************************************************************/
    // variables for the filter, these are the tuning paramters to set in
    // the GCS.
    // standard deviation of the gaussian
    float sigma = _filter_sigma; 
    //# of standard deviations to use in a single window
    float truncate = _filter_truncate;

    //these are local variables for the filter
    //this is half the window length
    int wl = int(truncate * sigma + 0.5);
    //weights[] is the kernel matrix, or array in this case
    int kernel_length = 2 * wl + 1;
    //float *weights = new float[kernel_length];
    float weights[kernel_length];
    weights[wl] = 1.0; //value will vary based on filter function shape
    float kernelsum = weights[wl];
    float t = sigma * sigma;


    //make the kernel
    for (int i = 0; i < wl + 1; i++){
        float temp = exp(-0.5 * i * i / t);
        weights[wl+i] = temp;
        weights[wl-i] = temp;
        kernelsum += 2.0 * temp;
	//TODO: this sum is wrong, needs to add once for i=0
	//hal.console->printf(
    }

    //normalize the kernel
    for ( int j = 0; j < kernel_length; j++ ) {
        weights[j] /= kernelsum;
	//hal.console->printf(" %i -- %2.5f", j, weights[j]);
    }
    //hal.console->printf("\n");

    //array to store data for filtering
    //uint8_t *input_data = new uint8_t[2*wl+40];
    float input_data[2*wl+40];
    float output_data[40];

    // read any available lines from the uLanding
    float sum = 0;
    uint16_t count = 0;
    uint8_t  index = 0;

#if ULANDING_VERSION == 1
    uint8_t ulanding_hdr = 254;
#else
    uint8_t ulanding_hdr = 72;
#endif

    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if (c == ulanding_hdr && index == 0) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if (index == 1) {
            linebuf[linebuf_len] = c;
            linebuf_len ++;

#if ULANDING_VERSION == 1
            if (linebuf_len == 6) {
            // we have received six bytes data 
            // checksum
                if (((linebuf[1] + linebuf[2] + linebuf[3] + linebuf[4]) & 0xFF) == linebuf[5]) {
                    //do the sum later now, after filtering
                    sum += linebuf[3]*256 + linebuf[2];
/*                    //sum += linebuf[3]*256 + linebuf[2];
                    if (count < 40) {
                        if (count == 0) { // || count == 39){//pads the front and back so that we can still use all the data
                            for (int i=0; i<wl; i++) {
                               input_data[i+count] = linebuf[3]*256 + linebuf[2];
                            }
                        } else if (count == 39)
                        
                            input_data[count+wl] = linebuf[3]*256 + linebuf[2];
                        
                    }
                    // collect raw data for plotting
                    raw_sum += linebuf[3]*256 + linebuf[2];
*/
                    count ++;
                }
                index = 0;
                linebuf_len = 0;
            }
#else
            if (linebuf_len == 3) {
                index = 0;
                sum += (linebuf[2]&0x7F) *128 + (linebuf[1]&0x7F);
                linebuf_len = 0;
                count ++;
            }
#endif
        }
    }
    for( int m = 0; m < 2*wl+1 ; m++) {
        hal.console->printf("%i %2.5f", m, input_data[m]);
    }
/*
    for(int i=0;i<15;i++){
        hal.console->printf("%i",input_data[i]);
    }
*/
    //apply the kernal
    for(int j=0;j<40;j++){
        for(int k=0;k<kernel_length;k++){
            output_data[j] += weights[k] * input_data[j+k];
	//hal.console->printf("%i %i %2.5f %2.5f %2.5f\n",j,k, output_data[j],weights[k],input_data[j+k]);
        }
	//hal.console->printf("summed output %i: %2.5f\n\n",j,output_data[j]);
        sum+=output_data[j];
    }

    if (count == 0) {
        return false;
    }

#if ULANDING_VERSION == 1
    //reading_cm = sum / count;
    _raw_uLanding  = sum / count;
    voltage_mv = _raw_uLanding;

    // apply filter
    _running_avg[_running_count] = _raw_uLanding;

    float running_sum = 0.0f;
    for (uint8_t i=0; i<_max_sum_count; i++) {
        running_sum += _running_avg[i];
    }
    reading_cm = running_sum / _max_sum_count;

    // cleanup running count
    if (++_running_count > _max_sum_count) {
        _running_count = 0;
    }

#if ULAND_FILT_DEBUG
    if (debug_print_count++ > 2) {
        hal.console->printf("raw: %0.2f  filt: %0.2f  count: %d\n", (float)(_raw_uLanding * 0.01f), (float)(reading_cm * 0.01f), count);
        debug_print_count = 0;
    }
#endif
#else
    reading_cm = 2.5f * sum / count;
#endif

    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_uLanding::update(void)
{
    if (get_reading(state.distance_cm, state.voltage_mv)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
