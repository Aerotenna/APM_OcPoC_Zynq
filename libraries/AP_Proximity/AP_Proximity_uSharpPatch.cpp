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
#include <AP_HAL_Linux/CANDriver.h>
#include "AP_Proximity_uSharpPatch.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_uSharpPatch::AP_Proximity_uSharpPatch(AP_Proximity &_frontend,
                                                   AP_Proximity::Proximity_State &_state,
                                                   AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    can = new Linux::CANDriver();

    if (can != nullptr) {
        can->init();
    }

    /* select work mode here */
    can_hub_init();

    for (uint8_t i = 0; i < _num_sectors; i++) {
        _angle[i] = _sector_middle_deg[i];
        _distance[i] = PROXIMITY_USHARP_DISTANCE_MAX;
        _distance_valid[i] = true;
    }
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uSharpPatch::detect(AP_SerialManager &serial_manager)
{
    // 
    return true;
}

/* update the state of the sensor */
void AP_Proximity_uSharpPatch::update(void)
{
    if (can == nullptr) {
        return;
    }

    /* read uSharp Hub */
    if (get_reading()) {
        set_status(AP_Proximity::Proximity_Good);
    } else {
        set_status(AP_Proximity::Proximity_NoData);
    }
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uSharpPatch::distance_max() const
{
    return PROXIMITY_USHARP_DISTANCE_MAX;
}

float AP_Proximity_uSharpPatch::distance_min() const
{
    return PROXIMITY_USHARP_DISTANCE_MIN;
}

bool AP_Proximity_uSharpPatch::can_hub_init()
{
    int ret = 0;
    int count = 0;

    /* temp array for can send command frame without data part to can hub, we use identifier transfer message */
    uint8_t temp_array[USHARP_DATA_FRAME];
    uint32_t id;

    /* initialize radar matrix, set radar state to fail and radar data tp 655.35m as default */
    for (int i = 0; i < RADAR_MATRIX_ROWS; i++) {
        for (int j = 0; j < RADAR_MATRIX_COLUMNS; j++) {
            Radar_Matrix[i][j] = 0;
        }
    }

    // send series of commands to initialize the can hub

    /* command can hub initialize base peripherals */
    can->can_write(nullptr, 0, HUB_CMD_INIT, CAN_EFF_FLAG);

    /* delay 10 milliseconds for frame interval */
    hal.scheduler->delay(10);

    /* check hub led */
    can->can_write(nullptr, 0, HUB_CMD_CHECK_LED, CAN_EFF_FLAG);

    // delay 1.5 seconds while hub checks all 10 led and displays state by led
    hal.scheduler->delay(1500);

    /* command can hub return the radars' states */
    can->can_write(nullptr, 0, HUB_CMD_REQ_STATE, CAN_EFF_FLAG);

    uint32_t last_call_time, now = AP_HAL::millis();
    last_call_time = now;

    /* get radar state, wait 100ms if can hub doen't response.
     * if the program is executed normally it would be about 80ms(max).
     * stm32 poll radar states code:(refer to stm32 code for more info)
     *
     * for(i=0;i<8;i++){
     *     ...
     *     wait_radar(10*1000);    ... in microseconds.
     *     ...
     * }
     * we reserved 100 milliseconds in case transmission delay.
     */
    while (last_call_time - now < 100 && ret <= 0) {
        ret = can->can_read(temp_array, USHARP_DATA_FRAME, &id);
        last_call_time = AP_HAL::millis();
        count++;
    }

#if USHARP_DEBUG_CONSOLE
    hal.console->printf("CAN init receive: 0x%08x | ", id);

    for (uint8_t i = 0; i < (USHARP_DATA_FRAME - 1); i++) {
        hal.console->printf("0x%02x | ", temp_array[i]);
    }

    hal.console->printf("0x%02x\r\n", temp_array[USHARP_DATA_FRAME - 1]);
#endif

    /* copy every radar's state into Radar_Matrix radar state line */
    if (ret > 0 && id == RADAR_ID_STATE) {
        for (uint8_t i = 0; i < NUM_USHARP_RADARS; i++) {
            Radar_Matrix[RML_STATE][i + USHARP_CAN_ID_SIZE] = temp_array[i];
        }
#if USHARP_DEBUG_CONSOLE
        hal.console->printf("---read state succeed!using: %dms-------\r\n",last_call_time - now);
        hal.console->printf("Matrix state line: ");

        for (uint8_t i = 0; i < USHARP_DATA_FRAME - 1; i++) {
            hal.console->printf("0x%02x ", Radar_Matrix[RML_STATE][i + USHARP_CAN_ID_SIZE]);
        }
        
        hal.console->printf("0x%02x\r\n", Radar_Matrix[RML_STATE][RADAR_MATRIX_COLUMNS - 1]);
#endif

    } else {
        /* read state failed */
#if USHARP_DEBUG_CONSOLE
        hal.console->printf("---read state failed:ret = %d,count = %d-------\r\n",ret,count);
#endif
        return false;
    }

    /* command radar return every radar's data for begin update loop */
    can->can_write(nullptr, 0, HUB_CMD_REQ_DATA, CAN_EFF_FLAG);

    /* it will return 1 while ret = -1 for the return type is bool */
    if (ret == -1) {
        return false;
    }
    
    return true;
}

/* return last value measured by uSharp sensor note: reading is distance in cm */
bool AP_Proximity_uSharpPatch::get_reading(void)
{
    bool ret = false;

    /* register and initialize can receive frame */
    Custom_Can_Frame Rx_frame;

    for (uint8_t i = 0; i < _num_sectors; i++) {
        Rx_frame.data[i] = 0xff;
    }

    Rx_frame.id._int = 0;

    // get radar data
    ret = get_usharp_can_data(&Rx_frame);

    /* uSharp Patch (Beijing Version) Radar Data:
     *
     * FRAME #1
     *  Radar_Matrix[radar_index][0 - 3]     - CAN ID bytes
     *  Radar_Matrix[radar_index][4, 5]      - Distance #1
     *  Radar_Matrix[radar_index][6]         - SNR #1
     *  Radar_Matrix[radar_index][7, 8]      - Distance #2
     *  Radar_Matrix[radar_index][9]         - SNR #2
     *  Radar_Matrix[radar_index][10,11]     - Distance #3
     *
     * FRAME #2
     *  Radar_Matrix[radar_index+1][4]       - SNR #3
     *  Radar_Matrix[radar_index+1][5, 6]    - Distance #4
     *  Radar_Matrix[radar_index+1][7]       - SNR #4
     *  Radar_Matrix[radar_index+1][8, 9]    - Distance #5
     *  Radar_Matrix[radar_index+1][10]      - SNR #5
     *
     * Distance is calculated with the two bytes as follows:
     *  Dist (cm) = [byte1] * 256 + [byte2]
     *
     *  Example using Distance #1 from above:
     *  Dist1 = Radar_Matrix[radar_index][4] * 256
     *        + Radar_Matrix[radar_index][5];
     */

    // fill AP_Proximity _distance data
    if (ret == false) {
        for (uint8_t i = 0; i < _num_sectors; i++) {
            _distance[i] = PROXIMITY_USHARP_DISTANCE_MAX;
        }
    } else {
        for (uint8_t i = 0; i < _num_sectors; i++) {
            int idx = AVOID_DIST_RADAR_IDX(i);

            int raw_range = 0;

            if (idx > 0) {
                raw_range = Radar_Matrix[idx][4] * 256 + Radar_Matrix[idx][5];
            }

            // if Radar returned nothing, set to max distance
            _distance[i] = raw_range == 0 ? PROXIMITY_USHARP_DISTANCE_MAX :
                                            (float)(raw_range) / 100.0f; // convert to meters
#if RADAR_DATA_DEBUG
            if (idx == RML_RD1_F1) {
                hal.console->printf("\nuSharp Front FRAME #1: %d, %d, %d, %d, %d, %d, %d, %d\n", Radar_Matrix[idx][4], Radar_Matrix[idx][5], Radar_Matrix[idx][6],
                                    Radar_Matrix[idx][7], Radar_Matrix[idx][8], Radar_Matrix[idx][9], Radar_Matrix[idx][10], Radar_Matrix[idx][11]);
                hal.console->printf("uSharp Front FRAME #2: %d, %d, %d, %d, %d, %d, %d\n", Radar_Matrix[idx+1][4], Radar_Matrix[idx+1][5], Radar_Matrix[idx+1][6],
                                    Radar_Matrix[idx+1][7], Radar_Matrix[idx+1][8], Radar_Matrix[idx+1][9], Radar_Matrix[idx+1][10]);
            }
#endif
#if RADAR_DISTANCE_DEBUG
            if (_distance[i] < PROXIMITY_USHARP_DISTANCE_MAX) {
                hal.console->printf("\nuSharp Dist: %f idx: %d sector: %d\n", _distance[i], idx, i);
            }
#endif
        }
    }

    for (uint8_t sector = 0; sector < _num_sectors; sector++) {
        update_boundary_for_sector(sector);
    }

    /* request hub return a new package radar data */
    can->can_write(nullptr, 0, HUB_CMD_REQ_DATA, CAN_EFF_FLAG);

    return ret;
}

void AP_Proximity_uSharpPatch::copy_can_frame(uint8_t* addr, Custom_Can_Frame* buffer_frame)
{
    for (uint8_t i = 0; i < RADAR_MATRIX_COLUMNS; i++) {
        if (i < USHARP_CAN_ID_SIZE) {
            // copy id to addr, in the union the LSB was saved in the array[0] position
            addr[i] = buffer_frame->id._char[3 - i];
        } else {
            // copy data to addr
            addr[i] = buffer_frame->data[i - 4];
        }
    }
}

bool AP_Proximity_uSharpPatch::get_usharp_can_data(Custom_Can_Frame* frame)
{
    /* if received a right frame, set the recv_flag corresponding bit */
    uint16_t recv_flag = 0x0000;

    // clear data first
    for (uint8_t i = 1; i < RADAR_MATRIX_ROWS; i++) {
        for (int j = 0; j < RADAR_MATRIX_COLUMNS; j++) {
            Radar_Matrix[i][j] = 0xff;
        }
    }

    // read data by frame
    for (uint8_t i = 1; i < RADAR_MATRIX_ROWS; i++) {

        if (can->can_read(frame->data, USHARP_DATA_FRAME, &frame->id._int) <= 0) {
            break;
        }

        /* according frame's id put the data into radar matrix */
        switch (frame->id._int) {
            case ANS_FRONT_F1:
                recv_flag |= 0x0001;
                copy_can_frame(Radar_Matrix[RML_RD1_F1], frame);
                break;

            case ANS_FRONT_F2:
                recv_flag |= 0x0002;
                copy_can_frame(Radar_Matrix[RML_RD1_F2], frame);
                break;

            case ANS_RIGHT_FRONT_F1:
                recv_flag |= 0x0004;
                copy_can_frame(Radar_Matrix[RML_RD2_F1], frame);
                break;

            case ANS_RIGHT_FRONT_F2:
                recv_flag |= 0x0008;
                copy_can_frame(Radar_Matrix[RML_RD2_F2], frame);
                break;

            case ANS_RIGHT_F1:
                recv_flag |= 0x0010;
                copy_can_frame(Radar_Matrix[RML_RD3_F1], frame);
                break;

            case ANS_RIGHT_F2:
                recv_flag |= 0x0020;
                copy_can_frame(Radar_Matrix[RML_RD3_F2], frame);
                break;

            case ANS_RIGHT_BACK_F1:
                recv_flag |= 0x0040;
                copy_can_frame(Radar_Matrix[RML_RD4_F1], frame);
                break;

            case ANS_RIGHT_BACK_F2:
                recv_flag |= 0x0080;
                copy_can_frame(Radar_Matrix[RML_RD4_F2], frame);
                break;

            case ANS_BACK_F1:
                recv_flag |= 0x0100;
                copy_can_frame(Radar_Matrix[RML_RD5_F1], frame);
                break;

            case ANS_BACK_F2:
                recv_flag |= 0x0200;
                copy_can_frame(Radar_Matrix[RML_RD5_F2], frame);
                break;

            case ANS_LEFT_BACK_F1:
                recv_flag |= 0x0400;
                copy_can_frame(Radar_Matrix[RML_RD6_F1], frame);
                break;

            case ANS_LEFT_BACK_F2:
                recv_flag |= 0x0800;
                copy_can_frame(Radar_Matrix[RML_RD6_F2], frame);
                break;

            case ANS_LEFT_F1:
                recv_flag |= 0x1000;
                copy_can_frame(Radar_Matrix[RML_RD7_F1], frame);
                break;

            case ANS_LEFT_F2:
                recv_flag |= 0x2000;
                copy_can_frame(Radar_Matrix[RML_RD7_F2], frame);
                break;

            case ANS_LEFT_FRONT_F1:
                recv_flag |= 0x4000;
                copy_can_frame(Radar_Matrix[RML_RD8_F1], frame);
                break;

            case ANS_LEFT_FRONT_F2:
                recv_flag |= 0x8000;
                copy_can_frame(Radar_Matrix[RML_RD8_F2], frame);
                break;
        }
    }

    return recv_flag != 0;
}
