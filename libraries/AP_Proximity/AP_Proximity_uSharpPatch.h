#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_USHARP_DISTANCE_MAX      50.0f
#define PROXIMITY_USHARP_DISTANCE_MIN      0.5f

#define NUM_USHARP_RADARS   8
#define NUM_RADAR_FRAMES    2
#define USHARP_CAN_ID_SIZE  4
#define USHARP_DATA_FRAME   8
#define RADAR_MATRIX_ROWS       (NUM_USHARP_RADARS * NUM_RADAR_FRAMES + 1)
#define RADAR_MATRIX_COLUMNS    (USHARP_CAN_ID_SIZE + USHARP_DATA_FRAME)

// find proper radar index for AC_Avoid "_distance[i]"
#define AVOID_DIST_RADAR_IDX(idx) (idx * NUM_RADAR_FRAMES + 1)

/* debug info enable bits */
#define USHARP_DEBUG_CONSOLE    0

/* Set RADAR_DATA_DEBUG to the radar matrix index of the radar
 * you're interested in to print all bytes of raw data.
 * Example: #define RADAR_DATA_DEBUG RML_RD1_F1
 */
#define RADAR_DATA_DEBUG        0

/* Set RADAR_DISTANCE_DEBUG to 1 to have get_reading() print
 * interpeted _distance[] data from all radars reporting
 * a raw range greater than 0.
 */
#define RADAR_DISTANCE_DEBUG    0


/* can hub command */
#define HUB_CMD_INIT        0x0155AAAA
#define HUB_CMD_REQ_ALL     0x0155AABB
#define HUB_CMD_CHECK_LED   0x0155AACC

/* can hub mode selection */
#define HUB_MODE_ROUTER     0x0155AA00
#define HUB_MODE_1_RD       0x0155AA01
#define HUB_MODE_2_RD       0x0155AA02
#define HUB_MODE_3_RD       0x0155AA04
#define HUB_MODE_4_RD       0x0155AA08
#define HUB_MODE_5_RD       0x0155AA10
#define HUB_MODE_6_RD       0x0155AA20
#define HUB_MODE_7_RD       0x0155AA40
#define HUB_MODE_8_RD       0x0155AA80

/* can hub request data frame identifier, do not request the state and data at same time */
#define HUB_CMD_REQ_STATE   0x0155AADD
#define HUB_CMD_REQ_DATA    0x0155BB01

/* can hub radar reply frame identifier, Note: "F1" and "F2" are two frames from the same radar */
#define RADAR_ID_1_F1   0x155aa500
#define RADAR_ID_1_F2   0x155aa501        /* front */

#define RADAR_ID_2_F1   0x155aa580
#define RADAR_ID_2_F2   0x155aa581        /* right front */

#define RADAR_ID_3_F1   0x155aa5c0
#define RADAR_ID_3_F2   0x155aa5c1        /* right */

#define RADAR_ID_4_F1   0x155aa5e0
#define RADAR_ID_4_F2   0x155aa5e1        /* right back */

#define RADAR_ID_5_F1   0x155aa5f0
#define RADAR_ID_5_F2   0x155aa5f1        /* back */

#define RADAR_ID_6_F1   0x155aa5f8
#define RADAR_ID_6_F2   0x155aa5f9        /* left back */

#define RADAR_ID_7_F1   0x155aa5fc
#define RADAR_ID_7_F2   0x155aa5fd        /* left */

#define RADAR_ID_8_F1   0x155aa5fe
#define RADAR_ID_8_F2   0x155aa5ff        /* left front */

#define RADAR_ID_STATE  0x155AA555        /* hub return radars' state by this id */

// Assign Radar Numbers to Body-Axis Directions
#define    ANS_FRONT_F1         RADAR_ID_1_F1
#define    ANS_FRONT_F2         RADAR_ID_1_F2
#define    ANS_RIGHT_FRONT_F1   RADAR_ID_2_F1
#define    ANS_RIGHT_FRONT_F2   RADAR_ID_2_F2
#define    ANS_RIGHT_F1         RADAR_ID_3_F1
#define    ANS_RIGHT_F2         RADAR_ID_3_F2
#define    ANS_RIGHT_BACK_F1    RADAR_ID_4_F1
#define    ANS_RIGHT_BACK_F2    RADAR_ID_4_F2
#define    ANS_BACK_F1          RADAR_ID_5_F1
#define    ANS_BACK_F2          RADAR_ID_5_F2
#define    ANS_LEFT_BACK_F1     RADAR_ID_6_F1
#define    ANS_LEFT_BACK_F2     RADAR_ID_6_F2
#define    ANS_LEFT_F1          RADAR_ID_7_F1
#define    ANS_LEFT_F2          RADAR_ID_7_F2
#define    ANS_LEFT_FRONT_F1    RADAR_ID_8_F1
#define    ANS_LEFT_FRONT_F2    RADAR_ID_8_F2

/* define radar matrix line as RML, use these macro represent the meaning of each line in the radar matrix */
#define RML_STATE   0
#define RML_RD1_F1  1
#define RML_RD1_F2  2
#define RML_RD2_F1  3
#define RML_RD2_F2  4
#define RML_RD3_F1  5
#define RML_RD3_F2  6
#define RML_RD4_F1  7
#define RML_RD4_F2  8
#define RML_RD5_F1  9
#define RML_RD5_F2  10
#define RML_RD6_F1  11
#define RML_RD6_F2  12
#define RML_RD7_F1  13
#define RML_RD7_F2  14
#define RML_RD8_F1  15
#define RML_RD8_F2  16

class AP_Proximity_uSharpPatch : public AP_Proximity_Backend
{

public:
    /* constructor */
    AP_Proximity_uSharpPatch(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    /* static detection function */
    static bool detect(AP_SerialManager &serial_manager);

    /* update state */
    void update(void);

    /* get maximum and minimum distances (in meters) of sensor */
    float distance_max() const;
    float distance_min() const;

private:
    /* custom can receive frame */
    typedef struct {
        uint8_t data[USHARP_DATA_FRAME];

        union {
            uint8_t _char[USHARP_CAN_ID_SIZE];
            uint32_t _int;
        } id;
    } Custom_Can_Frame;

    /* Matrix to hold Radar Data
     *  - Number of rows dependent on number of radars and frames per radar,
     *    plus an initial row for status
     *  - Number of columns dependent on number of bytes in the CAN ID,
     *    plus number of data bytes per frame
     */
    unsigned char Radar_Matrix[RADAR_MATRIX_ROWS][RADAR_MATRIX_COLUMNS];

    /* initialize the can hub and select work mode */
    bool can_hub_init();

    /* read data from sensor */
    bool get_reading(void);

    /* Receive uSharp CAN Data */
    bool get_usharp_can_data(Custom_Can_Frame* frame);

    /* copy can frame from receive buffer to matrix */
    void copy_can_frame(uint8_t* addr, Custom_Can_Frame* buffer_frame);

    Linux::CANDriver *can = nullptr;
};
