// -----------------------------------------------------------------------------
// File:    uav_protocol.h
// Authors: Garrett Smith, Kevin Macksamie
// Created: 09-16-2010
//
// Definitions for communication protocol between uav and remote computer.
// -----------------------------------------------------------------------------

#ifndef _UAV_PROTOCOL__H_
#define _UAV_PROTOCOL__H_

#define IDENT_MAGIC     0x09291988  // identification number
#define IDENT_VERSION   0x00000009  // uav_control protocal version
#define PKT_BUFF_LEN    2048        // max possible packet length

// Packet commands from server to client

#define SERVER_REQ_IDENT        0   // request client to identify itself
#define SERVER_ACK_IGNORED      1   // client request ignored (invalid state)
#define SERVER_ACK_TAKEOFF      2   // acknowledge request to take off
#define SERVER_ACK_LANDING      3   // acknowledge request to land
#define SERVER_ACK_TELEMETRY    4   // acknowledge request for telemetry (+data)
#define SERVER_ACK_MJPG_FRAME   5   // transmit a single frame of video
#define SERVER_UPDATE_CTL_MODE  6   // update (or acknowledge) control mode
#define SERVER_UPDATE_TRACKING  7   // send updated tracking state information
#define SERVER_UPDATE_CAM_DCI   8   // send updated camera device control info
#define SERVER_UPDATE_CAM_DCM   9   // send updated camera device control menu
#define SERVER_ACK_GTS          10
#define SERVER_ACK_GFS          11
#define SERVER_UPDATE_COLOR     12   // send updated tracking state information

// Packet commands from client to server

#define CLIENT_ACK_IDENT        0   // identify self to server
#define CLIENT_REQ_TAKEOFF      1   // command the helicopter to take off
#define CLIENT_REQ_LANDING      2   // command the helicopter to land
#define CLIENT_REQ_TELEMETRY    3   // state, orientation, altitude, battery
#define CLIENT_REQ_MJPG_FRAME   4   // request a single frame of video
#define CLIENT_REQ_SET_CTL_MODE 5   // request a change in control mode
#define CLIENT_REQ_FLIGHT_CTL   6   // command the helicopter's flight
#define CLIENT_REQ_CAM_TC       7   // request change in camera track color
#define CLIENT_REQ_CAM_DCI      8   // request camera device control info
#define CLIENT_REQ_CAM_DCC      9   // request camera device control config
#define CLIENT_REQ_CAM_COLORS   10  // request camera Color Tracking info
#define CLIENT_REQ_STS          11  // request set trim settings
#define CLIENT_REQ_GTS          12  // request get trim settings
#define CLIENT_REQ_SFS          13  // request set filter samples
#define CLIENT_REQ_GFS          14  // request get filter samples
#define CLIENT_REQ_SPIDS        15  // request set PID settings

// General packet offsets

#define PKT_COMMAND         0
#define PKT_LENGTH          1
#define PKT_BASE            2
#define PKT_BASE_LENGTH     (sizeof(uint32_t) * PKT_BASE)

// Remote Client Identification packet offsets

#define PKT_RCI_MAGIC       PKT_BASE + 0
#define PKT_RCI_VERSION     PKT_BASE + 1
#define PKT_RCI_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 2))

// Vehicle Telemetry Info packet offsets

#define PKT_VTI_YAW         PKT_BASE + 0   // yaw angle in degrees
#define PKT_VTI_PITCH       PKT_BASE + 1   // pitch angle in degrees
#define PKT_VTI_ROLL        PKT_BASE + 2   // roll angle in degrees
#define PKT_VTI_RSSI        PKT_BASE + 3   // single strength in dBm
#define PKT_VTI_ALT         PKT_BASE + 4   // altitude in inches
#define PKT_VTI_BATT        PKT_BASE + 5   // battery percentage remaining
#define PKT_VTI_AUX         PKT_BASE + 6   // auxiliary pwm channel
#define PKT_VTI_CPU         PKT_BASE + 7   // cpu utilization percentage
#define PKT_VTI_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 8))

// MJPG Frame Info packet offsets

#define PKT_MJPG_WIDTH      PKT_BASE + 0   // frame width
#define PKT_MJPG_HEIGHT     PKT_BASE + 1   // frame height
#define PKT_MJPG_FPS        PKT_BASE + 2   // frame rate
#define PKT_MJPG_IMG        PKT_BASE + 3   // frame
#define PKT_MJPG_LENGTH     (sizeof(uint32_t) * (PKT_BASE + 3))

// Vehicle Control Mode packet offsets

#define VCM_TYPE_RADIO      0x00000000      // fully radio controlled
#define VCM_TYPE_AUTO       0x00000001      // fully autonomously controlled
#define VCM_TYPE_MIXED      0x00000002      // mixed auto/remote control
#define VCM_TYPE_KILL       0x00000003      // killswitch - kill all outputs
#define VCM_TYPE_LOCKOUT    0x00000004      // radio lockout (aux channel)

#define VCM_AXIS_YAW        0x1
#define VCM_AXIS_PITCH      0x2
#define VCM_AXIS_ROLL       0x4
#define VCM_AXIS_ALT        0x8
#define VCM_AXIS_ALL        0xF

#define PKT_VCM_TYPE        PKT_BASE + 0
#define PKT_VCM_AXES        PKT_BASE + 1
#define PKT_VCM_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 2))

// Mixed Controller Mode packet offsets

#define PKT_MCM_AXIS_ALT    PKT_BASE + 0
#define PKT_MCM_AXIS_PITCH  PKT_BASE + 1
#define PKT_MCM_AXIS_ROLL   PKT_BASE + 2
#define PKT_MCM_AXIS_YAW    PKT_BASE + 3
#define PKT_MCM_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 4))

// Color tracking state

#define CTS_STATE_SEARCHING 0x00000001      // no detected object in view
#define CTS_STATE_DETECTED  0x00000002      // detected matching object

#define PKT_CTS_STATE       PKT_BASE + 0
#define PKT_CTS_X1          PKT_BASE + 1
#define PKT_CTS_Y1          PKT_BASE + 2
#define PKT_CTS_X2          PKT_BASE + 3
#define PKT_CTS_Y2          PKT_BASE + 4
#define PKT_CTS_XC          PKT_BASE + 5
#define PKT_CTS_YC          PKT_BASE + 6
#define PKT_CTS_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 7))

// Set tracking color, format, and thresholds

#define CAM_TC_FMT_RGB      0x00000001      // red/green/blue channels
#define CAM_TC_FMT_HSL      0x00000002      // hue/saturation/lightness channels

#define PKT_CAM_TC_ENABLE   PKT_BASE + 0
#define PKT_CAM_TC_FMT      PKT_BASE + 1
#define PKT_CAM_TC_CH0      PKT_BASE + 2
#define PKT_CAM_TC_CH1      PKT_BASE + 3
#define PKT_CAM_TC_CH2      PKT_BASE + 4
#define PKT_CAM_TC_TH0      PKT_BASE + 5
#define PKT_CAM_TC_TH1      PKT_BASE + 6
#define PKT_CAM_TC_TH2      PKT_BASE + 7
#define PKT_CAM_TC_FILTER   PKT_BASE + 8
#define PKT_CAM_TC_FPS      PKT_BASE + 9
#define PKT_CAM_TC_LENGTH   (sizeof(uint32_t) * (PKT_BASE + 10))

// Camera Device Control Info

#define CAM_DCI_TYPE_BOOL   0   // boolean (checkbox)
#define CAM_DCI_TYPE_INT    1   // integer (slider)
#define CAM_DCI_TYPE_MENU   2   // menu (combo box)

#define PKT_CAM_DCI_ID      PKT_BASE + 0    // V4L device control id
#define PKT_CAM_DCI_TYPE    PKT_BASE + 1    // type of device control
#define PKT_CAM_DCI_MIN     PKT_BASE + 2    // control value minimum
#define PKT_CAM_DCI_MAX     PKT_BASE + 3    // control value maximum
#define PKT_CAM_DCI_STEP    PKT_BASE + 4    // control value step quantity
#define PKT_CAM_DCI_DEFAULT PKT_BASE + 5    // control value default
#define PKT_CAM_DCI_CURRENT PKT_BASE + 6    // current value of the control
#define PKT_CAM_DCI_NAME    PKT_BASE + 7    // name of device control (32 char)
#define PKT_CAM_DCI_LENGTH  (sizeof(uint32_t) * (PKT_BASE + 16))

// Camera Device Control Menu

#define PKT_CAM_DCM_ID      PKT_BASE + 0
#define PKT_CAM_DCM_INDEX   PKT_BASE + 1
#define PKT_CAM_DCM_NAME    PKT_BASE + 2
#define PKT_CAM_DCM_LENGTH  (sizeof(uint32_t) * (PKT_BASE + 11))

// Camera Device Control Configuration

#define PKT_CAM_DCC_ID      PKT_BASE + 0    // V4L device control id
#define PKT_CAM_DCC_VALUE   PKT_BASE + 1    // new device control value
#define PKT_CAM_DCC_LENGTH  (sizeof(uint32_t) * (PKT_BASE + 2))

// Set trim settings

#define PKT_STS_AXES        PKT_BASE + 0    // axis to set trim
#define PKT_STS_VALUE       PKT_BASE + 1    // trim value (signed integer)
#define PKT_STS_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 2))

// Get trim settings

#define PKT_GTS_YAW         PKT_BASE + 0
#define PKT_GTS_PITCH       PKT_BASE + 1
#define PKT_GTS_ROLL        PKT_BASE + 2
#define PKT_GTS_ALT         PKT_BASE + 3
#define PKT_GTS_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 4))

// Set filter samples (for a single signal)

#define SFS_IMU             0
#define SFS_ALT             1
#define SFS_AUX             2
#define SFS_BATT            3

#define PKT_SFS_SIGNAL      PKT_BASE + 0
#define PKT_SFS_SAMPLES     PKT_BASE + 1   
#define PKT_SFS_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 2))

// Get filter samples (for all signals)

#define PKT_GFS_IMU         PKT_BASE + 0
#define PKT_GFS_ALT         PKT_BASE + 1
#define PKT_GFS_AUX         PKT_BASE + 2
#define PKT_GFS_BATT        PKT_BASE + 3
#define PKT_GFS_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 4))

// Set PID settings
#define SPIDS_KP            0
#define SPIDS_KI            1
#define SPIDS_KD            2

#define PKT_SPIDS_PARAM     PKT_BASE + 0
#define PKT_SPIDS_VALUE     PKT_BASE + 1
#define PKT_SPIDS_LENGTH    (sizeof(uint32_t) * (PKT_BASE + 2))

#endif // _UAV_PROTOCOL__H_

