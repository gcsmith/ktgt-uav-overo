// -----------------------------------------------------------------------------
// Definitions for communication protocol between uav and remote computer.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef UAV_PROTOCOL__H_
#define UAV_PROTOCOL__H_

#define IDENT_MAGIC     0x09291988  // identification number
#define IDENT_VERSION   0x00000001  // software version

// Packet commands from server to client

#define SERVER_REQ_IDENT        0   // request client to identify itself
#define SERVER_ACK_IGNORED      1   // client request ignored (invalid state)
#define SERVER_ACK_TAKEOFF      2   // acknowledge request to take off
#define SERVER_ACK_LANDING      3   // acknowledge request to land
#define SERVER_ACK_TELEMETRY    4   // acknowledge request for telemetry (+data)
#define SERVER_MJPG_FRAME       5   // transmit a single frame of video

// Packet commands from client to server

#define CLIENT_ACK_IDENT        0   // identify self to server
#define CLIENT_REQ_TAKEOFF      1   // command the helicopter to take off
#define CLIENT_REQ_LANDING      2   // command the helicopter to land
#define CLIENT_REQ_TELEMETRY    3   // state, orientation, altitude, battery

// General packet offsets

#define PKT_COMMAND     0

// Vehicle Telemetry Info packet offsets

#define PKT_VTI_YAW     1   // yaw angle in degrees
#define PKT_VTI_PITCH   2   // pitch angle in degrees
#define PKT_VTI_ROLL    3   // roll angle in degrees
#define PKT_VTI_RSSI    4   // single strength in dBm
#define PKT_VTI_ALT     5   // altitude in inches
#define PKT_VTI_BATT    6   // battery percentage remaining
#define PKT_VTI_SIZE    sizeof(uint32_t) * 7

#endif // UAV_PROTOCOL__H_

