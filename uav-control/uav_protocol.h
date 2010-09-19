// -----------------------------------------------------------------------------
// Definitions for communication protocol between uav and remote computer.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_PROTOCOL__H_
#define _UAV_PROTOCOL__H_

#define IDENT_MAGIC     0x09291988  // identification number
#define IDENT_VERSION   0x00000001  // software version

// Packet commands from server to client

#define SERVER_REQ_IDENT        0   // request client to identify itself
#define SERVER_ACK_IGNORED      1   // client request ignored (invalid state)
#define SERVER_ACK_TAKEOFF      2   // acknowledge request to take off
#define SERVER_ACK_LANDING      3   // acknowledge request to land
#define SERVER_ACK_TELEMETRY    4   // acknowledge request for telemetry (+data)
#define SERVER_ACK_MJPG_FRAME   5   // transmit a single frame of video

// Packet commands from client to server

#define CLIENT_ACK_IDENT        0   // identify self to server
#define CLIENT_REQ_TAKEOFF      1   // command the helicopter to take off
#define CLIENT_REQ_LANDING      2   // command the helicopter to land
#define CLIENT_REQ_TELEMETRY    3   // state, orientation, altitude, battery
#define CLIENT_REQ_MJPG_FRAME   4

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
#define PKT_VTI_LENGTH      (sizeof(uint32_t) * (PKT_BASE + 6))

#endif // _UAV_PROTOCOL__H_

