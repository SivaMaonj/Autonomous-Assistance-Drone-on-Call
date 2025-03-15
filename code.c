#include <MAVLink.h>
#include <Preferences.h>
#include <math.h>

#define SERIAL_PORT Serial2       // Pixhawk (TX=17, RX=16)
#define SIM800L Serial1           // GSM Module (RX=4, TX=2)
#define BAUD_RATE 57600
#define EARTH_RADIUS 6371000      // in meters

Preferences preferences;

// Mission parameters
const float TARGET_ALTITUDE = 7.0f;         // 7 m target altitude
const float ACCEPTANCE_RADIUS = 2.0f;         // 2 m acceptance radius
const unsigned long STATE_INTERVAL = 1000;    // State machine update: 1 s
const unsigned long OFFBOARD_INTERVAL = 100;  // Offboard updates: 10 Hz (100 ms)
const uint16_t OFFBOARD_TIMEOUT = 2000;       // Offboard failsafe timeout: 2 s
const float ALTITUDE_TOLERANCE = 0.3f;        // 0.3 m altitude deadzone

// MAVLink configuration: ignore velocity, acceleration, yaw, etc.
const uint16_t TYPE_MASK = (MAV_POS_TARGET_TYPE_MASK_VX_IGNORE | 
                           MAV_POS_TARGET_TYPE_MASK_VY_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_VZ_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_AX_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_AY_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_AZ_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_YAW_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK_YAW_RATE_IGNORE);

// Mission states
enum State { 
  STATE_IDLE = 0,
  STATE_ARMING,
  STATE_TAKEOFF,
  STATE_NAVIGATING,
  STATE_LANDING,
  STATE_COMPLETE
};

// Drone state structure
struct DroneState {
  float lat, lon, alt;
  bool is_armed;
} droneState;

// Mission control structure (persistent)
struct MissionControl {
  float target_lat, target_lon;
  State state;
  unsigned long last_update;
  unsigned long last_offboard;
  bool offboard_active;
} mission;

//======================================================================
// Function Prototypes
//======================================================================
void initializeGSM();
bool sendATCommand(const char* cmd, unsigned long timeout);
void clearGSMBuffer();
void checkSMS();
void processAndDeleteSMS(int index);
void parseSMS(String &sms);
bool isValidCoordinate(float lat, float lon);

void initializePixhawk();
void updateDroneState();
void loadPersistentState();
void savePersistentState();

void runStateMachine();
void runHighFrequencyTasks();
bool checkPositionReached();
float calculateDistance();

void setGuidedMode();
void armDrone(bool arm);
void sendTakeoff();
void sendOffboardPosition();
void activateOffboardMode();
void deactivateOffboardMode();
void sendLandCommand();

//======================================================================
// Setup
//======================================================================
void setup() {
  Serial.begin(115200);
  SERIAL_PORT.begin(BAUD_RATE, SERIAL_8N1, 16, 17);
  
  // Initialize GSM without using a reset pin
  initializeGSM();
  
  // Initialize Pixhawk
  initializePixhawk();
  
  // Initialize persistent storage and load state
  preferences.begin("drone", false);
  loadPersistentState();
  
  // Initialize timing variables
  mission.last_update = millis();
  mission.last_offboard = millis();
  mission.offboard_active = false;
  
  Serial.println("🚀 Cold Start Complete - Ready for Missions");
}

//======================================================================
// Main Loop
//======================================================================
void loop() {
  checkSMS();
  updateDroneState();
  runStateMachine();
  runHighFrequencyTasks();
}

//======================================================================
// GSM Functions (RST pin removed)
//======================================================================
void initializeGSM() {
  SIM800L.begin(9600);
  sendATCommand("AT+CMGF=1", 500);              // Set SMS text mode
  sendATCommand("AT+CMGDA=\"DEL ALL\"", 1000);    // Delete all SMS messages
  sendATCommand("AT+CNMI=1,2,0,0,0", 500);         // Immediate notifications
  clearGSMBuffer();
}

bool sendATCommand(const char* cmd, unsigned long timeout) {
  SIM800L.println(cmd);
  unsigned long start = millis();
  String response;
  while(millis() - start < timeout) {
    if(SIM800L.available()) {
      char c = SIM800L.read();
      response += c;
      if(response.endsWith("OK\r\n") || response.endsWith("ERROR\r\n")) break;
    }
  }
  if(response.indexOf("OK") != -1) return true;
  Serial.print("AT command failed: ");
  Serial.println(cmd);
  return false;
}

void clearGSMBuffer() {
  while(SIM800L.available()) SIM800L.read();
}

void checkSMS() {
  while(SIM800L.available()) {
    String sms = SIM800L.readString();
    sms.trim();
    if(sms.startsWith("+CMTI:")) {
      int msgIndex = sms.substring(sms.lastIndexOf(',') + 1).toInt();
      processAndDeleteSMS(msgIndex);
    }
  }
}

void processAndDeleteSMS(int index) {
  SIM800L.print("AT+CMGR=");
  SIM800L.println(index);
  delay(500);
  
  String sms;
  while(SIM800L.available()) {
    char c = SIM800L.read();
    if(c != '\n' && c != '\r') sms += c;
  }
  sms.trim();
  
  // Revised SMS format: "Drone called! <lat>,<lon>"
  if(sms.indexOf("Drone called!") != -1) {
    int startIdx = sms.indexOf("!") + 2; // Skip "!" and a space
    int commaIndex = sms.indexOf(',', startIdx);
    if(startIdx != -1 && commaIndex != -1) {
      String lat_str = sms.substring(startIdx, commaIndex);
      String lon_str = sms.substring(commaIndex + 1);
      lat_str.trim();
      lon_str.trim();
      float lat = lat_str.toFloat();
      float lon = lon_str.toFloat();
      if(isValidCoordinate(lat, lon)) {
        mission.target_lat = lat;
        mission.target_lon = lon;
        mission.state = STATE_ARMING;
        savePersistentState();
        Serial.println("⚡ New Mission Received!");
        Serial.print("Extracted Latitude: "); Serial.println(lat_str);
        Serial.print("Extracted Longitude: "); Serial.println(lon_str);
      }
    }
  }
  
  // Delete processed SMS
  SIM800L.print("AT+CMGD=");
  SIM800L.println(index);
  delay(200);
  clearGSMBuffer();
}

bool isValidCoordinate(float lat, float lon) {
  return (lat >= -90.0f && lat <= 90.0f) && 
         (lon >= -180.0f && lon <= 180.0f) &&
         !(lat == 0.0f && lon == 0.0f);
}

//======================================================================
// Pixhawk/MAVLink & Persistent State Functions
//======================================================================
void initializePixhawk() {
  setGuidedMode();
  armDrone(false);
  delay(1000);
}

void loadPersistentState() {
  mission.state = (State)preferences.getUInt("state", STATE_IDLE);
  mission.target_lat = preferences.getFloat("lat", 0);
  mission.target_lon = preferences.getFloat("lon", 0);
}

void savePersistentState() {
  preferences.putUInt("state", mission.state);
  preferences.putFloat("lat", mission.target_lat);
  preferences.putFloat("lon", mission.target_lon);
}

void updateDroneState() {
  mavlink_message_t msg;
  mavlink_status_t status;
  while(SERIAL_PORT.available()) {
    uint8_t c = SERIAL_PORT.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t gpos;
          mavlink_msg_global_position_int_decode(&msg, &gpos);
          droneState.lat = gpos.lat * 1e-7f;
          droneState.lon = gpos.lon * 1e-7f;
          droneState.alt = gpos.alt / 1000.0f;
          break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          droneState.is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
          break;
        }
      }
    }
  }
}

//======================================================================
// State Machine & High-Frequency Tasks
//======================================================================
void runStateMachine() {
  if(millis() - mission.last_update < STATE_INTERVAL) return;
  mission.last_update = millis();

  switch(mission.state) {
    case STATE_ARMING:
      if(!droneState.is_armed) {
        setGuidedMode();
        armDrone(true);
        Serial.println("🔌 Arming...");
      } else {
        mission.state = STATE_TAKEOFF;
        savePersistentState();
      }
      break;

    case STATE_TAKEOFF:
      sendTakeoff();
      if(fabs(droneState.alt - TARGET_ALTITUDE) < ALTITUDE_TOLERANCE) {
        mission.state = STATE_NAVIGATING;
        activateOffboardMode();
        savePersistentState();
        Serial.println("🛫 Altitude Reached, switching to NAVIGATING");
      }
      break;

    case STATE_NAVIGATING:
      if(checkPositionReached()) {
        mission.state = STATE_LANDING;
        deactivateOffboardMode();
        savePersistentState();
        Serial.println("🎯 Target Reached, initiating LANDING");
      }
      break;

    case STATE_LANDING:
      sendLandCommand();
      if(droneState.alt <= 0.5f) {
        mission.state = STATE_COMPLETE;
        savePersistentState();
        Serial.println("✅ Landing Complete");
      }
      break;

    case STATE_COMPLETE:
      armDrone(false);
      mission.state = STATE_IDLE;
      preferences.remove("state");
      preferences.remove("lat");
      preferences.remove("lon");
      Serial.println("🔒 Mission Complete - State Reset");
      break;

    case STATE_IDLE:
      // Waiting for new mission command via SMS.
      break;
  }
}

void runHighFrequencyTasks() {
  if(mission.state == STATE_NAVIGATING) {
    if(millis() - mission.last_offboard > OFFBOARD_TIMEOUT) {
      Serial.println("⚠️ Offboard Timeout! Landing...");
      mission.state = STATE_LANDING;
      deactivateOffboardMode();
      savePersistentState();
      return;
    }
    
    if(millis() - mission.last_offboard > OFFBOARD_INTERVAL) {
      sendOffboardPosition();
      mission.last_offboard = millis();
    }
    
    if(fabs(droneState.alt - TARGET_ALTITUDE) > ALTITUDE_TOLERANCE) {
      Serial.println("🔄 Altitude Correction");
      sendTakeoff();
    }
  }
}

bool checkPositionReached() {
  return calculateDistance() <= ACCEPTANCE_RADIUS;
}

float calculateDistance() {
  float lat1 = radians(droneState.lat);
  float lat2 = radians(mission.target_lat);
  float dLat = lat2 - lat1;
  float dLon = radians(mission.target_lon - droneState.lon);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
  return EARTH_RADIUS * 2 * atan2(sqrt(a), sqrt(1-a));
}

//======================================================================
// MAVLink Command Functions
//======================================================================
void armDrone(bool arm) {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_COMP_ID_AUTOPILOT1,
    MAV_CMD_COMPONENT_ARM_DISARM, 0,
    arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
}

void setGuidedMode() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  // Mode 4 is typically GUIDED in ArduPilot
  mavlink_msg_set_mode_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
}

void sendTakeoff() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  // Send takeoff command using current drone position
  mavlink_msg_command_int_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_COMP_ID_AUTOPILOT1,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MAV_CMD_NAV_TAKEOFF, 0, 1,
    0, 0, 0, 0,
    (int32_t)(droneState.lat * 1e7),
    (int32_t)(droneState.lon * 1e7),
    TARGET_ALTITUDE * 1000,
    MAV_MISSION_TYPE_MISSION
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
}

void sendOffboardPosition() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_set_position_target_global_int_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    millis(),
    MAV_COMP_ID_AUTOPILOT1,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    TYPE_MASK,
    (int32_t)(mission.target_lat * 1e7),
    (int32_t)(mission.target_lon * 1e7),
    TARGET_ALTITUDE * 1000,
    0, 0, 0,
    0, 0, 0,
    0, 0
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
  mission.last_offboard = millis();
}

void activateOffboardMode() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_COMP_ID_AUTOPILOT1,
    MAV_CMD_NAV_GUIDED_ENABLE, 0,
    1, 0, 0, 0, 0, 0, 0
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
  mission.offboard_active = true;
  mission.last_offboard = millis();
  Serial.println("📡 Offboard mode activated");
}

void deactivateOffboardMode() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_COMP_ID_AUTOPILOT1,
    MAV_CMD_NAV_GUIDED_ENABLE, 0,
    0, 0, 0, 0, 0, 0, 0
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
  mission.offboard_active = false;
}

void sendLandCommand() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    255, MAV_COMP_ID_MISSIONPLANNER, &msg,
    1, MAV_COMP_ID_AUTOPILOT1,
    MAV_CMD_NAV_LAND, 0,
    0, 0, 0, 0, 0, 0, 0
  );
  SERIAL_PORT.write(buffer, mavlink_msg_to_send_buffer(buffer, &msg));
}
