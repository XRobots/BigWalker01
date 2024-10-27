#include <Arduino.h>
#include "ODriveCAN.h"

#include <Dynamixel2Arduino.h>

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

#include <IBusBM.h>
IBusBM IBus; // IBus object

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrvives
#define ODRV0_NODE_ID 0

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

# ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000


static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

#endif // IS_MCP2515


/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}


#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 21; // DYNAMIXEL Shield DIR PIN 
const float DXL_PROTOCOL_VERSION = 2.0;

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {0, 1, 2, 3, 4, 5};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position[2] = {1024, 2048};
uint8_t goal_position_index = 0;


BNO08x myIMU;
#define BNO08X_INT  17
#define BNO08X_RST  16
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

float pitch;
float roll;
float yaw;

float x;
float z;
float y;

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float RFBb;
float RLRb;
float RTb;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

float scaled1;
float scaled2;

float pot1;
float pot2;
float pot3;

int sw1;
int sw2;

float shoulder0a;
float shoulder1a;

float shoulder2a;
float shoulder3a;

float elbow;

float encoderTrack1;
int encoderTrack2;
int encoderTrack3;

float shoulder0;
float shoulder0Filtered;
float shoulder1;
float shoulder1Filtered;
float shoulder2;
float shoulder2Filtered;
float shoulder3;
float shoulder3Filtered;
float elbow4;
float elbow4Filtered;
float elbow5;
float elbow5Filtered;

int clFlag;  // ODrive init flag

float wheel0;
int walk = 0;

unsigned long currentMillis;
long previousMillis = 0;        // set up timers
long interval = 10;             // time constant for timer


void setup() {

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  Serial7.begin(115200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  IBus.begin(Serial1, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required

  uint8_t i;
  DEBUG_SERIAL.begin(115200);
  dxl.begin(115200);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

// Prepare the SyncRead structure
  for(i = 0; i < DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

  Wire.begin();
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();
  }
 

  // Here is where you define the sensor outputs you want to receive
  void setReports(void) {
    Serial.println("Setting desired reports");
    if (myIMU.enableRotationVector() == true) {
      Serial.println(F("Rotation vector enabled"));
      Serial.println(F("Output in form roll, pitch, yaw"));
    } else {
      Serial.println("Could not enable rotation vector");
    }

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, 0, 200);

    // Insert a new Goal Position to the SyncWrite Packet              
  sw_data[0].goal_position = 2047;     // DXL ID 0
  sw_data[1].goal_position = 2047;     // DXL ID 1
  sw_data[2].goal_position = 2047;     // DXL ID 2
  sw_data[3].goal_position = 2047;     // DXL ID 3
  sw_data[4].goal_position = 2047;      // DXL ID 4
  sw_data[5].goal_position = 2047;      // DXL ID 5

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;
  
  // Build a SyncWrite Packet and transmit to DYNAMIXEL  
  dxl.syncWrite(&sw_infos); 
}

void loop() {

      currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {  // start timed event    
          previousMillis = currentMillis;

          pot1 = analogRead(A10);
          pot2 = analogRead(A11);
          pot3 = analogRead(A12);
          sw1 = digitalRead(2);
          sw2 = digitalRead(3);

          if (sw1 == 0) {          // Init Odrives

          //check for ODrives

          Serial.println("Waiting for ODrive 0...");
          while (!odrv0_user_data.received_heartbeat) {
            pumpEvents(can_intf);
            delay(100);
          }
          Serial.println("found ODrive 0");
        
          odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
        
          for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
          }
          
              Serial.println("Enabling closed loop control 0...");
                while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrv0.clearErrors();
                delay(1);
                odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
              }
              clFlag = 1;          // reset flag
          }

          else if (sw1 == 1) {      // make sure we only power on ODrives once per button press
            clFlag = 0;
          }

          IBus.loop();
    
          ch1 = IBus.readChannel(0); // get latest value for servo channel 0
          ch2 = IBus.readChannel(1); // get latest value for servo channel 1
          ch3 = IBus.readChannel(2); // get latest value for servo channel 3
          ch4 = IBus.readChannel(3); // get latest value for servo channel 4
          ch5 = IBus.readChannel(4); // get latest value for servo channel 5
          ch6 = IBus.readChannel(5); // get latest value for servo channel 6
          ch7 = IBus.readChannel(6); // get latest value for servo channel 7
          ch8 = IBus.readChannel(7); // get latest value for servo channel 8
          ch9 = IBus.readChannel(8); // get latest value for servo channel 9
          ch10 = IBus.readChannel(9); // get latest value for servo channel 10
    
          LFB = ch1;
          RFB = ch4;
          RLR = ch2;
          RT = ch6;
          LLR = ch3;
          LT = ch5;
          
          // *** threshold sticks ***
          RFBa = thresholdStick(RFB)*-1;
          RLRa = thresholdStick(RLR);
          RTa = thresholdStick(RT);
          LFBa = thresholdStick(LFB);
          LLRa = thresholdStick(LLR);
          LTa = thresholdStick(LT);
    
          // *** filter sticks ***
          RFBFiltered = filter(RFBa, RFBFiltered,50);
          RLRFiltered = filter(RLRa, RLRFiltered,50);
          RTFiltered = filter(RTa, RTFiltered,50);
          LFBFiltered = filter(LFBa, LFBFiltered,10);
          LLRFiltered = filter(LLRa, LLRFiltered,10);
          LTFiltered = filter(LTa, LTFiltered,10);

          if (myIMU.wasReset()) {
              Serial.print("sensor was reset ");
              setReports();
          }
    
          // Has a new event come in on the Sensor Hub Bus?
          if (myIMU.getSensorEvent() == true) {
      
          // is it the correct sensor data we want?
          if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      
              pitch = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
              roll = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
              //yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

              pitch = pitch *-1;
              roll = roll *-1;
              }
          }

          // print position and velocity for Serial Plotter
          if (odrv0_user_data.received_feedback) {
            Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
            odrv0_user_data.received_feedback = false;
            encoderTrack1 = feedback.Pos_Estimate/73.44;
            encoderTrack2 = int(encoderTrack1);
            encoderTrack3 = int(1000)*(encoderTrack1-encoderTrack2);
            //Serial.print(feedback.Pos_Estimate);
            //Serial.print(" , ");
            Serial.print(encoderTrack3);
            Serial.println();
          }

          //*** motor speed control through the walking cycle ***

          if (RFBa > 50) {
            if (encoderTrack3 > -5 && encoderTrack3 < 350) {
              wheel0 = 17;
            }
            else if (encoderTrack3 > 351 && encoderTrack3 < 520) {      // 400 - 520
              wheel0 = 4;
            }
            else if (encoderTrack3 > 521 && encoderTrack3 < 850) {
              wheel0 = 17;
            }
            else if (encoderTrack3 > 851 && encoderTrack3 < 1000) {
              wheel0 = 4;
            }
            else if (encoderTrack3 > 1000) {
              wheel0 = 17;
            }
          }          
          else {
            wheel0 = 0;
          }

          if (sw2 == 0) {
              odrv0.setVelocity(wheel0);
          }

          else if (sw2 == 1) {
              odrv0.setVelocity(0);
          }

          // *** arm control ***

          shoulder0 = 2047;
          shoulder1 = 2047;
          shoulder2 = 1500 - (pitch*75);
          shoulder3 = 2547 + (pitch*75);
          shoulder2 = constrain(shoulder2,1800,2500);
          shoulder3 = constrain(shoulder3,1500,2300);
          shoulder2Filtered = filter(shoulder2, shoulder2Filtered,30);
          shoulder3Filtered = filter(shoulder3, shoulder3Filtered,30);

          elbow4 = 2047 - (pitch*50);
          elbow5 = 2047 + (pitch*50);
          elbow4 = constrain(elbow4,2047,3547);
          elbow5 = constrain(elbow5,500,2047);
          elbow4Filtered = filter(elbow4, elbow4Filtered, 20);
          elbow5Filtered = filter(elbow5, elbow5Filtered, 20);

          // *** manual arm control ***
          if (ch7 > 1200) {                                       
            shoulder0a = RLRFiltered*3;
            shoulder1a = RLRFiltered*3;
            shoulder0a = constrain(shoulder0a,0,800);
            shoulder1a = constrain(shoulder1a,-800,0);

            shoulder2a = RFBFiltered*4;
            shoulder3a = RFBFiltered*4;
            shoulder2a = constrain(shoulder2a,0,1000);
            shoulder3a = constrain(shoulder3a,-1000,0);

            elbow = RTFiltered * 4;
            elbow = constrain(elbow,0,1000);
          }
                     
          // Insert a new Goal Position to the SyncWrite Packet              
          sw_data[0].goal_position = shoulder0 + shoulder0a;     // DXL ID 0
          sw_data[1].goal_position = shoulder1 + shoulder1a;     // DXL ID 1
          sw_data[2].goal_position = (shoulder2Filtered-200) + shoulder2a;     // DXL ID 2
          sw_data[3].goal_position = (shoulder3Filtered+200) + shoulder3a;     // DXL ID 3
          sw_data[4].goal_position = elbow4Filtered + elbow;      // DXL ID 4
          sw_data[5].goal_position = elbow5Filtered - elbow;      // DXL ID 5
        
          // Update the SyncWrite packet status
          sw_infos.is_info_changed = true;
          
          // Build a SyncWrite Packet and transmit to DYNAMIXEL  
          dxl.syncWrite(&sw_infos); 

          // ** head control ***
  
          if (ch7 > 1200) {         // manual animatronic control         
            Serial7.print(500);
            Serial7.print(" , ");
            Serial7.print(LFBa);
            Serial7.print(" , ");
            Serial7.print(LLRa);
            Serial7.print(" , ");
            Serial7.print(LTa);
            Serial7.print('\n');
          }
          else {                    // default positions
            Serial7.print(500);
            Serial7.print(" , ");
            Serial7.print(0);
            Serial7.print(" , ");
            Serial7.print(0);
            Serial7.print(" , ");
            Serial7.print(0);
            Serial7.print('\n');
          }
  

  }     // end of timed loop

   

}
