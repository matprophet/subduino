#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SSMCAN.h>

// Global Settings
#define SSM_REQUEST_INTERVAL  1000

// Debug logging
#if 1
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

SoftwareSerial gSerialPort = SoftwareSerial(7, 6); // Rx, Tx
bool gIsClearToSend = true;
unsigned long gTimeLast = 0;
unsigned long gTimeCurrent = 0;

// -----------------------------------------------
//  Subduino custom code
// -----------------------------------------------
enum SDRequestState : byte {
  SDRequestStateInit,
  SDRequestStateBlock1,
  SDRequestStateBlock2,
  SDRequestStateSet1,
};

byte gSDMemory[304]; // 304 bytes should fit all the data
SDRequestState gSDRequestState = SDRequestStateInit;

typedef struct {
  byte location;
  byte length;
} SDRequest;
SDRequest gSDRequestBlock1 = { 0x07, 98 }; // Addresses from 0x07 --> 0x69
SDRequest gSDRequestBlock2 = { 0xCE, 83 }; // Addresses from 0xCE --> 0x121

byte gSDRequestAddressSet1Size = 9;
unsigned long gSDRequestAddressSet1SSMAddresses[9] = {
  0x0F, // Engine Speed 2 (RPM) low byte
  0x0E, // Engine Speed 1 (RPM) high byte
  0x0D, // Manifold Absolute Pressure
  0x08, // Coolant Temperature
  0x10, // Vehicle Speed 
  0x12, // Intake Air Temperature
  0x15, // Throttle Opening Angle
  0x1C, // Battery Voltage
  0x46, // Air/Fuel Sensor #1  
};


// -----------------------------------------------
//  SETUP
// -----------------------------------------------
void setup() {
  DPRINTLN("Connecting to Serial...");
  Serial.begin(SERIAL_DIAG_SPEED); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  DPRINTLN("Initializing SSM serial port...");
  gSerialPort.begin(SSM_BUS_SPEED); //SSM uses 4800 8N1 baud rate
  do{
    delay(50);
  } while (!gSerialPort);
  DPRINTLN("SSM Serial Line Established.");

  gTimeLast = millis();
}

// -----------------------------------------------
//  LOOP
// -----------------------------------------------
void loop() {
  gTimeCurrent = millis();
  if ((gTimeCurrent - gTimeLast) > SSM_REQUEST_INTERVAL) {
    gSerialPort.flush();
    DPRINTLN("-----flush-----");
    sendSSMCommandForState(gSDRequestState); // re-send the current request
    gTimeLast = gTimeCurrent;
  }

  if (readSSMResponse()) {
    gTimeLast = gTimeCurrent;
    gIsClearToSend = true;
  }
  
  if (gIsClearToSend) {
    sendSSMCommandForState(gSDRequestState);
    gIsClearToSend = false;
  }
}


// -----------------------------------------------
//  
// -----------------------------------------------
void sendSSMCommandForState(SDRequestState inState) {
  switch (inState) {
    case SDRequestStateInit:
      DPRINTLN("Requesting SSM init...");
      sendSSMPacket(packetForSSMInit(), gSerialPort);
      break;

    case SDRequestStateBlock1:
      DPRINTLN("Requesting SSM block 1...");
      sendSSMPacket(packetForBlockRead(gSDRequestBlock1.location, gSDRequestBlock1.length), gSerialPort);
      break;

    case SDRequestStateBlock2:
      DPRINTLN("Requesting SSM block 2...");
      sendSSMPacket(packetForBlockRead(gSDRequestBlock2.location, gSDRequestBlock2.length), gSerialPort);
      break;

    case SDRequestStateSet1:
      DPRINTLN("Requesting SSM set 1...");
      sendSSMPacket(packetForAddressRead(gSDRequestAddressSet1SSMAddresses, gSDRequestAddressSet1Size), gSerialPort);
      break;
  }
}

bool readSSMResponse() {
  if (!gSerialPort.available()) {
    return false;
  }  

  SSMPacket *packet = readPacketFromSSMBus(gSerialPort);
  if (!packet) {
     DPRINT("ERR: Could not create SSM response packet for request state: ");
     DPRINTLN(gSDRequestState);
     return false;
  }

  switch (gSDRequestState) {
    case SDRequestStateInit: 
      gSDRequestState = SDRequestStateBlock1;      
      break;

    case SDRequestStateBlock1:
      // advance past the response code
      memcpy(&gSDMemory[gSDRequestBlock1.location], packet->data + 1, packet->dataSize - 1);      
      gSDRequestState = SDRequestStateBlock2;
      break;
      
    case SDRequestStateBlock2:
      // advance past the response code
      memcpy(&gSDMemory[gSDRequestBlock2.location], packet->data + 1, packet->dataSize - 1);
      gSDRequestState = SDRequestStateBlock1;
      break;

    case SDRequestStateSet1:
      for (byte i=1; i<=(packet->dataSize - 1); i++){
        gSDMemory[ gSDRequestAddressSet1SSMAddresses[i-1] ] = packet->data[i];
      }
      break;
  }
  
  freePacket(packet);
  return true;
}
