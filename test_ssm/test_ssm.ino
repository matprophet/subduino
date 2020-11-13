#include <Arduino.h>
#include <SoftwareSerial.h>

// Debug logging
#define DEBUG 
#ifdef DEBUG 
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

//  Arduino Serial Port
const int RXChannel = 7;
const int TXChannel = 6;
SoftwareSerial gSerialPort = SoftwareSerial(RXChannel, TXChannel);

unsigned long lastTimeMS;
bool isClearToRequest;
const unsigned long maxPollingIntervalMS = 1000;

// Subaru SSM Request Packet
// ReqData
//  128 - 0x80 - Header
//   16 - 0x10 - Destination: ECU
//  240 - 0xF0 - Source: Diagnostic tool
//   26 - 0x1A - Data size: 26
//  168 - 0xA8 - Command: Read addresses
//    0 - 0x00 - Response type: Single
// 0015 - 0x0F - Parameter: RPM low byte
// 0014 - 0x0E - Parameter: RPM high byte
// 0070 - 0x46 - Parameter: AFR
// 0021 - 0x15 - Parameter: TPS - Throttle Opening Angle
// 0013 - 0x0D - Parameter: MAP
// 0018 - 0x12 - Parameter: IAT
//  008 - 0x08 - Parameter: Temp
//  009 - 0x09 - Parameter: AFC
//  234 - 0xEA - Checksum
//                                    |s |     RPM - 2 bytes    |   AFR   |   TPS   |   MAP   |   IAT   |  Temp  |   AFC  |checksum|
byte gReqData[31] = {128, 16, 240, 26, 168, 0, 0, 0, 15, 0, 0, 14, 0, 0, 70, 0, 0, 21, 0, 0, 13, 0, 0, 18, 0, 0, 8, 0, 0, 9, 234};
byte gReqDataSize = 31;
byte gECUbytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};


void setup() {
  Serial.begin(115200); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  gSerialPort.begin(4800); //SSM uses 4800 8N1 baud rate
  while (!gSerialPort) {
    delay(50);
  }

  isClearToRequest = true;
  
  delay(50);
  sendSSMCommand();
  delay(2);
}

void loop() {
  unsigned long now = millis();
  if ((now - lastTimeMS) > maxPollingIntervalMS) {
    gSerialPort.flush();
    lastTimeMS = now;
    sendSSMCommand();
  }

  if (gSerialPort.available() && readSSMResponse(gECUbytes, 8)) {
    lastTimeMS = now;
  }

  if (isClearToRequest == true) {
    sendSSMCommand();
    isClearToRequest = false;
  }
}

void sendSSMCommand() {
  DPRINT("TX packet length: ");
  DPRINT(gReqDataSize);
  DPRINTLN("");
  for (byte x = 0; x < gReqDataSize; x++) {
    gSerialPort.write(gReqData[x]);
  }
}

boolean readSSMResponse(byte* dataArray, byte dataArrayLength) {
  byte loopLength = 10; // allowed failed attempts + number of fields to fill in before dataSize
  byte numAllowedFailed = 5;
  byte failCount = 0;
  boolean didFail = false;
  int portData;
  byte dataSize;
  byte checksum = 0 ;

  for (byte j = 0; j < loopLength; j++){
    checksum = 0;
    if (failCount >= numAllowedFailed) {
        break;
    }

    int portData = gSerialPort.read();
    if (portData == -1) {
        failCount++;
        continue;
    }
    
    if ((portData & 0xFF) != 0x80) { // 0x80 marks the beginning of a packet
        continue;
    }
    checksum += 0x80;

    // Destination : 0xF0 - Diag tool
    portData = gSerialPort.read();
    if (portData == -1) { failCount++; continue; }
    if ( (portData & 0xFF) != 0xF0){ failCount++; DPRINTLN("Err: Read - bad destination"); continue; }
    checksum += 0xF0;

    // Source : 0x10 - ECU
    portData = gSerialPort.read();
    if (portData == -1) { failCount++; continue; }
    if ( (portData & 0xFF) != 0x10){ failCount++; DPRINTLN("Err: Read - bad source"); continue; }
    checksum += 0x10;

    // Datasize
    portData = gSerialPort.read();
    if (portData == -1) { failCount++; continue; }
    dataSize = (portData & 0xFF);
    if ( dataSize == 0){ failCount++;  DPRINTLN("Err: Read - bad datasize"); continue; }
    checksum += dataSize;

    // Data
    didFail = false;
    for (byte i=0; i<dataSize; i++) {
       portData = gSerialPort.read();
       if (portData == -1) { failCount++; didFail = true; continue; }
       dataArray[i] = (portData & 0xFF);
      checksum += dataArray[i];
    }
    if (didFail) { continue; }
    
    // Checksum
    portData = gSerialPort.read();
    if (portData == -1) { failCount++; continue; }
    if (checksum != (portData & 0xFF)){ failCount++; DPRINTLN("Err: Read - checksum mismatch"); continue; }
    else {
      DPRINTLN("Read - Success");
      // success
      break;
    }
  }
}
