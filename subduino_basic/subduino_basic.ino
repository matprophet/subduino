#include <Arduino.h>
#include <SoftwareSerial.h>

// CAN support using https://github.com/McNeight/CAN_Library
#include <CAN.h>
#include <SPI.h>
#include <CAN_MCP2515.h>

// Debug logging
//#define DEBUG 
#ifdef DEBUG 
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

// Define the CAN speed (bitrate).
#define bitrate   CAN_BPS_500K
#define CANBaseID 0x259 // 601

//  Arduino Serial Port
const int RXChannel = 7;
const int TXChannel = 6;
SoftwareSerial serialPort = SoftwareSerial(RXChannel, TXChannel);

unsigned long lastTimeMS;
bool isClearToRequest;
const int readDelayMS = 2;
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
byte ReqData[31] = {128, 16, 240, 26, 168, 0, 0, 0, 15, 0, 0, 14, 0, 0, 70, 0, 0, 21, 0, 0, 13, 0, 0, 18, 0, 0, 8, 0, 0, 9, 234};
byte ReqDataSize = 31;
int  ECUbytes[7] = {0, 0, 0, 0, 0, 0, 0};


void setup()
{
  DPRINTLN("Connecting to Serial...");
  Serial.begin(115200); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  DPRINTLN("Initializing SSM serial port...");
  serialPort.begin(4800); //SSM uses 4800 8N1 baud rate
  while (!serialPort) {
    delay(50);
  }

  DPRINTLN("SSM Serial Line Established");
  DPRINTLN("Setup Complete");

  isClearToRequest = true;
  
  delay(50);
  requestECUData();
  delay (2);
}

void loop()
{
  unsigned long now = millis();

  if ((now - lastTimeMS) > maxPollingIntervalMS) {
    serialPort.flush();
    lastTimeMS = now;
    requestECUData();
  }

  if (serialPort.available() && readECU(ECUbytes, 7, false)) {
    lastTimeMS = now;
    CAN2RCP1(ECUbytes[0], // RBP low
             ECUbytes[1], // RPM high
             ECUbytes[2], // AFR
             ECUbytes[3], // TPS
             ECUbytes[4], // MAP
             ECUbytes[5], // IAT
             ECUbytes[6], // Temp
             0); // AFC
  }

  if (isClearToRequest == true) {
    requestECUData();
    isClearToRequest = false;
  }
}

/* returns the 8 least significant bits of an input byte*/
byte CheckSum(byte sum) {
  byte counter = 0;
  byte power = 1;
  for (byte n = 0; n < 8; n++) {
    counter += bitRead(sum, n) * power;
    power = power * 2;
  }
  return counter;
}

void requestECUData() {
  writeECU(ReqData, ReqDataSize, serialPort);
}

/*writes data over the software serial port
  the &digiSerial passes a reference to the external
  object so that we can control it outside of the function*/
void writeECU(byte data[], byte length, SoftwareSerial &digiSerial) {
  DPRINT("TX packet length: ");
  DPRINT(length);
  DPRINTLN("");
  for (byte x = 0; x < length; x++) {
    digiSerial.write(data[x]);
  }
}

// Changes the values in dataArray, populating it with values respective of the poll array address calls
boolean readECU(int* dataArray, byte dataArrayLength, boolean nonZeroes)
{
  byte data = 0;
  boolean isPacket = false;
  byte sumBytes = 0;
  byte checkSumByte = 0;
  byte dataSize = 0;
  byte bytePlace = 0;
  byte zeroesLoopSpot = 0;
  byte loopLength = 20;

  for (byte j = 0; j < loopLength; j++)
  {
    data = serialPort.read();
    delay(readDelayMS);
    
    if (data == 128 && dataSize == 0) { //0x80 or 128 marks the beginning of a packet
      isPacket = true;
      j = 0;
      DPRINTLN("Begin Packet");
    }

    // terminate function and return false if no response is detected
    if (j == (loopLength - 1) && isPacket != true)
    {
      DPRINTLN("no data");
      return false;
    }

    if (isPacket == true && data != -1) {
      DPRINT(data); // for debugging: shows in-packet data
      DPRINT(" ");

      if (bytePlace == 3) { // how much data is coming
        dataSize = data;
        loopLength = data + 6;
      }

      if (bytePlace > 4 && bytePlace - 5 < dataArrayLength && nonZeroes == false)
      {
        dataArray[bytePlace - 5] = data;
      }
      else if (bytePlace > 4 && zeroesLoopSpot < dataArrayLength / 2 && nonZeroes == true && data != 0 && bytePlace < dataSize + 4)
      {
        dataArray[zeroesLoopSpot] = data;
        dataArray[zeroesLoopSpot + (dataArrayLength / 2)] = bytePlace;
        zeroesLoopSpot++;
      }

      bytePlace += 1; //increment bytePlace

      // Once the data is all received, checksum and re-set counters
      DPRINT("byte place: ");
      DPRINTLN(bytePlace);

      if (bytePlace == dataSize + 5) {
        checkSumByte = CheckSum(sumBytes);  //the 8 least significant bits of sumBytes

        if (data != checkSumByte) {
          DPRINTLN(F("checksum error"));
          return false;
        }

        DPRINTLN("Checksum is good");

        isClearToRequest = true;
        isPacket = false;
        sumBytes = 0;
        bytePlace = 0;
        checkSumByte = 0;
        dataSize = 0;
        return true;
      }
      else
      {
        sumBytes += data; // this is to compare with the checksum byte
        DPRINT(F("sum: "));
        DPRINTLN(sumBytes);
      }
    }
  }
}

void CAN2RCP1(byte Rpm1, byte Rpm2, byte AFR, byte Tps, byte MAP, byte IAT, byte Temp, byte AFC)
{
  CAN_Frame standard_message; // Create message object to use CAN message structure
  standard_message.id = CANBaseID;
  standard_message.valid = true;
  standard_message.rtr = 0;
  standard_message.extended = CAN_STANDARD_FRAME;
  standard_message.timeout = 100;
  standard_message.length = 8; // Data length
  standard_message.data[0] = Rpm1;
  standard_message.data[1] = Rpm2;
  standard_message.data[2] = AFR;  // P0x46 = low byte - Divide value by 128.0 to get Lambda
  standard_message.data[3] = Tps;  // P0x15 = low byte - Multiply value by 100.0 and divide by 255 to get percent
  standard_message.data[4] = MAP;  // P0x0D = low byte - Multiply value by 37.0 and divide by 255 to get psig
  standard_message.data[5] = IAT;  // P0x12 = low byte - Subtract 40 from value to get Degrees C 
  standard_message.data[6] = Temp; // P0x08 = low byte - Subtract 40 from value to get Degrees C
  standard_message.data[7] = AFC;  // P0x09 = low byte - Subtract 128 from value and divide by 1.28 to get percent (air/fuel correction)
  
  uint8_t numBytesSent = CAN.write(standard_message); // Load message and send

  DPRINT(" CAN2RCP1 sent:");
  DPRINT(numBytesSent);
  DPRINTLN();
}



