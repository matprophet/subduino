#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>

boolean debugLogEnabled = false;
boolean canbusDebugLogEnabled = true;

int RX = 7;
int TX = 6;
SoftwareSerial serialPort = SoftwareSerial(RX, TX); // Rx, Tx

int ECUbytes[7] = {0, 0, 0, 0, 0, 0, 0};
//                                   |s|rpm - 2 bytes| AFR  | TPS  | MAP  | IAT  |Temp | AFC |checksum|
byte ReqData[31] = {128, 16, 240, 26, 168, 0, 0, 0, 15, 0, 0, 14, 0, 0, 70, 0, 0, 21, 0, 0, 13, 0, 0, 18, 0, 0, 8, 0, 0, 9, 234};
byte ReqDataSize = 31;

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

int RRPM;
int AFR;
int TPS;
int MAP;
int IAT;
int TEMP;
int AFC;

int SerialStatus = 0;
int milli;
int ClrToSnd;
int readDelay = 2;
unsigned long prvTime;
unsigned long curTime;

void setup()
{
  Serial.println("Connecting to Serial...");
  Serial.begin(115200); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  Serial.println("Initializing SSM serial port...");
  serialPort.begin(4800); //SSM uses 4800 8N1 baud rate
  while (!serialPort) {
    delay(50);
  }

  Serial.println("SSM Serial Line Established");
  Serial.println("Setup Complete");

  delay(50);
  writeECU(ReqData, ReqDataSize, serialPort);
  delay (2);
}

void loop()
{
  curTime = millis();
  milli = curTime - prvTime;

  if (milli > 1000) {
    serialPort.flush();
    writeECU(ReqData, ReqDataSize, serialPort);
    prvTime = millis();
  }

  if (serialPort.available()) {
    readECU(ECUbytes, 7, false);

    prvTime = curTime;

    if (canbusDebugLogEnabled) {
      Serial.print("RPM: ");
      Serial.print(((ECUbytes[0] | (ECUbytes[1] << 8)) / 4.0));
      Serial.print("    ");
      Serial.print("AFR: ");
      Serial.print(ECUbytes[2]);
      Serial.print("    ");
      Serial.print("TPS: ");
      Serial.print(ECUbytes[3]);
      Serial.print("    ");
      Serial.print("MAP: ");
      Serial.print(ECUbytes[4]);
      Serial.print("    ");
      Serial.print("IAT: ");
      Serial.print(ECUbytes[5]);
      Serial.print("    ");
      Serial.print("TEMP: ");
      Serial.print(ECUbytes[6]);
      Serial.print("    ");
      Serial.print("Millis: ");
      Serial.println(milli);
    }
  }

  if (ClrToSnd == 0) {
    writeECU(ReqData, ReqDataSize, serialPort);
    ClrToSnd = 1;
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

/*writes data over the software serial port
  the &digiSerial passes a reference to the external
  object so that we can control it outside of the function*/
void writeECU(byte data[], byte length, SoftwareSerial &digiSerial) {

  if (debugLogEnabled) {
    Serial.print("TX packet length: ");
    Serial.print(length);
    Serial.println("");
  }

  for (byte x = 0; x < length; x++) {
    digiSerial.write(data[x]);
  }
}

// This will change the values in dataArray, populating them with values respective of the poll array address calls
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
    delay(readDelay);
    
    if (data == 128 && dataSize == 0) { //0x80 or 128 marks the beginning of a packet
      isPacket = true;
      j = 0;
      if (debugLogEnabled)
        Serial.println("Begin Packet");
    }

    // terminate function and return false if no response is detected
    if (j == (loopLength - 1) && isPacket != true)
    {
      if (debugLogEnabled)
        Serial.println("no data");

      return false;
    }

    if (isPacket == true && data != -1) {
      if (debugLogEnabled) {
        Serial.print(data); // for debugging: shows in-packet data
        Serial.print(" ");
      }

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
      if (debugLogEnabled) {
        Serial.print("byte place: ");
        Serial.println(bytePlace);
      }

      if (bytePlace == dataSize + 5) {
        checkSumByte = CheckSum(sumBytes);  //the 8 least significant bits of sumBytes

        if (data != checkSumByte) {
          Serial.println(F("checksum error"));
          return false;
        }

        if (debugLogEnabled) {
          Serial.println("Checksum is good");
        }

        ClrToSnd = 0;
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

        if (debugLogEnabled) {
          Serial.print(F("sum: "));
          Serial.println(sumBytes);
        }
      }
    }
  }
}



