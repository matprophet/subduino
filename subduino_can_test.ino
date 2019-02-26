#include <SoftwareSerial.h>
#include <Arduino.h>

// CAN support using https://github.com/McNeight/CAN_Library
#include <CAN.h>
#include <SPI.h>
#include <CAN_MCP2515.h>

boolean debugLogEnabled = true;

// Debug logging
#define DEBUG 
#ifdef DEBUG 
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

// Define our CAN speed (bitrate).
#define bitrate CAN_BPS_500K
#define CANBaseID 0x259 // 601

unsigned long prvTime;
unsigned long curTime;

void setup() {
  
  CAN_MCP2515( 10 );  
  
  Serial.begin(115200); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  DPRINTLN("Can Init");
  CAN.begin(bitrate);

  delay(50);
  DPRINTLN("Setup Complete");
  delay(50);

  curTime = prvTime = millis();
}

void loop() {  
  curTime = millis();
  if (curTime < prvTime + 1000) {
    return;
  }
  prvTime = curTime;

  int rpm = prvTime % 7000;

  // Divide value by 128.0 to get Lambda
  int afr = 110 + (rand() % 30);

  // Subtract 40 from value to get Degrees C
  int temp = 110 + (rand() % 30); 

  // Multiply value by 100.0 and divide by 255 to get percent
  int tps = rand() % 255;

  // Multiply value by 37.0 and divide by 255 to get psig
  int manifold_pressure = rand() % 165;

  // Subtract 40 from value to get Degrees C
  int iat  = rand() % 255;

  int afc = rand() % 20;

  DPRINT("RPM:");
  DPRINT(rpm);
  DPRINT(" AFR:");
  DPRINT(afr);
  DPRINT(" TPS:");
  DPRINT(tps);
  DPRINT(" MAP:");
  DPRINT(manifold_pressure);
  DPRINT(" IAT:");
  DPRINT(iat);
  DPRINT(" TEMP:");
  DPRINT(temp);
  DPRINT(" AFC:");
  DPRINT(afc);
  
  uint8_t numBytesSent = CAN2RCP1((rpm | 0xff), (rpm >> 8), afr, tps, manifold_pressure, iat, temp, afc);
  DPRINT("- Sent:");
  DPRINT(numBytesSent);
  DPRINTLN();
}

uint8_t CAN2RCP1(byte Rpm1, byte Rpm2, byte AFR, byte Tps, byte MAP, byte IAT, byte Temp, byte AFC)
{
  CAN_Frame standard_message; // Create message object to use CAN message structure
  standard_message.id = CANBaseID; // 601
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
  
  return CAN.write(standard_message); // Load message and sent
}




