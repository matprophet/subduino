#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <assert.h>
#include <SSMCAN.h>

// Global Settings
#define SSM_REQUEST_INTERVAL  1000

// Use block reading (not supported on all Subaru ECUs) to read
// parameters in blocks of data from the ECU.
#define ENABLE_SSM_BLOCK_READS 0

#define CAPABILITIES_SIZE 48 // number of bytes

// Debug logging
#if 1
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

SoftwareSerial gSerialPort = SoftwareSerial(6, 7); // Rx, Tx

// -----------------------------------------------
//  ECU Addresses
// -----------------------------------------------
int gECUMemorySize = 304;
byte gECUMemory[304]; // 304 bytes should fit all available ECU data

// -----------------------------------------------
//  SETUP
// -----------------------------------------------
void setup()
{
  DPRINTLN("Connecting to Serial...");
  Serial.begin(SERIAL_DIAG_SPEED); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  DPRINTLN("Initializing SSM serial port...");
  gSerialPort.begin(SSM_BUS_SPEED); // SSM uses 4800 8N1 baud rate
  do{
    delay(50);
  } while (!gSerialPort);
  DPRINTLN("SSM Serial Line Established.");
}

// -----------------------------------------------
//  LOOP
// -----------------------------------------------
void loop()
{
  generateRandomECUData();

  if (!gSerialPort.available()) {
    // DPRINTLN("no serial port data");
    return;
  }  

  SSMPacket *commandPacket = readPacketFromSSMBus(gSerialPort);
  if (!commandPacket) {
    // DPRINTLN("no command packet");
     return;
  }

  SSMPacket *responsePacket = NULL;

  SSMCommand command = (SSMCommand)(commandPacket->data[0]);
  switch (command) {
    case kSSMCommandECUInit: {
      responsePacket = responsePacketForSSMInit(commandPacket);
      break;
    }

    case kSSMCommandBlockRead: {
      responsePacket = responsePacketForBlockRead(commandPacket);
      break;
    }

    case kSSMCommandBlockWrite: {
      responsePacket = responsePacketForBlockWrite(commandPacket);
      break;
    }
    
    case kSSMCommandAddressRead: {
      responsePacket = responsePacketForAddressRead(commandPacket);
      break;
    }

    case kSSMCommandAddressWrite: {
      responsePacket = responsePacketForAddressWrite(commandPacket);
      break;
    }
    
    default: {
      DPRINT("Unknown SSM command: ");
      DPRINTLN(command);
      break;
    }
  }

  if (responsePacket) {
    sendSSMPacket(responsePacket, gSerialPort);
    freePacket(responsePacket);
  }

  freePacket(commandPacket);
}


// -----------------------------------------------
//   Packet Builders
// -----------------------------------------------
SSMPacket *responsePacketForSSMInit(SSMPacket *commandPacket) {
  DPRINTLN("Command: ECU Init");
  assert(commandPacket->data[0] == kSSMCommandECUInit);

  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceDiagTool;
  packet->source = kSSMDeviceECU;
  packet->dataSize = 1 + 5 + CAPABILITIES_SIZE; // Response + ID length + Capabilites
  packet->data = (byte *)malloc( packet->dataSize );
  packet->data[0] = kSSMResponseECUInit;

  byte ecuID[5] = {0x11, 0x22, 0x33, 0x44, 0x55}; // Fake ID, obviously.
  byte capabilities[CAPABILITIES_SIZE]; // TODO: Fill with capabilities
  
  memcpy(packet->data + 1, ecuID, 5);      
  memcpy(packet->data + 1 + 5, capabilities, CAPABILITIES_SIZE);      

  return packet;
}

SSMPacket *responsePacketForBlockRead(SSMPacket *commandPacket) {
  /*
  Example:
  Block Read: Read 128 bytes from address 0×200000 (ecu returned all zeros)
  Sent:
  0x80 0x10 0xF0 0x06 0xA0 0x00 0x20 0x00 0x00 0x7F 0xC5

  0x80 - Header
  0x10 - Diag tool source
  0xF0 - ECU target
  0x06 - Data size num bytes
  0xA0 0x00 - Command: kSSMCommandBlockRead
  0x20 0x00 0x00 - Address 0×200000
  0x7F - 127 num bytes to read in addition to the base address
  0xC5 - Checksum
  
  Received: 
  0x80 0xF0 0x10 0x81 0xE0
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 
  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xE1
  
  0x80 - Header
  0xF0 - ECU source
  0x10 - Diag tool target
  0x81 - data size num bytes
  0xE0 - Response Block Read
  0x00 - 0x00
  0xE1 - Checksum  
   */
  
  DPRINTLN("Command: Block Read");
  assert(commandPacket->data[0] == kSSMCommandBlockRead);
  
  unsigned int address = 0;
  address |= commandPacket->data[2];
  address = address << 8;
  address |= commandPacket->data[3];
  address = address << 8;
  address |= commandPacket->data[4];    
  
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceDiagTool;
  packet->source = kSSMDeviceECU;
  packet->dataSize = 1 + 1 + commandPacket->data[6];
  packet->data = (byte *)malloc( packet->dataSize );
  packet->data[0] = kSSMResponseBlockRead;

  if ((address + packet->dataSize - 1) < gECUMemorySize) {
    memcpy(packet->data + 1, &gECUMemory[address], packet->dataSize - 1);      
  }
  
  return packet;
}

SSMPacket *responsePacketForBlockWrite(SSMPacket *commandPacket) {
  /*
  Example:
  Block Write: Write 4 bytes to address 0×200000 (ecu returns written data)   
  Sent:
  0x80 0x10 0xF0 0x08 0xB0 0x20 0x00 0x00 0x01 0x02 0x03 0x04 0x62
  0x80 - Header
  0x10 - Diag tool source
  0xF0 - ECU target
  0x08 - data size num bytes
  0xB0 - Command: kSSMCommandBlockWrite
  0x20 0x00 0x00 - Address 0×200000
  0x01 0x02 0x03 0x04 - 4 byte value to write
  0x62 - Checksum

  Received:
  0x80 0xF0 0x10 0x05 0xF0 0x01 0x02 0x03 0x04 0x7F
  0x80 - start
  0xF0 - ECU source
  0x10 - Diag tool target
  0x05 - data size num bytes
  0xF0 - Response: kSSMResponseBlockWrite
  0x01 0x02 0x03 0x04 - Echo of written value
  0x7F - Checksum
  */
  
  DPRINTLN("Command: Block Write");
  assert(commandPacket->data[0] == kSSMCommandBlockWrite);
  byte sizeOfWrite = commandPacket->dataSize - (1 + 3); // Command + address bytes

  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceDiagTool;
  packet->source = kSSMDeviceECU;
  packet->dataSize = 1 + sizeOfWrite; // Respone + sizeOfWrite 
  
  packet->data = (byte *)malloc( packet->dataSize );
  packet->data[0] = kSSMResponseBlockWrite;
  memcpy(packet->data + 1, // skip the response byte 
         commandPacket->data + (1 + 3), // 1 command byte, 3 address bytes
         sizeOfWrite); // write the echo data

  // don't acutally write any data
  
  return packet;
}

SSMPacket *responsePacketForAddressRead(SSMPacket *commandPacket) {
  /*
  Example:
  Address Read: Read Address 0×000008 and 0×00001C (ecu returns values 0×7D and 0xB1) 
  
  Sent:
  0x80 0x10 0xF0 0x08 0xA8 0x00 0x00 0x00 0x08 0x00 0x00 0x1C 0x54
  0x80 - Header
  0x10 - Diag tool source
  0xF0 - ECU target
  0x08 - data size num bytes
  0xA8 - Command: Address Read
  0x00 - PP: 0×00 (single response), 0×01 (respond until interrupted)
  0x00 0x00 0x08 - First address to read
  0x00 0x00 0x1C - Second address to read
  0x54 - Checksum
  
  Received:
  0x80 0xF0 0x10 0x03 0xE8 0x7D 0xB1 0x99
  0x80 - Header
  0xF0 - ECU source
  0x10 - Diag tool target
  0x03 - data size num bytes
  0xE8 - Response: Address Read
  0x7D - Address 1
  0xB1 - Address 2
  0x99 - Checksum
  */

  DPRINTLN("Command: Address Read");
  assert(commandPacket->data[0] == kSSMCommandAddressRead);

  int numberOfAddresses = commandPacket->dataSize - 2; // Command and PP
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceDiagTool;
  packet->source = kSSMDeviceECU;  
  packet->dataSize = (1 + numberOfAddresses); // Response + number of addresses
  packet->data = (byte *)malloc( packet->dataSize );
  packet->data[0] = kSSMResponseAddressRead;
  int idx = 1;
  for (byte i=1; i<=numberOfAddresses; i++){
    unsigned int address = 0;
    address |= commandPacket->data[idx];
    address = address << 8;
    address |= commandPacket->data[idx+1];
    address = address << 8;
    address |= commandPacket->data[idx+2];
    idx += 3;
    if (address < gECUMemorySize) {
      packet->data[i] = gECUMemory[address];
    }
  }

  return packet;
}

SSMPacket *responsePacketForAddressWrite(SSMPacket *commandPacket) {
  /*
  Example
  Write single address: Write value 0×02 to address 0×00006F 
  
  Sent:
  0x80 0x10 0xF0 0x05 0xB8 0x00 0x00 0x6F 0x02 0xAE
  0x80 - Header
  0x10 - Diag tool source
  0xF0 - ECU target
  0x05 - data size num bytes
  0xB8 - Command: Address Write
  0x00 0x00 0x6F - Address
  0x02 - Value to write
  0xAE - Checksum
  
  Received:
  0x80 0xF0 0x10 0x02 0xF8 0x02 0x7C
  0x80 - Header
  0xF0 - ECU source
  0x10 - Diag tool target
  0x02 - data size num bytes
  0xF8 - Response: Address Write
  0x02 - Echo'd value that was written
  0x7C - Checksum
 */
  
  DPRINTLN("Command: Address Write");
  assert(commandPacket->data[0] == kSSMCommandAddressWrite);

  unsigned int address = 0;
  address |= commandPacket->data[1];
  address = address << 8;
  address |= commandPacket->data[2];
  address = address << 8;
  address |= commandPacket->data[3];
  if (address < gECUMemorySize) {
    gECUMemory[address] = commandPacket->data[4];
  }
  
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceDiagTool;
  packet->source = kSSMDeviceECU;  
  packet->dataSize = 2; // response + echo of written value
  packet->data = (byte *)malloc( packet->dataSize );
  packet->data[0] = kSSMResponseAddressWrite;
  packet->data[1] = commandPacket->data[4];

  return packet;
}

void generateRandomECUData() {
  #define MINRPM (750 * 4)
  #define RPMINC (10*4)
  #define MAXRPM (7500 *4)
  static uint16_t gRPM = (6201 * 4);
  gRPM += RPMINC;
  if (gRPM > MAXRPM)
    gRPM = MINRPM;

  gECUMemory[0x0E] = (gRPM & 0xff);
  gECUMemory[0x0F] = (gRPM >> 8);

  // AFR - Divide value by 128.0 to get Lambda
  gECUMemory[0x46] = 110 + (rand() % 30);

  // IAT - Subtract 40 from value to get Degrees C
  gECUMemory[0x08] = 110 + (rand() % 30); 

  // TPS - Multiply value by 100.0 and divide by 255 to get percent
  gECUMemory[0x15] = rand() % 255;

  // MAP - Multiply value by 37.0 and divide by 255 to get psig
  // range of 0-22psi = 100 - 252
  gECUMemory[0x0D] = (100 + rand() % 150);

  // IAT - Subtract 40 from value to get Degrees C
  gECUMemory[0x12] = rand() % 255;

  // AFC - Subtract 128 from value and divide by 1.28 to get percent (air/fuel correction)
  gECUMemory[0x09] = rand() % 128;

  // Battery - Multiply value by 0.08 to get volts
  gECUMemory[0x1C] = 12 + rand() % 5;
  
  // Gear - 1 + to get gear
  gECUMemory[0x4A] = 1 + rand() % 4;

  // Speed - in kph
  static byte KPH = 0;
  KPH += 1;
  if (KPH > 160)
    KPH = 0;
  gECUMemory[0x10] = 100;
}

/*
byte gECUData[] = {
    0x0F, // Engine Speed 2 (RPM) low byte
    0x0E, // Engine Speed 1 (RPM) high byte
    0x46, // Air/Fuel Sensor #1
    0x15, // Throttle Opening Angle 
    0x0D, // Manifold Absolute Pressure
    0x10, // Vehicle Speed      
    0x08, // Coolant Temperature    
    0x4A, // Gear Position
    0x11, // Ignition Timing      
    0x12, // Intake Air Temperature 
    0x13, // Mass Air Flow 1   
    0x14, // Mass Air Flow 2
    0x16, // Front O2 Sensor #1   
    0x17, // Front O2 Sensor #1 - 2
    0x18, // Rear O2 Sensor
    0x19, // Oil pressure (psi)
    0x07, // Engine Load    
    0x09, // Air/Fuel Correction #1 
    0x0A, // Air/Fuel Learning #1   
    0x0B, // Air/Fuel Correction #2 
    0x0C, // Air/Fuel Learning #2   
    0x1A, // Front O2 Sensor #2   
    0x1B, // Front O2 Sensor #2 - 2
    0x30, // Primary Wastegate Duty Cycle
    0x23, // Atmospheric Pressure
    0x1C, // Battery Voltage
    0x1D, // Air Flow Sensor Voltage
    0x1E, // Throttle Sensor Voltage
    0x1F, // Diff Pres. Sens V.
    0x20, // Fuel Injection #1 Pulse W
    0x21, // Fuel Injection #2 Pulse W
    0x22, // Knock Correction
    0x24, // Manifold Relative Pressure
    0x25, // Pressure Differential Sensor
    0x26, // Fuel Tank Pressure
    0x27, // CO Adjustment
    0x28, // Learned Ignition Timing
    0x29, // Accelerator Opening Angle
    0x2A, // Fuel Temperature
    0x2B  // Front O2 Heater #1
    0x2C, // Rear O2 Heater Current
    0x2D, // Front O2 Heater #2
    0x2E, // Fuel Level (%)
    0x31, // Secondary Wastegate Duty Cycle
    0x32, // CPC Valve Duty Ratio
    0x33, // Tumble Valve Position Sensor Right
    0xD3, // Air/Fuel Adjustment Voltage
    0x34, // Tumble Valve Position Sensor Left
    0x35, // Idle Speed Control Valve Duty Ratio
    0x36, // Air/Fuel Lean Correction
    0x37, // Air/Fuel Heater Duty
    0x38, // Idle Speed Control Valve Step
    0x39, // Number of Ex. Gas Recirc Steps
    0x3A, // Alternator Duty
    0x3B, // Fuel Pump Duty
    0x3C, // VVT Advance Angle Right
    0x3D, // VVT Advance Angle Left
    0x3E, // OCV Duty Right
    0x3F, // OCV Duty Left
    0x40, // OCV Current Right
    0x41, // OCV Current Left
    0x42, // Air/Fuel Sensor #1 Current
    0x43, // Air/Fuel Sensor #2 Current
    0x44, // Air/Fuel Sensor #1 Resistance
    0x45, // Air/Fuel Sensor #2 Resistance
    0x47, // Air/Fuel Sensor #2
    0x53, // Air/Fuel Sensor #1 Heater Current
    0x54, // Air/Fuel Sensor #2 Heater Current
    0xD0, // Air/Fuel Correction #3
    0xD1, // Air/Fuel Learning #3
    0xD2, // Rear O2 Heater Voltage
    0xCE, // Roughness Monitor Cylinder #1
    0xCF, // Roughness Monitor Cylinder #2
    0xD8, // Roughness Monitor Cylinder #4
    0xD9, // Roughness Monitor Cylinder #3
    0xFA, // Throttle Motor Duty
    0xFB, // Throttle Motor Voltage
    0x120, // Switch - "ETC Motor Relay"
    0x121, // Switch - "Accel", "Brake", "Clutch", etc...
    0x100, // Sub Throttle Sensor
    0x101, // Main Throttle Sensor
    0x102, // Sub Accelerator Sensor
    0x103, // Main Accelerator Sensor
    0x104, // Brake Booster Pressure
    0x105, // Fuel Pressure (High)
    0x106, // Exhaust Gas Temperature
    0x108, // Cold Start Injector
    0x61,
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
    0x67, // skips 0x68...
    0x69,
};
*/
