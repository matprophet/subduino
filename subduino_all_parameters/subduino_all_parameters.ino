#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>

// CAN support using https://github.com/McNeight/CAN_Library
#include <CAN.h>
#include <SPI.h>
#include <CAN_MCP2515.h>

// Debug logging
#define DEBUG 
#ifdef DEBUG 
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

#define SERIAL_DIAG_SPEED 115200
#define SSM_BUS_SPEED 4800

#define DEBUG_CAN 1

enum SSMDevice {
  kSSMDeviceECU = 0x10,
  kSSMDeviceTCU = 0x18,
  kSSMDeviceDiagTool = 0xF0,
};

enum SSMCmd {
  kSSMCmdBlockRead = 0xA0,
  kSSMCmdAddressRead = 0xA8,
  kSSMCmdBlockWrite = 0xB0, 
  kSSMCmdAddressWrite = 0xB8,
  kSSMCmdECUInit = 0xBF,
};

enum SSMResponse {
  kSSMResponseSingle = 0x00,
  kSSMResponseStreamed = 0x01,
};

typedef struct {
  byte header;
  SSMDevice destination;
  SSMDevice source;
  byte dataSize;
  SSMCmd command;
  SSMResponse responseType;
  byte *data;
  byte checksum;
} SSMPacket;

byte SSMHeader = 0x80;
byte SSMAddressSize = 0x3;
byte SSMPacketDataOffset = 6;
byte SSMPacketHeaderSize = 7; // minus the data portion

enum SSMRequestState {
  SSMRequestInit,
  SSMRequestBlock1,
  SSMRequestBlock2,
};

typedef struct {
  SSMRequestState type;
  byte location;
  int length;  
} SSMRequest;

byte SSMMemory[304]; // 304 bytes should fit all the data
SSMRequestState requestState = SSMRequestInit;
SSMRequest block1Request = { SSMRequestBlock1, 0x7, 98 };
SSMRequest block2Request = { SSMRequestBlock2, 0xCE, 83 };

SoftwareSerial gSerialPort = SoftwareSerial(7, 6); // Rx, Tx
byte SoftwareSerialNoData = -1;
bool IsClearToSend = true;
int SerialReadDelayMS = 2;
unsigned long prvTime = 0;
unsigned long curTime = 0;

typedef struct {
  uint16_t canID;
  uint16_t data[8];
} SSMData;

SSMData priority0[] = {
  {
    0x259,// CAN ID 601
    0x0E, // Engine Speed 2 (RPM)
    0x0F, // Engine Speed 1 (RPM)
    0x46, // Air/Fuel Sensor #1
    0x15, // Throttle Opening Angle 
    0x0D, // Manifold Absolute Pressure
    0x10, // Vehicle Speed      
    0x08, // Coolant Temperature    
    0x4A, // Gear Position
  },
};

SSMData priority1[] = {
  {
    0x25A,// CAN ID 602
    0x11, // Ignition Timing      
    0x12, // Intake Air Temperature 
    0x13, // Mass Air Flow 1   
    0x14, // Mass Air Flow 2
    0x16, // Front O2 Sensor #1   
    0x17, // Front O2 Sensor #1 - 2
    0x18, // Rear O2 Sensor
    0x19, // Rear O2 Sensor - 2
  },
  {
    0x25B,// CAN ID
    0x07, // Engine Load    
    0x09, // Air/Fuel Correction #1 
    0x0A, // Air/Fuel Learning #1   
    0x0B, // Air/Fuel Correction #2 
    0x0C, // Air/Fuel Learning #2   
    0x1A, // Front O2 Sensor #2   
    0x1B, // Front O2 Sensor #2 - 2
    0x30, // Primary Wastegate Duty Cycle
  }
};

SSMData priority2[] =  {
  {
    0x25C, // CAN ID - 604
    0x23, // Atmospheric Pressure
    0x1C, // Battery Voltage
    0x1D, // Air Flow Sensor Voltage
    0x1E, // Throttle Sensor Voltage
    0x1F, // Diff Pres. Sens V.
    0x20, // Fuel Injection #1 Pulse W
    0x21, // Fuel Injection #2 Pulse W
    0x22, // Knock Correction
  },
  {
    0x25D, // CAN ID
    0x24, // Manifold Relative Pressure
    0x25, // Pressure Differential Sensor
    0x26, // Fuel Tank Pressure
    0x27, // CO Adjustment
    0x28, // Learned Ignition Timing
    0x29, // Accelerator Opening Angle
    0x2A, // Fuel Temperature
    0x2B  // Front O2 Heater #1
  },
  {
    0x25E, // CAN ID
    0x2C, // Rear O2 Heater Current
    0x2D, // Front O2 Heater #2
    0x2E, // Fuel Level
    0x31, // Secondary Wastegate Duty Cycle
    0x32, // CPC Valve Duty Ratio
    0x33, // Tumble Valve Position Sensor Right
    0xD3, // Air/Fuel Adjustment Voltage
  },
  {
    0x25F, // CAN ID
    0x34, // Tumble Valve Position Sensor Left
    0x35, // Idle Speed Control Valve Duty Ratio
    0x36, // Air/Fuel Lean Correction
    0x37, // Air/Fuel Heater Duty
    0x38, // Idle Speed Control Valve Step
    0x39, // Number of Ex. Gas Recirc Steps
    0x3A, // Alternator Duty
    0x3B, // Fuel Pump Duty
  },
  {
    0x260, // CAN ID
    0x3C, // VVT Advance Angle Right
    0x3D, // VVT Advance Angle Left
    0x3E, // OCV Duty Right
    0x3F, // OCV Duty Left
    0x40, // OCV Current Right
    0x41, // OCV Current Left
    0x42, // Air/Fuel Sensor #1 Current
    0x43, // Air/Fuel Sensor #2 Current
  },
  {
    0x261, // CAN ID
    0x44, // Air/Fuel Sensor #1 Resistance
    0x45, // Air/Fuel Sensor #2 Resistance
    0x47, // Air/Fuel Sensor #2
    0x53, // Air/Fuel Sensor #1 Heater Current
    0x54, // Air/Fuel Sensor #2 Heater Current
    0xD0, // Air/Fuel Correction #3
    0xD1, // Air/Fuel Learning #3
    0xD2, // Rear O2 Heater Voltage
  }
};

SSMData priority3[] = {
  {
    0x262, // CAN ID
    0xCE, // Roughness Monitor Cylinder #1
    0xCF, // Roughness Monitor Cylinder #2
    0xD8, // Roughness Monitor Cylinder #4
    0xD9, // Roughness Monitor Cylinder #3
    0xFA, // Throttle Motor Duty
    0xFB, // Throttle Motor Voltage
    0x120, // Switch - "ETC Motor Relay"
    0x121, // Switch - "Accel", "Brake", "Clutch", etc...
  },
  {
    0x263,  // CAN ID
    0x100, // Sub Throttle Sensor
    0x101, // Main Throttle Sensor
    0x102, // Sub Accelerator Sensor
    0x103, // Main Accelerator Sensor
    0x104, // Brake Booster Pressure
    0x105, // Fuel Pressure (High)
    0x106, // Exhaust Gas Temperature
    0x108, // Cold Start Injector
  },
  {
    0x264, // CAN ID
    0x61,
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
    0x67, // skip 0x68...
    0x69,
  },
};

typedef SSMData SSMData_Array[];

typedef struct {
  byte cycleCount;
  byte cycle;
  byte index;
  byte numIndexes;
} CanPriority;

#define NUM_CAN_PRIORITIES 4

CanPriority counterSchedule[NUM_CAN_PRIORITIES] = { 
   {0, 1, 0, 1},
   {0, 2, 0, 2},
   {0, 4, 0, 6},
   {0, 6, 0, 3}
};



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
  gSerialPort.begin(SSM_BUS_SPEED); //SSM uses 4800 8N1 baud rate
  do{
    delay(50);
  } while (!gSerialPort);

  DPRINTLN("Can Init");
  CAN.begin(CAN_BPS_500K);

  DPRINTLN("SSM Serial Line Established");
  DPRINTLN("SSM Initializing...");
  sendRequestForState(requestState);

  prvTime = millis();
}

// -----------------------------------------------
//  LOOOOOOOOP
// -----------------------------------------------
void loop()
{
  curTime = millis();
  if ((curTime - prvTime) > 1000) {
    gSerialPort.flush();
    sendRequestForState(requestState); // re-send the current request
    prvTime = curTime;
  }

  if (DEBUG_CAN)
    generateRandomSSMData();

  if (gSerialPort.available()) {    
    if (requestState = SSMRequestInit) {
      DPRINTLN("SSM Init finished, requesting block data...");
      requestState = SSMRequestBlock1;
    }
    else if (requestState == SSMRequestBlock1) {
      requestState == SSMRequestBlock2;
      SSMPacket *packet = readPacketFromSSMBus();
      if (packet) {
        memcpy(&(SSMMemory[block1Request.location]), packet->data, block1Request.length);
        free(packet);
      } else {
        DPRINTLN("Err: could not create packet from block 1");
      }
    }
    else if (requestState == SSMRequestBlock2) {
      requestState == SSMRequestBlock1;
      SSMPacket *packet = readPacketFromSSMBus();
      if (packet) {
        memcpy(&(SSMMemory[block2Request.location]), packet->data, block2Request.length);
        free(packet);
      } else {
        DPRINTLN("Err: could not create packet from block 2");
      }
    }
    
    prvTime = curTime;
  }

  for (int i=0; i<NUM_CAN_PRIORITIES; i++) {
    CanPriority *cp = &counterSchedule[i];
    cp->cycleCount++;
    if (cp->cycleCount >= cp->cycle) {
      cp->cycleCount = 0;
      cp->index++;
      if (cp->index >= cp->numIndexes) {
        cp->index = 0;

        switch(i) {
          case 0:
             sendCANPacketFor(&priority0[cp->index], &SSMMemory[0]);
             break;
          case 1:
             sendCANPacketFor(&priority1[cp->index], &SSMMemory[0]);
             break;
          case 2:
             sendCANPacketFor(&priority2[cp->index], &SSMMemory[0]);
             break;
          case 3:
             sendCANPacketFor(&priority3[cp->index], &SSMMemory[0]);
             break;
          default:
             DPRINT("Err: Unhandled priority: ");
             DPRINTLN(i);
             break;
        }
      }
    }
  }

  if (IsClearToSend && !DEBUG_CAN) {
    sendRequestForState(requestState);
    IsClearToSend = false;
  }
}

void generateRandomSSMData() {

#define MINRPM (750 * 4)
#define RPMINC (10*4)
#define MAXRPM (7500 *4)

  static uint16_t gRPM = (6201 * 4);
//  gRPM += RPMINC;
//  if (gRPM > MAXRPM)
//    gRPM = MINRPM;

  SSMMemory[0x0E] = (gRPM & 0xff);
  SSMMemory[0x0F] = (gRPM >> 8);

  // AFR - Divide value by 128.0 to get Lambda
  SSMMemory[0x46] = 110 + (rand() % 30);

  // IAT - Subtract 40 from value to get Degrees C
  SSMMemory[0x08] = 110 + (rand() % 30); 

  // TPS - Multiply value by 100.0 and divide by 255 to get percent
  SSMMemory[0x15] = rand() % 255;

  // MAP - Multiply value by 37.0 and divide by 255 to get psig
  // range of 0-22psi = 100 - 252
  SSMMemory[0x0D] = (100 + rand() % 150);

  // IAT - Subtract 40 from value to get Degrees C
  SSMMemory[0x12] = rand() % 255;

  // AFC - Subtract 128 from value and divide by 1.28 to get percent (air/fuel correction)
  SSMMemory[0x09] = rand() % 128;

  // Battery - Multiply value by 0.08 to get volts
  SSMMemory[0x1C] = 12 + rand() % 5;
  
  // Gear - 1 + to get gear
  SSMMemory[0x4A] = 1 + rand() % 4;

  // Speed - in kph
  static byte KPH = 0;
  KPH += 1;
  if (KPH > 160)
    KPH = 0;
  SSMMemory[0x10] = 100;
}

uint8_t sendCANPacketFor(SSMData *ssmData, byte *ssmMemory) {
  CAN_Frame standard_message; // Create message object to use CAN message structure
  standard_message.id = ssmData->canID;
  standard_message.valid = true;
  standard_message.rtr = 0;
  standard_message.extended = CAN_STANDARD_FRAME;
  standard_message.timeout = 100;
  standard_message.length = 8; // Data length

  // Fill in the frame with data from the SSM memory
  for(int i=0; i<8; i++) {    
    standard_message.data[i] = ssmMemory[ssmData->data[i]];
  }

  return CAN.write(standard_message); // Load message and send
}

void sendRequestForState(SSMRequestState state)
{
  switch (state) {
    case SSMRequestInit:
      sendSSMPacket(packetForSSMInit(), gSerialPort);
      break;
      
    case SSMRequestBlock1:
      sendSSMPacket(packetForBlockRead(block1Request.location, block1Request.length), gSerialPort);
      break;

    case SSMRequestBlock2:
      sendSSMPacket(packetForBlockRead(block2Request.location, block2Request.length), gSerialPort);
      break;     
  }
}

// -----------------------------------------------
//   Packet Builders
// -----------------------------------------------
SSMPacket *packetForSSMInit() {
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceECU;
  packet->source = kSSMDeviceDiagTool;
  packet->dataSize = SSMPacketHeaderSize;
  packet->command = kSSMCmdECUInit;
  packet->responseType = kSSMResponseSingle;
  packet->data = NULL;
  return packet;
}

SSMPacket *packetForNumberOfAddresses(SSMCmd command, byte numberOfBytes) {
  int addressRequestSize = SSMAddressSize * numberOfBytes;
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceECU;
  packet->source = kSSMDeviceDiagTool;
  packet->dataSize = SSMPacketHeaderSize + addressRequestSize;
  packet->command = command;
  packet->responseType = kSSMResponseSingle;
  packet->data = (byte *)malloc( addressRequestSize );

  for (byte x = 0; x < addressRequestSize; x++) {
     packet->data[x] = 0xFF;
  }
  
  return packet;
}

SSMPacket *packetForBlockRead(byte baseByte, byte numberOfBytes) {
  int addressRequestSize = SSMAddressSize * numberOfBytes;
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  packet->header = SSMHeader;
  packet->destination = kSSMDeviceECU;
  packet->source = kSSMDeviceDiagTool;
  packet->dataSize = SSMPacketHeaderSize + addressRequestSize;
  packet->command = kSSMCmdBlockRead;
  packet->responseType = kSSMResponseSingle;
  packet->data = (byte *)malloc( 4 );
  packet->data[0] = 0x00;
  packet->data[1] = 0x00;
  packet->data[2] = baseByte;
  packet->data[3] = numberOfBytes;
  return packet;
}


// -----------------------------------------------
//  Transport
// -----------------------------------------------

void sendSSMPacket(SSMPacket *packet, SoftwareSerial &digiSerial) {
  byte numDataBytes = (packet->dataSize - SSMPacketHeaderSize);  

  for (byte x = 0; x < SSMPacketDataOffset; x++) {
    digiSerial.write(((byte *)packet)[x]);
  }

  for (byte x = 0; x < numDataBytes; x++) {
    digiSerial.write(packet->data[x]);
  }

  byte checksum = 0;
  checksum += packet->header;
  checksum += packet->destination;
  checksum += packet->source;
  checksum += packet->dataSize;
  checksum += packet->command;
  checksum += packet->responseType;
  for(int i=0; i< (packet->dataSize - SSMPacketHeaderSize); i++) {
    checksum += packet->data[i];
  }

  digiSerial.write(checksum);

  free( packet->data );
  free( packet );
}

/* returns the 8 least significant bits of an input byte*/
byte computeChecksum(byte sum) {
  byte counter = 0;
  byte power = 1;
  for (byte n = 0; n < 8; n++) {
    counter += bitRead(sum, n) * power;
    power = power * 2;
  }
  return counter;
}

//
// This will change the values in dataArray, populating them with 
// values respective of the poll array address calls
//
SSMPacket *readPacketFromSSMBus()
{
  SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
  boolean isPacket = false;
  byte data = 0;
  byte sumBytes = 0;
  byte dataSize = 0;
  byte bytePlace = 0;
  byte loopLength = 20;

  for (byte j = 0; j < loopLength; j++)
  {
    data = gSerialPort.read();

    // Header marks the beginning of a packet
    if (data == SSMHeader && dataSize == 0) {
      isPacket = true;
      j = 0;
      DPRINTLN("Begin Packet");
      packet->header = data;
    }

    // terminate function and return false if no response is detected
    if (!isPacket && (j == (loopLength - 1))) {
      DPRINTLN("no data");
      free(packet);
      return NULL;
    }

    if (!isPacket || data == SoftwareSerialNoData) {
      delay(SerialReadDelayMS);
      continue;
    }
    
    DPRINT(data); // for debugging: shows in-packet data
    DPRINT(" ");

    ((byte *)packet)[bytePlace] = data;

    if (bytePlace == 3) { // how much data is coming
      dataSize = data;
      loopLength = dataSize + SSMPacketDataOffset;

      packet->data = (byte *)malloc( dataSize );
    }

    if (bytePlace > 4 && bytePlace - 5 < dataSize)
    {
      packet->data[bytePlace - 5] = data;
    }

    // increment bytePlace
    bytePlace += 1;

    // Once the data is all received, checksum and re-set counters
    DPRINT("byte place: ");
    DPRINTLN(bytePlace);
      
    if (bytePlace == dataSize + 5) {
      //the 8 least significant bits of sumBytes
      if (data != computeChecksum(sumBytes)) {
        DPRINTLN(F("checksum error"));
        free( packet->data );
        free( packet );
        return NULL;
      }

      DPRINTLN("Checksum is good");

      IsClearToSend = true;
      return packet;
    }
    else
    {
      sumBytes += data; // this is to compare with the checksum byte
      //DPRINT(F("sum: "));
      //DPRINTLN(sumBytes);
    }
  }
}




