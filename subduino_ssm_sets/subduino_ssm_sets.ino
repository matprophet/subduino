#include <Arduino.h>
#include <SPI.h>
#include <SSMCAN.h>

// CAN support using https://github.com/McNeight/CAN_Library
#include <CAN.h>
#include <SPI.h>
#include <CAN_MCP2515.h>

// Global Settings
#define SERIAL_DIAG_SPEED     115200
#define SSM_REQUEST_INTERVAL  1000
#define OIL_TEMP_PIN          1
#define OIL_PRESSURE_PIN      3
#define FUEL_LEVEL_PIN        4

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
//  Subduino (SD) custom code
// -----------------------------------------------
enum SDRequestState : byte {
  SDRequestStateInit,
  SDRequestStateSet1,
};

byte gSDMemory[304]; // 304 bytes should fit all the data
SDRequestState gSDRequestState = SDRequestStateInit;

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

typedef struct {
  uint16_t canID;
  uint16_t data[8];
} SDData;
typedef SDData SDData_Array[];

SDData gSDDataPriority0[] = {
  {
    0x259,// CAN ID 601
    0x0F, // Engine Speed 2 (RPM) low byte
    0x0E, // Engine Speed 1 (RPM) high byte
    0x46, // Air/Fuel Sensor #1
    0x15, // Throttle Opening Angle 
    0x0D, // Manifold Absolute Pressure
    0x10, // Vehicle Speed      
    0x08, // Coolant Temperature    
    0x4A, // Gear Position
  },
};

SDData gSDDataPriority1[] = {
  {
    0x25A,// CAN ID 602
    0x11, // Ignition Timing      
    0x12, // Intake Air Temperature 
    0x13, // Mass Air Flow 1   
    0x14, // Mass Air Flow 2
    0x16, // Front O2 Sensor #1   
    0x17, // Front O2 Sensor #1 - 2
    0x18, // Rear O2 Sensor
    0x19, // Oil pressure (psi)
  },
  {
    0x25B,// CAN ID 603
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

SDData gSDDataPriority2[] =  {
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
    0x25D, // CAN ID - 605
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
    0x25E, // CAN ID - 606
    0x2C, // Rear O2 Heater Current
    0x2D, // Front O2 Heater #2
    0x2E, // Fuel Level (%)
    0x31, // Secondary Wastegate Duty Cycle
    0x32, // CPC Valve Duty Ratio
    0x33, // Tumble Valve Position Sensor Right
    0xD3, // Air/Fuel Adjustment Voltage
  },
  {
    0x25F, // CAN ID - 607
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
    0x260, // CAN ID - 608
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
    0x261, // CAN ID - 609
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

SDData gSDDataPriority3[] = {
  {
    0x262, // CAN ID - 610
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
    0x263,  // CAN ID - 611
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
    0x264, // CAN ID - 612
    0x61,  // Oil Temp (C)
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
    0x67, // skip 0x68...
    0x69,
  },
};

typedef struct {
  byte cycleCount;
  byte cycle;
  byte index;
  byte numIndexes;
} CanPriority;

#define NUM_CAN_PRIORITIES 4

// cycleCount | cycle to trigger on | current index | numIndexes // 
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
  DPRINTLN("SSM Serial Line Established.");

  DPRINTLN("Initializing CAN bus...");
  CAN.begin(CAN_BPS_500K);
  DPRINTLN("CAN Bus Initialized.");

  gTimeLast = millis();
}

// -----------------------------------------------
//  LOOP
// -----------------------------------------------
void loop()
{
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

  readAnalogPinsIntoSDMemory();
  sendCANPacketsFromSDMemory();
  
  if (gIsClearToSend) {
    sendSSMCommandForState(gSDRequestState);
    gIsClearToSend = false;
  }
}

void sendCANPacketsFromSDMemory() {
  for (int i=0; i<NUM_CAN_PRIORITIES; i++) {
    CanPriority *cp = &counterSchedule[i];
    cp->cycleCount++;
    if (cp->cycleCount >= cp->cycle) {
      cp->cycleCount = 0;

      cp->index++;
      if (cp->index >= cp->numIndexes)
        cp->index = 0;

      switch(i) {
        case 0:
           sendCANPacketFor(&gSDDataPriority0[cp->index], &gSDMemory[0]);
           break;
        case 1:
           sendCANPacketFor(&gSDDataPriority1[cp->index], &gSDMemory[0]);
           break;
        case 2:
           sendCANPacketFor(&gSDDataPriority2[cp->index], &gSDMemory[0]);
           break;
        case 3:
           sendCANPacketFor(&gSDDataPriority3[cp->index], &gSDMemory[0]);
           break;
        default:
           DPRINT("Err: Unhandled priority: ");
           DPRINTLN(i);
           break;
      }
    }
  }
}

uint8_t sendCANPacketFor(SDData *ssmData, byte *memory) {
  CAN_Frame standard_message; // Create message object to use CAN message structure
  standard_message.id = ssmData->canID;
  standard_message.valid = true;
  standard_message.rtr = 0;
  standard_message.extended = CAN_STANDARD_FRAME;
  standard_message.timeout = 100;
  standard_message.length = 8; // Data length

  // Fill in the frame with data from the SSM memory
  for(int i=0; i<8; i++) {    
    standard_message.data[i] = memory[ssmData->data[i]];
  }

  return CAN.write(standard_message); // Load message and send;
}

void sendSSMCommandForState(SDRequestState inState) {
  switch (inState) {
    case SDRequestStateInit:
      DPRINTLN("Requesting SSM init...");
      sendSSMPacket(packetForSSMInit(), gSerialPort);
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
      gSDRequestState = SDRequestStateSet1;
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

void readAnalogPinsIntoSDMemory() {  
  // 0 @ 0bar, 512 @ 10bar
  int oilPressure = analogRead(OIL_PRESSURE_PIN);
  if (oilPressure) {
    gSDMemory[0x19] = (byte)((((163840.0 / oilPressure ) - 160) / 160.0) * 145.0);  // 0-10bar converted to psi
  } else
    gSDMemory[0x19] = 0;
  
  // Oil Temp Thermistor
  int oilTemp = analogRead(OIL_TEMP_PIN);
  if (oilTemp) {
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    float R1 = 9200;
    float R2 = R1 * (1023.0 / (float)oilTemp - 1.0);
    float logR2 = log(R2);
    float TempC = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    gSDMemory[0x61] = (byte)(TempC - 273.15); // Celsius
  }
  else {
    gSDMemory[0x61] = 0;
  }
  
  // 0 @ full tank, 512 @ empty tank
  int fuelLevel = analogRead(FUEL_LEVEL_PIN);
  if (fuelLevel)
    gSDMemory[0x2E] = (byte)(100.0 * ((fuelLevel - 512)/512.0));
  else
    gSDMemory[0x2E] = 0;

#if 0
  DPRINT( "oilPressure pin: " ); DPRINT( oilPressure ); DPRINT( " calc: " ); DPRINTLN( gSDMemory[0x19] );
  DPRINT( "oilTemp     pin: " ); DPRINT( oilTemp );     DPRINT( " calc: " ); DPRINTLN( gSDMemory[0x61] );
  DPRINT( "fuelLevel   pin: " ); DPRINT( fuelLevel );   DPRINT( " calc: " ); DPRINTLN( gSDMemory[0x2E] );
#endif
}
