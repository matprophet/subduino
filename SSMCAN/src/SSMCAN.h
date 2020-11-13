#ifndef _SSMCAN_H
#define _SSMCAN_H
#include <Arduino.h>
#include <SoftwareSerial.h>

#define SSM_BUS_SPEED         4800

// -----------------------------------------------
//  SSM Protocol
// -----------------------------------------------
typedef struct {
    byte header;
    byte destination;
    byte source;
    byte dataSize;
    byte *data;
    byte checksum;
} SSMPacket;

enum SSMDevice : unsigned int  {
    kSSMDeviceECU = 0x10,
    kSSMDeviceTCU = 0x18,
    kSSMDeviceDiagTool = 0xF0,
};

enum SSMCommand : unsigned int  {
    kSSMCommandBlockRead = 0xA0, // Some ECUs do not support BlockRead commands
    kSSMCommandAddressRead = 0xA8,
    kSSMCommandBlockWrite = 0xB0,
    kSSMCommandAddressWrite = 0xB8,
    kSSMCommandECUInit = 0xBF,
};

enum SSMResponse : unsigned int  {
    kSSMResponseBlockRead = 0xE0,
    kSSMResponseAddressRead = 0xE8,
    kSSMResponseBlockWrite = 0xF0,
    kSSMResponseAddressWrite = 0xF8,
    kSSMResponseECUInit = 0xFF,
};

enum SSMResponseType : unsigned int {
    kSSMResponseTypeSingle = 0x00,
    kSSMResponseTypeStreamed = 0x01,
};

extern unsigned int SSMHeader;
extern unsigned int SSMAddressSize;
extern unsigned int SSMPacketDataOffset;

extern SSMPacket *packetForSSMInit();
extern SSMPacket *packetForBlockRead(unsigned long address, byte numberOfBytes);
extern SSMPacket *packetForAddressRead(unsigned long *addresses, byte numberOfAddresses);
extern void logPacket(SSMPacket *packet);
extern void freePacket(SSMPacket *packet);

extern SSMPacket *readPacketFromSSMBus(SoftwareSerial &serial);
extern void sendSSMPacket(SSMPacket *packet, SoftwareSerial &serial);

#endif
