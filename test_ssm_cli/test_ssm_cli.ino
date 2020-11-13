#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "lilParser.h"
#include <SSMCAN.h>

// Debug logging
#if 1
  #define DPRINT(...)    Serial.print(__VA_ARGS__) 
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

#define SERIAL_DIAG_SPEED 115200
#define ECU_IDENTITY_SIZE 5
#define ECU_CAPABILITIES_SIZE 50

lilParser cmdParser;
enum commands {   noCommand,  // ALWAYS start with noCommand. Or something simlar.
                  ecuInit,    // The rest is up to you. help would be a good one. Have it list
                  addressRead,  // What the other commands are, how they work and what they do.
                  blockRead
                  };          // Our list of commands.


SoftwareSerial gSerialPort = SoftwareSerial(7, 6); // Rx, Tx

// -----------------------------------------------
//  SETUP
// -----------------------------------------------
void setup() {
  DPRINTLN("Connecting to Serial...");
  Serial.begin(SERIAL_DIAG_SPEED); //for diagnostics
  while (!Serial) {
    delay(50);
  }

  cmdParser.addCmd(ecuInit,"ecuinit"); 
  cmdParser.addCmd(addressRead,"lsaddr");
  cmdParser.addCmd(blockRead,"lsblock"); 
  
  DPRINTLN("Initializing SSM serial port...");
  gSerialPort.begin(SSM_BUS_SPEED); //SSM uses 4800 8N1 baud rate
  do{
    delay(50);
  } while (!gSerialPort);
  DPRINTLN("SSM Serial Line Established.");
}

// -----------------------------------------------
//  COMMANDS
// -----------------------------------------------
void doECUInit() {
  sendSSMPacket(packetForSSMInit(), gSerialPort);
  SSMPacket *packet = readPacketFromSSMBus(gSerialPort);
  if (!packet) {
    DPRINTLN("No response.");
    return;
  }
  logPacket(packet);

  DPRINT("ECU ID: ");
  for (int i=0; i< ECU_IDENTITY_SIZE ; i++) {
    DPRINT(packet->data[1+i], HEX);
  }
  DPRINTLN(" ");

  DPRINT("ECU CAPABILITIES: ");
  for (int i=ECU_IDENTITY_SIZE; i< (ECU_CAPABILITIES_SIZE + ECU_IDENTITY_SIZE) ; i++) {
    DPRINT(packet->data[1+i], HEX);
  }
  DPRINTLN(" ");
 }

void doAddressRead() {
  char* charBuff;                                 // A pointer for the folder name string.
   
  if (cmdParser.numParams() == 1) {               // If they typed in somethng past the command.
    charBuff = cmdParser.getParamBuff();         // We get the first parameter, assume its the new folder's name.
    unsigned long address = strtol(charBuff, 0, 16);
    free(charBuff);

#if 0
    DPRINT("Address: 0x");
    DPRINTLN(address, HEX);
#endif
    
    sendSSMPacket(packetForAddressRead(&address, 1), gSerialPort);
    SSMPacket *packet = readPacketFromSSMBus(gSerialPort);
    if (!packet) {
      return;
    }
    logPacket(packet);
   } else {
    DPRINTLN("Err: lsaddr command format is 'lsaddr 0x{XXXXXX}' ");
   }   
}

void doBlockRead() {
  char* charBuff;                                 // A pointer for the folder name string.
   
  if (cmdParser.numParams() == 2) {               // If they typed in somethng past the command.
    charBuff = cmdParser.getParamBuff();         // We get the first parameter, assume its the new folder's name.
    char *addr = strtok(charBuff, " ");
    unsigned long address = strtol(addr, 0, 16);
    char *lenStr = strtok(NULL, " ");
    byte readLen = strtol(lenStr, 0, 10);

#if 0
    DPRINT("Address: 0x");
    DPRINTLN(address, HEX);
    DPRINT("Length: ");
    DPRINTLN(readLen);
#endif

    free(charBuff);

    sendSSMPacket(packetForBlockRead(address, readLen), gSerialPort);
    SSMPacket *packet = readPacketFromSSMBus(gSerialPort);
    if (!packet) {
      return;
    }
    logPacket(packet);
   } else {
    DPRINTLN("Err: lsblock command format is 'lsblock 0x{XXXXXX} {length}' ");
   }  
}

void loop() {
   char  inChar;
   int   command;
   if (Serial.available()) {
      inChar = Serial.read();
      //Serial.print(inChar); 
      command = cmdParser.addChar(inChar);
      switch (command) {
         case noCommand   : break; 
         case ecuInit     : doECUInit(); break;
         case addressRead : doAddressRead(); break;
         case blockRead   : doBlockRead(); break;
         default          : Serial.println("What?"); break;
      }
   }
}
