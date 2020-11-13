#include <SSMCAN.h>
#include <SoftwareSerial.h>

#define SERIAL_READ_DELAY_MS  20

unsigned int SSMHeader = 0x80;
unsigned int SSMAddressSize = 0x3;
unsigned int SSMPacketDataOffset = 4;

// -----------------------------------------------
//   Packet Builders
// -----------------------------------------------
SSMPacket *packetForSSMInit() {
    SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
    packet->header = SSMHeader;
    packet->destination = kSSMDeviceECU;
    packet->source = kSSMDeviceDiagTool;
    packet->dataSize = 1;
    packet->data = (byte *)malloc( packet->dataSize );
    packet->data[0] = kSSMCommandECUInit;
    return packet;
}

SSMPacket *packetForBlockRead(unsigned long address, byte numberOfBytes) {
    SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
    packet->header = SSMHeader;
    packet->destination = kSSMDeviceECU;
    packet->source = kSSMDeviceDiagTool;
    packet->dataSize = 6;
    packet->data = (byte *)malloc( packet->dataSize );
    packet->data[0] = kSSMCommandBlockRead;
    packet->data[1] = kSSMResponseTypeSingle;
    packet->data[2] = (byte)(0xFF & (address >> 16));
    packet->data[3] = (byte)(0xFF & (address >> 8));
    packet->data[4] = (byte)(0xFF & address);
    packet->data[5] = (numberOfBytes-1);
    return packet;
}

SSMPacket *packetForAddressRead(unsigned long *addresses, byte numberOfAddresses) {
    int addressRequestSize = SSMAddressSize * numberOfAddresses;
    SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
    packet->header = SSMHeader;
    packet->destination = kSSMDeviceECU;
    packet->source = kSSMDeviceDiagTool;
    packet->dataSize = (2 + addressRequestSize);
    packet->data = (byte *)malloc( packet->dataSize );
    packet->data[0] = kSSMCommandAddressRead;
    packet->data[1] = kSSMResponseTypeSingle;
    
    for (byte i=0; i<numberOfAddresses; i++){
        byte offset = 2 + (i*3);
        unsigned long address = addresses[i];
        packet->data[offset] = (byte)(0xFF & (address >> 16));
        packet->data[offset+1] = (byte)(0xFF & (address >> 8));
        packet->data[offset+2] = (byte)(0xFF & address);
    }
    
    return packet;
}

void logPacket(SSMPacket *packet) {
    Serial.println(" ------- PACKET START ------");
    Serial.print(" header:      0x"); Serial.println(packet->header, HEX);
    Serial.print(" destination: 0x"); Serial.println(packet->destination, HEX);
    Serial.print(" source:      0x"); Serial.println(packet->source, HEX);
    Serial.print(" data size:   "); Serial.println(packet->dataSize);
    
    if (packet->data) {
        for (int i=0; i< packet->dataSize ; i++) {
            Serial.print(" data[");
            Serial.print(i);
            Serial.print("]:    0x");
            Serial.println( packet->data[i], HEX);
        }
        Serial.print(" checksum:     0x"); Serial.println(packet->checksum, HEX);
    }
    else {
        Serial.println(" ### no data ###");
    }
    Serial.println(" ------- PACKET END   ------");
}

void freePacket(SSMPacket *packet) {
    if (packet->data)
        free(packet->data);
    if (packet)
        free(packet);
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


SSMPacket *readPacketFromSSMBus(SoftwareSerial &serial) {
    SSMPacket *packet = (SSMPacket *)malloc( sizeof(SSMPacket) );
    packet->data = NULL;
    boolean isPacket = false;
    
    byte data = 0;
    byte *dataArray;
    int numReadExpected;
    int numReadActual;
    
    byte sumBytes = 0;
    int dataSize = 0;
    byte bytePlace = 0;
    byte loopLength = 20;
    
    numReadExpected = 1;
    
    for (byte j = 0; j < loopLength; j++) {
        if (!serial.available()) {
            delay(SERIAL_READ_DELAY_MS);
            continue;
        }
        
        numReadActual = serial.readBytes(&data, numReadExpected);
        
        // Header marks the beginning of a packet
        if (data == SSMHeader && dataSize == 0) {
            isPacket = true;
            j = 0;
        }
        
        // Terminate if no response is detected
        if (!isPacket && (j == (loopLength - 1))) {
            break;
        }
        
        if (!isPacket || numReadActual == 0) {
            delay(SERIAL_READ_DELAY_MS);
            continue;
        }
        
        if (bytePlace == 0) {
            packet->header = data;
        }
        else if (bytePlace == 1) {
            packet->destination = data;
        }
        else if (bytePlace == 2) {
            packet->source = data;
        }
        else if (bytePlace == 3) { // how much data is coming
            dataSize = data;
            packet->dataSize = dataSize;
            packet->data = (byte *)malloc( packet->dataSize );
            loopLength += dataSize;
        }
        else if (bytePlace > 3 && bytePlace - 4 < dataSize) {
            packet->data[bytePlace - 4] = data;
        }
        else if (bytePlace == (4 + dataSize)) {
            // the 8 least significant bits of sumBytes
            if (data == computeChecksum(sumBytes)) {
                return packet;
            }
            else {
                break;
            }
        }
        
        sumBytes += data; // this is to compare with the checksum byte
        bytePlace++;
    }
    
    freePacket(packet);
    return NULL;
}

void sendSSMPacket(SSMPacket *packet, SoftwareSerial &ss) {
    byte checksum = 0;
    for (byte i = 0; i < SSMPacketDataOffset; i++) {
        byte value = ((byte *)packet)[i];
        checksum += value;
        ss.write(value);
    }
    
    for (byte i = 0; i < packet->dataSize; i++) {
        byte value = packet->data[i];
        checksum += value;
        ss.write(value);
    }
    ss.write(checksum);
    
    packet->checksum = checksum;
    logPacket(packet);
    
    freePacket(packet);
}

