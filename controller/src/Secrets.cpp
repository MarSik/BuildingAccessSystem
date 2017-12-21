#include <Arduino.h>
#include <driverlib/eeprom.h>
#include "Secrets.h"

// This reader is used to open door group no. 1
byte DOOR_ID = 1;

// This 2K3DES key is the default key for pristine Ultralight C cards stored on the card as BREAKMEIFYOUCAN!
const byte DEFAULT_UL_KEY[16] = {'I', 'E', 'M', 'K', 'A', 'E', 'R', 'B', '!', 'N', 'A', 'C', 'U', 'O', 'Y', 'F'};

// This 2K3DES key is used to derive a 16 byte application master key from the UID of the card and the user name.
// The purpose is that each card will have it's unique application master key that can be calculated from known values.
byte APPLICATION_KEY[16] = {'I', 'E', 'M', 'K', 'A', 'E', 'R', 'B', '!', 'N', 'A', 'C', 'U', 'O', 'Y', 'F'};

// -----------------------------------------------------------------------------------------------------------

// The ID of the application, it will be stored on the Ultralight C card
// to mark the card as part of the system
// This value must be between 0x000001 and 0xFFFFFF (NOT zero!)
// BCD decimal, 4 * 5 bit char (ASCII - 65), 2* BCD decimal
// 0OPLK18 = 0x0 79 80 76 75 0x18
//   = (after - 65) 0x0 14 15 11 10 0x18
//   = 0000 01110 01111 01011 01010 0001 1000
//   = 0000 0111 0011 1101 0110 1010 0001 1000
//   = Ox073D6A18
const uint32_t APPLICATION_ID = 0x073D6A18; // 0OPLK18

void loadApplicationKeyFromEEPROM() {
    uint32_t candidate[5];
    EEPROMRead(candidate, 0, 20);
    if ((0x55555555 ^ candidate[0] ^ candidate[1] ^ candidate[2] ^ candidate[3]) == candidate[4]) {
        Serial.write("Successfully restored application key.\r\n");
        memcpy(APPLICATION_KEY, candidate, 16);
    } else {
        Serial.write("Invalid application key - using the default one!!!\r\n");
    }
}

void setApplicationKey(const byte newKey[16]) {
    uint32_t candidate[5];
    memcpy(candidate, newKey, 16);
    candidate[4] = 0x55555555 ^ candidate[0] ^ candidate[1] ^ candidate[2] ^ candidate[3];
    if (!EEPROMProgram(candidate, 0, 20)) {
        Serial.write("New application key set.\r\n");
        memcpy(APPLICATION_KEY, newKey, 16);
    } else {
        Serial.write("Failed to set new application key.\r\n");
    }
}

void clearApplicationKey() {
    uint32_t candidate[5] = {0, 0, 0, 0, 0};
    if (!EEPROMProgram(candidate, 0, 5)) {
        Serial.write("Application key cleared.\r\n");
        memcpy(APPLICATION_KEY, DEFAULT_UL_KEY, 16);
    } else {
        Serial.write("Failed to clear the application key.\r\n");
    }
}
