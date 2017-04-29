/**************************************************************************

    @author   Elmü
    This file contains secret values for encryption.

**************************************************************************/

#ifndef SECRETS_H
#define SECRETS_H

#include <Arduino.h>

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

#define CARD_PAGE_APPID 0x04
#define CARD_PAGE_DOOR1 0x05 // Format 1b: 0 allow on match, 1 deny on match; 7b days of week, 8b - bitset for 3h intervals, 16b reserved
#define CARD_PAGE_DOOR2 0x06
#define CARD_PAGE_DOOR3 0x07
#define CARD_PAGE_DOOR4 0x08

#endif
