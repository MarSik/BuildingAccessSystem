
#ifndef ULTRALIGHT_C_H
#define ULTRALIGHT_C_H

#include "PN532.h"
#include "DES.h"
#include "Buffer.h"

// Just an invalid key number
#define NOT_AUTHENTICATED      255

#define UC_MAX_FRAME_SIZE         60 // The maximum total length of a packet that is transfered to / from the card

// -------- Ultralight C instructions ----------

#define UC_AUTHENTICATE           0x1A // 1A 00 [crc] -> AF [RndB:8B] [crc] or [nak]
#define UC_AUTHENTICATE_RESPONSE  0xAF

#define UC_WRITE 0xA2 // A2 [page] AB CD EF GH [crc] -> [ack/nak] - byte 0 first (LSB)
#define UC_READ 0x30 // 30 [page] [crc] -> [data:16B] [crc] or [nak] - reads four pages, skips keys (rolls over to 0x00)

#define UC_ACK 0xA // anything else is NAK even when the code is unknown
#define UC_NAK_EEPROM 0x2
#define UC_NAK_CRC 0x1
#define UC_NAK 0X0

#define UC_PAGE_KEY1_LSB 0x2C // byte 0 LSB
#define UC_PAGE_KEY1_MSB 0x2D // byte 3 MSB
#define UC_PAGE_KEY2_LSB 0x2E // byte 0 LSB
#define UC_PAGE_KEY2_MSB 0x2F // byte 3 MSB

#define UC_PAGE_LOCK12 0x02 // byte 2 and 3 - locks pages 0x3 - 0xF
#define UC_PAGE_LOCK23 0x28 // only byte 0 and 1 - locks pages 0x10 - end

#define UC_PAGE_OTP 0x03 // writes are ORred (default value 0, can only set 1s)

#define UC_PAGE_COUNT 0x29 // only bytes 0 and 1 -- initial set allows anything, additional writes only allow 0x01 - 0x0F in byte0, the value will be added to the counter, byte0 is LSB and byte1 MSB when reading

#define UC_PAGE_AUTH1 0x2A // authentication configuration
#define UC_PAGE_AUTH2 0x2B

#define UC_PAGE_UID0 0x00 // byte 0 = manufacturer; byte 1 and 2 = SN1,2; byte 3 = BCC0 = Ox88 xor MANUF xor SN1 xor SN2
#define UC_PAGE_UID1 0x01 // SN3,4,5,6
#define UC_PAGE_UID_BCC1 0x02 // byte 0 = BCC1 = SN3 xor SN4 xor SN5 xor SN6

#define UC_PAGE_USER_MIN 0x04
#define UC_PAGE_USER_COMPAT_MAX 0x0F // last page available in Ultralight tags
#define UC_PAGE_USER_MAX 0x27

class UltralightC : public PN532
{
 public:
    UltralightC();

    bool Authenticate (DESFireKey* pi_Key);
    bool ChangeKey    (DESFireKey* pi_NewKey, DESFireKey* pi_CurKey);

    bool ReadPageData     (byte u8_PageID, uint32_t[4] data);
    bool WritePageData    (byte u8_PageID, uint32_t data);

    bool SwitchOffRfField();  // overrides PN532::SwitchOffRfField()
    byte GetLastPN532Error(); // See comment for this function in CPP file

    DES  DES3_DEFAULT_KEY; // 3K3DES key with 24 zeroes 

 private:
    bool Communicate(byte command, byte argc, byte* argv, byte respsize, byte* respv);    

    bool          mb_authenticated;
    byte          mu8_LastPN532Error;
};

#endif

