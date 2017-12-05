#ifndef MFRC522_ULTRALIGHT_H
#define MFRC522_ULTRALIGHT_H

#include "MFRC522Mifare.h"

template < class T > class MFRC522Ultralight:public MFRC522Mifare < T > {
public:
  MFRC522Ultralight(T & intf, const byte powerEnablePin) : MFRC522Mifare<T>(intf, powerEnablePin) {}

  /**
   * Writes a 4 byte page to the active MIFARE Ultralight PICC.
   *
   * @return STATUS_OK on success, STATUS_??? otherwise.
   */
    StatusCode MIFARE_Ultralight_Write(const byte page,	///< The page (2-15) to write to.
                                     const byte * const buffer,	///< The 4 bytes to write to the PICC
                                     const byte bufferSize	///< Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
      )
    {
      StatusCode result;

      // Sanity check
      if (buffer == NULL || bufferSize < 4) {
        return STATUS_INVALID;
      }
      // Build commmand buffer
      byte cmdBuffer[6];
      cmdBuffer[0] = PICC_CMD_UL_WRITE;
      cmdBuffer[1] = page;
      memcpy(&cmdBuffer[2], buffer, 4);

      // Perform the write
      result = this->PCD_MIFARE_Transceive(cmdBuffer, 6);	// Adds CRC_A and checks that the response is MF_ACK.
      if (result != STATUS_OK) {
        return result;
      }
      return STATUS_OK;
    }				// End MIFARE_Ultralight_Write()

    // Configure the autnentication based memory protection
    StatusCode UltralightC_SetAuthProtection(const byte firstPage, const boolean protectRead = false)
    {
       byte buffer[4] = {0x00, 0x00, 0x00, 0X00};
       buffer[0] = protectRead ? 0x00 : 0X01;
       StatusCode result = MIFARE_Ultralight_Write(UL_AUTH1, buffer, 4);
       if (result != STATUS_OK) return result;

       buffer[0] = firstPage;
       result = MIFARE_Ultralight_Write(UL_AUTH0, buffer, 4);
       return result;
    }

    StatusCode UltralightC_ChangeKey(const byte key[16]) {
      CREATE_BUFFER(buff, 4);

      ADD_BUFFER(buff, key[0x7]);
      ADD_BUFFER(buff, key[0x6]);
      ADD_BUFFER(buff, key[0x5]);
      ADD_BUFFER(buff, key[0x4]);
      StatusCode result = MIFARE_Ultralight_Write(UL_3DES1_LSB, BUFFER(buff), 4);

      BUFFER_CLEAR(buff);
      ADD_BUFFER(buff, key[0x3]);
      ADD_BUFFER(buff, key[0x2]);
      ADD_BUFFER(buff, key[0x1]);
      ADD_BUFFER(buff, key[0x0]);
      result = MIFARE_Ultralight_Write(UL_3DES1_MSB, BUFFER(buff), 4);

      BUFFER_CLEAR(buff);
      ADD_BUFFER(buff, key[0xF]);
      ADD_BUFFER(buff, key[0xE]);
      ADD_BUFFER(buff, key[0xD]);
      ADD_BUFFER(buff, key[0xC]);
      result = MIFARE_Ultralight_Write(UL_3DES2_LSB, BUFFER(buff), 4);

      BUFFER_CLEAR(buff);
      ADD_BUFFER(buff, key[0xB]);
      ADD_BUFFER(buff, key[0xA]);
      ADD_BUFFER(buff, key[0x9]);
      ADD_BUFFER(buff, key[0x8]);
      result = MIFARE_Ultralight_Write(UL_3DES2_MSB, BUFFER(buff), 4);

      return result;
    }

    bool UltralightC_Authenticate(const byte key[16]) {
      CREATE_BUFFER(command, 35);

      ADD_BUFFER(command, PICC_CMD_UL_AUTHENTICATE);
      ADD_BUFFER(command, 0x00);
      byte responseSize = BUFFER_SIZE(command);

      StatusCode result = this->PCD_Mifare_TransceiveWithReply(BUFFER(command), BUFFER_SIZE(command), BUFFER_LEN(command), BUFFER(command), &responseSize);
      if (result != STATUS_OK || *(BUFFER(command)) != PICC_CMD_UL_AUTHENTICATE_RESPONSE)
      {
          MFRC522Logger.println(ERROR, "Authentication failed (1)");
          return false;
      }

      const int s32_RandomSize = 8;

      byte RndB[8];  // decrypted random B

      mbedtls_des3_context tdes_ctx;
      mbedtls_des3_init(&tdes_ctx);

      // Fill IV with zeroes !ONLY ONCE HERE!
      byte iv[8] = {0, 0, 0, 0, 0, 0, 0, 0};

      // decrypt command[1:17] using CBC_RECEIVE to RndB
      mbedtls_des3_set2key_dec(&tdes_ctx, key);
      mbedtls_des3_crypt_cbc(&tdes_ctx, MBEDTLS_DES_DECRYPT, s32_RandomSize, iv, BUFFER(command) + 1, RndB);

      byte RndB_rot[8]; // rotated random B
      rotate_left(RndB_rot, RndB, s32_RandomSize);

      byte RndA[8];
      generate_random(RndA, s32_RandomSize);

      CREATE_BUFFER(i_RndAB, 16); // (randomA + rotated randomB)
      ADD_BUFFER_PTR(i_RndAB, RndA,     s32_RandomSize);
      ADD_BUFFER_PTR(i_RndAB, RndB_rot, s32_RandomSize);

      CREATE_BUFFER(i_RndAB_enc, 16); // encrypted (randomA + rotated randomB)
      SET_BUFFER_LEN(i_RndAB_enc, 2*s32_RandomSize);

      // encrypt i_RndAB[:32] to i_RndAB_enc, CBC_SEND
      mbedtls_des3_set2key_enc(&tdes_ctx, key);
      mbedtls_des3_crypt_cbc(&tdes_ctx, MBEDTLS_DES_ENCRYPT, BUFFER_SIZE(i_RndAB), iv, BUFFER(i_RndAB), BUFFER(i_RndAB_enc));

      MFRC522Logger.write(DEBUG, "* RndB_enc:  ");
      MFRC522Logger.println(DEBUG, BUFFER(command) + 1, s32_RandomSize, HEX);
      MFRC522Logger.write(DEBUG, "* RndB:      ");
      MFRC522Logger.println(DEBUG, RndB, s32_RandomSize, HEX);
      MFRC522Logger.write(DEBUG, "* RndB_rot:  ");
      MFRC522Logger.println(DEBUG, RndB_rot, s32_RandomSize, HEX);
      MFRC522Logger.write(DEBUG, "* RndA:      ");
      MFRC522Logger.println(DEBUG, RndA, s32_RandomSize, HEX);
      MFRC522Logger.write(DEBUG, "* RndAB:     ");
      MFRC522Logger.println(DEBUG, BUFFER(i_RndAB), BUFFER_LEN(i_RndAB), HEX);
      MFRC522Logger.write(DEBUG, "* RndAB_enc: ");
      MFRC522Logger.println(DEBUG, BUFFER(i_RndAB_enc), BUFFER_LEN(i_RndAB_enc), HEX);

      responseSize = BUFFER_SIZE(command);
      BUFFER_CLEAR(command);
      ADD_BUFFER(command, PICC_CMD_UL_AUTHENTICATE_RESPONSE);
      ADD_BUFFER_PTR(command, BUFFER(i_RndAB_enc), BUFFER_LEN(i_RndAB_enc));

      result = this->PCD_Mifare_TransceiveWithReply(BUFFER(command), BUFFER_SIZE(command), BUFFER_LEN(command), BUFFER(command), &responseSize);
      if (result != STATUS_OK || *BUFFER(command) != 0x00)
      {
        MFRC522Logger.println(ERROR, "Authentication failed (2)");
          return false;
      }

      // decrypt command[1:17] to RndA_enc, CBC_RECEIVE
      byte RndA_dec[8]; // encrypted random A
      mbedtls_des3_set2key_dec(&tdes_ctx, key);
      mbedtls_des3_crypt_cbc(&tdes_ctx, MBEDTLS_DES_DECRYPT, s32_RandomSize, iv, BUFFER(command) + 1, RndA_dec);

      MFRC522Logger.write(DEBUG, "* RndA_enc recv:     ");
      MFRC522Logger.println(DEBUG, BUFFER(command) + 1, s32_RandomSize, HEX);
      MFRC522Logger.write(DEBUG, "* RndA_recv:     ");
      MFRC522Logger.println(DEBUG, RndA_dec, s32_RandomSize, HEX);

      // compare rotate_left(RndA) with RndA_enc
      if (*(RndA_dec + s32_RandomSize - 1) != RndA[0]) {
         MFRC522Logger.println(ERROR, "Authentication failed (3)");
         return false;
      }

      for (byte idx = 0; idx < s32_RandomSize - 1; idx++) {
        if (*(RndA_dec + idx) != RndA[1 + idx]) {
          MFRC522Logger.println(ERROR, "Authentication failed (4)");
          return false;
        }
      }

      return true;
    }
};

#endif
