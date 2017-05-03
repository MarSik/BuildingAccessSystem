#ifndef MFRC522_MIFARE_CLASSIC_H
#define MFRC522_MIFARE_CLASSIC_H

#include "MFRC522Mifare.h"

template < class T > class MFRC522MifareClassic:public MFRC522Mifare < T > {
public:
  MFRC522MifareClassic(T & intf, byte resetPowerDownPin) : MFRC522Mifare(intf, resetPowerDownPin) {}

  /**
   * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
   */
    void MIFARE_SetAccessBits(byte * accessBitBuffer,	///< Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
  			    byte g0,	///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
  			    byte g1,	///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
  			    byte g2,	///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
  			    byte g3	///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
      )
    {
      byte c1 =
        ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
      byte c2 =
        ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
      byte c3 =
        ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

      accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
      accessBitBuffer[1] = c1 << 4 | (~c3 & 0xF);
      accessBitBuffer[2] = c3 << 4 | c2;
    }				// End MIFARE_SetAccessBits()

    /**
     * Performs the "magic sequence" needed to get Chinese UID changeable
     * Mifare cards to allow writing to sector 0, where the card UID is stored.
     *
     * Note that you do not need to have selected the card through REQA or WUPA,
     * this sequence works immediately when the card is in the reader vicinity.
     * This means you can use this method even on "bricked" cards that your reader does
     * not recognise anymore (see MIFARE_UnbrickUidSector).
     *
     * Of course with non-bricked devices, you're free to select them before calling this function.
     */
      bool MIFARE_OpenUidBackdoor(bool logErrors)
      {
        // Magic sequence:
        // > 50 00 57 CD (HALT + CRC)
        // > 40 (7 bits only)
        // < A (4 bits only)
        // > 43
        // < A (4 bits only)
        // Then you can write to sector 0 without authenticating

        PICC_HaltA();		// 50 00 57 CD

        byte cmd = 0x40;
        byte validBits = 7;		/* Our command is only 7 bits. After receiving card response,
    				   this will contain amount of valid response bits. */
        byte response[32];		// Card's response is written here
        byte received;
        StatusCode status = PCD_TransceiveData(&cmd, (byte) 1, response, &received, &validBits, (byte) 0, false);	// 40
        if (status != STATUS_OK) {
          if (logErrors) {
    	Serial.println(F
    		       ("Card did not respond to 0x40 after HALT command. Are you sure it is a UID changeable one?"));
    	Serial.print(F("Error name: "));
    	Serial.println(GetStatusCodeName(status));
          }
          return false;
        }
        if (received != 1 || response[0] != 0x0A) {
          if (logErrors) {
    	Serial.print(F("Got bad response on backdoor 0x40 command: "));
    	Serial.print(response[0], HEX);
    	Serial.print(F(" ("));
    	Serial.print(validBits);
    	Serial.print(F(" valid bits)\r\n"));
          }
          return false;
        }

        cmd = 0x43;
        validBits = 8;
        status = PCD_TransceiveData(&cmd, (byte) 1, response, &received, &validBits, (byte) 0, false);	// 43
        if (status != STATUS_OK) {
          if (logErrors) {
    	Serial.println(F
    		       ("Error in communication at command 0x43, after successfully executing 0x40"));
    	Serial.print(F("Error name: "));
    	Serial.println(GetStatusCodeName(status));
          }
          return false;
        }
        if (received != 1 || response[0] != 0x0A) {
          if (logErrors) {
    	Serial.print(F("Got bad response on backdoor 0x43 command: "));
    	Serial.print(response[0], HEX);
    	Serial.print(F(" ("));
    	Serial.print(validBits);
    	Serial.print(F(" valid bits)\r\n"));
          }
          return false;
        }
        // You can now write to sector 0 without authenticating!
        return true;
      }				// End MIFARE_OpenUidBackdoor()

      /**
       * Reads entire block 0, including all manufacturer data, and overwrites
       * that block with the new UID, a freshly calculated BCC, and the original
       * manufacturer data.
       *
       * It assumes a default KEY A of 0xFFFFFFFFFFFF.
       * Make sure to have selected the card before this function is called.
       */
        bool MIFARE_SetUid(byte * newUid, byte uidSize, bool logErrors)
        {

          // UID + BCC byte can not be larger than 16 together
          if (!newUid || !uidSize || uidSize > 15) {
            if (logErrors) {
      	Serial.println(F("New UID buffer empty, size 0, or size > 15 given"));
            }
            return false;
          }
          // Authenticate for reading
          MIFARE_Key key = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
          StatusCode status =
            PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (byte) 1, &key, &uid);
          if (status != STATUS_OK) {

            if (status == STATUS_TIMEOUT) {
      	// We get a read timeout if no card is selected yet, so let's select one

      	// Wake the card up again if sleeping
      //        byte atqa_answer[2];
      //        byte atqa_size = 2;
      //        PICC_WakeupA(atqa_answer, &atqa_size);

      	if (!PICC_IsNewCardPresent() || !PICC_ReadCardSerial()) {
      	  Serial.println(F
      			 ("No card was previously selected, and none are available. Failed to set UID."));
      	  return false;
      	}

      	status =
      	  PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (byte) 1, &key, &uid);
      	if (status != STATUS_OK) {
      	  // We tried, time to give up
      	  if (logErrors) {
      	    Serial.println(F
      			   ("Failed to authenticate to card for reading, could not set UID: "));
      	    Serial.println(GetStatusCodeName(status));
      	  }
      	  return false;
      	}
            } else {
      	if (logErrors) {
      	  Serial.print(F("PCD_Authenticate() failed: "));
      	  Serial.println(GetStatusCodeName(status));
      	}
      	return false;
            }
          }
          // Read block 0
          byte block0_buffer[18];
          byte byteCount = sizeof(block0_buffer);
          status = MIFARE_Read((byte) 0, block0_buffer, &byteCount);
          if (status != STATUS_OK) {
            if (logErrors) {
      	Serial.print(F("MIFARE_Read() failed: "));
      	Serial.println(GetStatusCodeName(status));
      	Serial.println(F
      		       ("Are you sure your KEY A for sector 0 is 0xFFFFFFFFFFFF?"));
            }
            return false;
          }
          // Write new UID to the data we just read, and calculate BCC byte
          byte bcc = 0;
          for (uint8_t i = 0; i < uidSize; i++) {
            block0_buffer[i] = newUid[i];
            bcc ^= newUid[i];
          }

          // Write BCC byte to buffer
          block0_buffer[uidSize] = bcc;

          // Stop encrypted traffic so we can send raw bytes
          PCD_StopCrypto1();

          // Activate UID backdoor
          if (!MIFARE_OpenUidBackdoor(logErrors)) {
            if (logErrors) {
      	Serial.println(F("Activating the UID backdoor failed."));
            }
            return false;
          }
          // Write modified block 0 back to card
          status = MIFARE_Write((byte) 0, block0_buffer, (byte) 16);
          if (status != STATUS_OK) {
            if (logErrors) {
      	Serial.print(F("MIFARE_Write() failed: "));
      	Serial.println(GetStatusCodeName(status));
            }
            return false;
          }
          // Wake the card up again
          byte atqa_answer[2];
          byte atqa_size = 2;
          PICC_WakeupA(atqa_answer, &atqa_size);

          return true;
        }

      /**
       * Resets entire sector 0 to zeroes, so the card can be read again by readers.
       */
        bool MIFARE_UnbrickUidSector(bool logErrors)
        {
          MIFARE_OpenUidBackdoor(logErrors);

          byte block0_buffer[] =
            { 0x01, 0x02, 0x03, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00
          };

          // Write modified block 0 back to card
          StatusCode status = MIFARE_Write((byte) 0, block0_buffer, (byte) 16);
          if (status != STATUS_OK) {
            if (logErrors) {
      	Serial.print(F("MIFARE_Write() failed: "));
      	Serial.println(GetStatusCodeName(status));
            }
            return false;
          }
          return true;
        }

        /**
         * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
         * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
         * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
         * Use MIFARE_Transfer() to store the result in a block.
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_Increment(byte blockAddr,	///< The block (0-0xff) number.
        			      int32_t delta	///< This number is added to the value of block blockAddr.
            )
          {
            return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
          }				// End MIFARE_Increment()

        /**
         * MIFARE Restore copies the value of the addressed block into a volatile memory.
         * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
         * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
         * Use MIFARE_Transfer() to store the result in a block.
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_Restore(byte blockAddr	///< The block (0-0xff) number.
            )
          {
            // The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
            // Doing only a single step does not work, so I chose to transfer 0L in step two.
            return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
          }				// End MIFARE_Restore()

        /**
         * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
         * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
         * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_Transfer(byte blockAddr	///< The block (0-0xff) number.
            )
          {
            StatusCode result;
            byte cmdBuffer[2];		// We only need room for 2 bytes.

            // Tell the PICC we want to transfer the result into block blockAddr.
            cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
            cmdBuffer[1] = blockAddr;
            result = PCD_MIFARE_Transceive(cmdBuffer, 2);	// Adds CRC_A and checks that the response is MF_ACK.
            if (result != STATUS_OK) {
              return result;
            }
            return STATUS_OK;
          }				// End MIFARE_Transfer()

        /**
         * Helper routine to read the current value from a Value Block.
         *
         * Only for MIFARE Classic and only for blocks in "value block" mode, that
         * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
         * the block must be authenticated before calling this function.
         *
         * @param[in]   blockAddr   The block (0x00-0xff) number.
         * @param[out]  value       Current value of the Value Block.
         * @return STATUS_OK on success, STATUS_??? otherwise.
          */
          StatusCode MIFARE_GetValue(byte blockAddr, int32_t * value)
          {
            StatusCode status;
            byte buffer[18];
            byte size = sizeof(buffer);

            // Read the block
            status = MIFARE_Read(blockAddr, buffer, &size);
            if (status == STATUS_OK) {
              // Extract the value
              *value =
        	(int32_t(buffer[3]) << 24) | (int32_t(buffer[2]) << 16) |
        	(int32_t(buffer[1]) << 8) | int32_t(buffer[0]);
            }
            return status;
          }				// End MIFARE_GetValue()

        /**
         * Helper routine to write a specific value into a Value Block.
         *
         * Only for MIFARE Classic and only for blocks in "value block" mode, that
         * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
         * the block must be authenticated before calling this function.
         *
         * @param[in]   blockAddr   The block (0x00-0xff) number.
         * @param[in]   value       New value of the Value Block.
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_SetValue(byte blockAddr, int32_t value)
          {
            byte buffer[18];

            // Translate the int32_t into 4 bytes; repeated 2x in value block
            buffer[0] = buffer[8] = (value & 0xFF);
            buffer[1] = buffer[9] = (value & 0xFF00) >> 8;
            buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
            buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
            // Inverse 4 bytes also found in value block
            buffer[4] = ~buffer[0];
            buffer[5] = ~buffer[1];
            buffer[6] = ~buffer[2];
            buffer[7] = ~buffer[3];
            // Address 2x with inverse address 2x
            buffer[12] = buffer[14] = blockAddr;
            buffer[13] = buffer[15] = ~blockAddr;

            // Write the whole data block
            return MIFARE_Write(blockAddr, buffer, 16);
          }				// End MIFARE_SetValue()

            /**
             * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
             * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
             * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
             * Use MIFARE_Transfer() to store the result in a block.
             *
             * @return STATUS_OK on success, STATUS_??? otherwise.
             */
              StatusCode MIFARE_Decrement(byte blockAddr,	///< The block (0-0xff) number.
            			      int32_t delta	///< This number is subtracted from the value of block blockAddr.
                )
              {
                return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
              }				// End MIFARE_Decrement()
};

#endif
