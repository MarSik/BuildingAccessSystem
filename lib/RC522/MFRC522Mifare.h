#ifndef MFRC522_MIFARE_H
#define MFRC522_MIFARE_H

#include "MFRC522.h"

template < class T > class MFRC522Mifare : public MFRC522 < T > {
public:
  MFRC522Mifare(T & intf, byte resetPowerDownPin) : MFRC522<T>(intf, resetPowerDownPin) {}

  /**
   * Wrapper for MIFARE protocol communication.
   * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
   *
   * @return STATUS_OK on success, STATUS_??? otherwise.
   */
    StatusCode PCD_MIFARE_Transceive(byte * sendData,	///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
  				   byte sendLen,	///< Number of bytes in sendData.
  				   bool acceptTimeout	= false ///< True => A timeout is also success
      )
    {
      StatusCode result;
      byte cmdBuffer[18];		// We need room for 16 bytes data and 2 bytes CRC_A.

      // Sanity check
      if (sendData == NULL || sendLen > 16) {
        return STATUS_INVALID;
      }
      // Copy sendData[] to cmdBuffer[] and add CRC_A
      memcpy(cmdBuffer, sendData, sendLen);
      result = this->PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
      if (result != STATUS_OK) {
        return result;
      }
      sendLen += 2;

      // Transceive the data, store the reply in cmdBuffer[]
      byte waitIRq = 0x30;	// RxIRq and IdleIRq
      byte cmdBufferSize = sizeof(cmdBuffer);
      byte validBits = 0;
      result =
        this->PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen,
  			      cmdBuffer, &cmdBufferSize, &validBits);
      if (acceptTimeout && result == STATUS_TIMEOUT) {
        return STATUS_OK;
      }
      if (result != STATUS_OK) {
        return result;
      }
      // The PICC must reply with a 4 bit ACK
      if (cmdBufferSize != 1 || validBits != 4) {
        return STATUS_ERROR;
      }
      if (cmdBuffer[0] != MF_ACK) {
        return STATUS_MIFARE_NACK;
      }
      return STATUS_OK;
    }				// End PCD_MIFARE_Transceive()

  /**
   * Dumps memory contents of a MIFARE Ultralight PICC.
   */
    void PICC_DumpMifareUltralightToSerial()
    {
      StatusCode status;
      byte byteCount;
      byte buffer[18];
      byte i;

      Serial.println(F("Page  0  1  2  3"));
      // Try the mpages of the original Ultralight. Ultralight C has more pages.
      for (byte page = 0; page < 16; page += 4) {	// Read returns data for 4 pages at a time.
        // Read pages
        byteCount = sizeof(buffer);
        status = MIFARE_Read(page, buffer, &byteCount);
        if (status != STATUS_OK) {
  	Serial.print(F("MIFARE_Read() failed: "));
  	Serial.println(this->GetStatusCodeName(status));
  	break;
        }
        // Dump data
        for (byte offset = 0; offset < 4; offset++) {
  	i = page + offset;
  	if (i < 10)
  	  Serial.print(F("  "));	// Pad with spaces
  	else
  	  Serial.print(F(" "));	// Pad with spaces
  	Serial.print(i);
  	Serial.print(F("  "));
  	for (byte index = 0; index < 4; index++) {
  	  i = 4 * offset + index;
  	  if (buffer[i] < 0x10)
  	    Serial.print(F(" 0"));
  	  else
  	    Serial.print(F(" "));
  	  Serial.print(buffer[i], HEX);
  	}
  	Serial.println();
        }
      }
    }				// End PICC_DumpMifareUltralightToSerial()

    /**
     * Dumps memory contents of a MIFARE Classic PICC.
     * On success the PICC is halted after dumping the data.
     */
      void PICC_DumpMifareClassicToSerial(Uid * uid,	///< Pointer to Uid struct returned from a successful PICC_Select().
    				      PICC_Type piccType,	///< One of the PICC_Type enums.
    				      MIFARE_Key * key	///< Key A used for all sectors.
        )
      {
        byte no_of_sectors = 0;
        switch (piccType) {
        case PICC_TYPE_MIFARE_MINI:
          // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
          no_of_sectors = 5;
          break;

        case PICC_TYPE_MIFARE_1K:
          // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
          no_of_sectors = 16;
          break;

        case PICC_TYPE_MIFARE_4K:
          // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
          no_of_sectors = 40;
          break;

        default:			// Should not happen. Ignore.
          break;
        }

        // Dump sectors, highest address first.
        if (no_of_sectors) {
          Serial.println(F
    		     ("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"));
          for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
    	PICC_DumpMifareClassicSectorToSerial(uid, key, i);
          }
        }
        this->PICC_HaltA();		// Halt the PICC before stopping the encrypted session.
        this->PCD_StopCrypto1();
      }				// End PICC_DumpMifareClassicToSerial()

    /**
     * Dumps memory contents of a sector of a MIFARE Classic PICC.
     * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
     * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
     */
      void PICC_DumpMifareClassicSectorToSerial(Uid * uid,	///< Pointer to Uid struct returned from a successful PICC_Select().
    					    MIFARE_Key * key,	///< Key A for the sector.
    					    byte sector	///< The sector to dump, 0..39.
        )
      {
        StatusCode status;
        byte firstBlock;		// Address of lowest address to dump actually last block dumped)
        byte no_of_blocks;		// Number of blocks in sector
        bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.

        // The access bits are stored in a peculiar fashion.
        // There are four groups:
        //    g[3]  Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
        //    g[2]  Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
        //    g[1]  Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
        //    g[0]  Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
        // Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
        // The four CX bits are stored together in a nible cx and an inverted nible cx_.
        byte c1, c2, c3;		// Nibbles
        byte c1_, c2_, c3_;		// Inverted nibbles
        bool invertedError;		// True if one of the inverted nibbles did not match
        byte g[4];			// Access bits for each of the four groups.
        byte group;			// 0-3 - active group for access bits
        bool firstInGroup;		// True for the first block dumped in the group

        // Determine position and size of sector.
        if (sector < 32) {		// Sectors 0..31 has 4 blocks each
          no_of_blocks = 4;
          firstBlock = sector * no_of_blocks;
        } else if (sector < 40) {	// Sectors 32-39 has 16 blocks each
          no_of_blocks = 16;
          firstBlock = 128 + (sector - 32) * no_of_blocks;
        } else {			// Illegal input, no MIFARE Classic PICC has more than 40 sectors.
          return;
        }

        // Dump blocks, highest address first.
        byte byteCount;
        byte buffer[18];
        byte blockAddr;
        isSectorTrailer = true;
        invertedError = false;	// Avoid "unused variable" warning.
        for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0;
    	 blockOffset--) {
          blockAddr = firstBlock + blockOffset;
          // Sector number - only on first line
          if (isSectorTrailer) {
    	if (sector < 10)
    	  Serial.print(F("   "));	// Pad with spaces
    	else
    	  Serial.print(F("  "));	// Pad with spaces
    	Serial.print(sector);
    	Serial.print(F("   "));
          } else {
    	Serial.print(F("       "));
          }
          // Block number
          if (blockAddr < 10)
    	Serial.print(F("   "));	// Pad with spaces
          else {
    	if (blockAddr < 100)
    	  Serial.print(F("  "));	// Pad with spaces
    	else
    	  Serial.print(F(" "));	// Pad with spaces
          }
          Serial.print(blockAddr);
          Serial.print(F("  "));
          // Establish encrypted communications before reading the first block
          if (isSectorTrailer) {
    	status =
    	  this->PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
    	if (status != STATUS_OK) {
    	  Serial.print(F("PCD_Authenticate() failed: "));
    	  Serial.println(this->GetStatusCodeName(status));
    	  return;
    	}
          }
          // Read block
          byteCount = sizeof(buffer);
          status = MIFARE_Read(blockAddr, buffer, &byteCount);
          if (status != STATUS_OK) {
    	Serial.print(F("MIFARE_Read() failed: "));
    	Serial.println(this->GetStatusCodeName(status));
    	continue;
          }
          // Dump data
          for (byte index = 0; index < 16; index++) {
    	if (buffer[index] < 0x10)
    	  Serial.print(F(" 0"));
    	else
    	  Serial.print(F(" "));
    	Serial.print(buffer[index], HEX);
    	if ((index % 4) == 3) {
    	  Serial.print(F(" "));
    	}
          }
          // Parse sector trailer data
          if (isSectorTrailer) {
    	c1 = buffer[7] >> 4;
    	c2 = buffer[8] & 0xF;
    	c3 = buffer[8] >> 4;
    	c1_ = buffer[6] & 0xF;
    	c2_ = buffer[6] >> 4;
    	c3_ = buffer[7] & 0xF;
    	invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF))
    	  || (c3 != (~c3_ & 0xF));
    	g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
    	g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
    	g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
    	g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
    	isSectorTrailer = false;
          }
          // Which access group is this block in?
          if (no_of_blocks == 4) {
    	group = blockOffset;
    	firstInGroup = true;
          } else {
    	group = blockOffset / 5;
    	firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
          }

          if (firstInGroup) {
    	// Print access bits
    	Serial.print(F(" [ "));
    	Serial.print((g[group] >> 2) & 1, DEC);
    	Serial.print(F(" "));
    	Serial.print((g[group] >> 1) & 1, DEC);
    	Serial.print(F(" "));
    	Serial.print((g[group] >> 0) & 1, DEC);
    	Serial.print(F(" ] "));
    	if (invertedError) {
    	  Serial.print(F(" Inverted access bits did not match! "));
    	}
          }

          if (group != 3 && (g[group] == 1 || g[group] == 6)) {	// Not a sector trailer, a value block
    	int32_t value =
    	  (int32_t(buffer[3]) << 24) | (int32_t(buffer[2]) << 16) |
    	  (int32_t(buffer[1]) << 8) | int32_t(buffer[0]);
    	Serial.print(F(" Value=0x"));
    	Serial.print(value, HEX);
    	Serial.print(F(" Adr=0x"));
    	Serial.print(buffer[12], HEX);
          }
          Serial.println();
        }

        return;
      }				// End PICC_DumpMifareClassicSectorToSerial()

      /**
       * Dumps debug info about the selected PICC to Serial.
       * On success the PICC is halted after dumping the data.
       * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.
       *
       * @DEPRECATED Kept for bakward compatibility
       */
        void PICC_DumpToSerial(Uid * uid	///< Pointer to Uid struct returned from a successful PICC_Select().
          )
        {
          MIFARE_Key key;

          // Dump UID, SAK and Type
          this->PICC_DumpDetailsToSerial(uid);

          // Dump contents
          PICC_Type piccType = this->PICC_GetType(uid->sak);
          switch (piccType) {
          case PICC_TYPE_MIFARE_MINI:
          case PICC_TYPE_MIFARE_1K:
          case PICC_TYPE_MIFARE_4K:
            // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
            for (byte i = 0; i < 6; i++) {
      	key.keyByte[i] = 0xFF;
            }
            PICC_DumpMifareClassicToSerial(uid, piccType, &key);
            break;

          case PICC_TYPE_MIFARE_UL:
            PICC_DumpMifareUltralightToSerial();
            break;

          case PICC_TYPE_ISO_14443_4:
          case PICC_TYPE_MIFARE_DESFIRE:
          case PICC_TYPE_ISO_18092:
          case PICC_TYPE_MIFARE_PLUS:
          case PICC_TYPE_TNP3XXX:
            Serial.println(F
      		     ("Dumping memory contents not implemented for that PICC type."));
            break;

          case PICC_TYPE_UNKNOWN:
          case PICC_TYPE_NOT_COMPLETE:
          default:
            break;			// No memory dump here
          }

          Serial.println();
          this->PICC_HaltA();		// Already done if it was a MIFARE Classic PICC.
        }				// End PICC_DumpToSerial()

        /**
         * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
         *
         * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
         *
         * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
         * The MF0ICU1 returns a NAK for higher addresses.
         * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
         * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
         * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
         *
         * The buffer must be at least 18 bytes because a CRC_A is also returned.
         * Checks the CRC_A before returning STATUS_OK.
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_Read(byte blockAddr,	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
        			 byte * buffer,	///< The buffer to store the data in
        			 byte * bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
            )
          {
            StatusCode result;

            // Sanity check
            if (buffer == NULL || *bufferSize < 18) {
              return STATUS_NO_ROOM;
            }
            // Build command buffer
            buffer[0] = PICC_CMD_MF_READ;
            buffer[1] = blockAddr;
            // Calculate CRC_A
            result = this->PCD_CalculateCRC(buffer, 2, &buffer[2]);
            if (result != STATUS_OK) {
              return result;
            }
            // Transmit the buffer and receive the response, validate CRC_A.
            return this->PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
          }				// End MIFARE_Read()

        /**
         * Writes 16 bytes to the active PICC.
         *
         * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
         *
         * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
         * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
         * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
         * *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
          StatusCode MIFARE_Write(byte blockAddr,	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
        			  byte * buffer,	///< The 16 bytes to write to the PICC
        			  byte bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
            )
          {
            StatusCode result;

            // Sanity check
            if (buffer == NULL || bufferSize < 16) {
              return STATUS_INVALID;
            }
            // Mifare Classic protocol requires two communications to perform a write.
            // Step 1: Tell the PICC we want to write to block blockAddr.
            byte cmdBuffer[2];
            cmdBuffer[0] = PICC_CMD_MF_WRITE;
            cmdBuffer[1] = blockAddr;
            result = PCD_MIFARE_Transceive(cmdBuffer, 2);	// Adds CRC_A and checks that the response is MF_ACK.
            if (result != STATUS_OK) {
              return result;
            }
            // Step 2: Transfer the data
            result = PCD_MIFARE_Transceive(buffer, bufferSize);	// Adds CRC_A and checks that the response is MF_ACK.
            if (result != STATUS_OK) {
              return result;
            }

            return STATUS_OK;
          }				// End MIFARE_Write()

          /**
           * Authenticate with a NTAG216.
           *
           * Only for NTAG216. First implemented by Gargantuanman.
           *
           * @param[in]   passWord   password.
           * @param[in]   pACK       result success???.
           * @return STATUS_OK on success, STATUS_??? otherwise.
           */
            StatusCode PCD_NTAG216_AUTH(byte * passWord, byte pACK[])	//Authenticate with 32bit password
            {
              // TODO: Fix cmdBuffer length and rxlength. They really should match.
              //       (Better still, rxlength should not even be necessary.)

              StatusCode result;
              byte cmdBuffer[18];		// We need room for 16 bytes data and 2 bytes CRC_A.

              cmdBuffer[0] = 0x1B;	//Comando de autentificacion

              for (byte i = 0; i < 4; i++)
                cmdBuffer[i + 1] = passWord[i];

              result = this->PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);

              if (result != STATUS_OK) {
                return result;
              }
              // Transceive the data, store the reply in cmdBuffer[]
              byte waitIRq = 0x30;	// RxIRq and IdleIRq
          //  byte cmdBufferSize  = sizeof(cmdBuffer);
              byte validBits = 0;
              byte rxlength = 5;
              result =
                this->PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, 7,
          			      cmdBuffer, &rxlength, &validBits);

              pACK[0] = cmdBuffer[0];
              pACK[1] = cmdBuffer[1];

              if (result != STATUS_OK) {
                return result;
              }

              return STATUS_OK;
            }				// End PCD_NTAG216_AUTH()

            /**
             * Send command to Mifare compatible card and receive the reply. CRCs are computed and checked automatically,
             * leave 2 extra bytes in the buffer and response buffer for CRC
             *
             * buffer - command buffer
             * bufferSize - total size of buffer
             * usedSize - valid data count in buffer
             */
            StatusCode PCD_Mifare_TransceiveWithReply(byte* buffer, byte bufferSize, byte usedSize, byte* readBuffer, byte* readSize) {
              StatusCode result;

              if (bufferSize < usedSize + 2) {
                return STATUS_NO_ROOM;
              }

              // Calculate CRC_A
              result = this->PCD_CalculateCRC(buffer, usedSize, &buffer[usedSize]);
              if (result != STATUS_OK) {
                return result;
              }

              // Transmit the buffer and receive the response, validate CRC_A.
              if (this->debug) {
                Serial.print("Sent: ");
                println(buffer, usedSize + 2, HEX);
              }

              result = this->PCD_TransceiveData(buffer, usedSize + 2, readBuffer, readSize, NULL, 0, true);

              if (this->debug) {
                Serial.print("Received status: ");
                Serial.print(result);
                Serial.print(" resp. size: ");
                Serial.print(*readSize);
                Serial.print(" resp. code: ");
                println(readBuffer, *readSize, HEX);
              }

              if (result != STATUS_OK) {
                return result;
              }

              *readSize -= 2; // Substract the CRC size
              return STATUS_OK;
            }

            /**
             * Send command to Mifare compatible card and receive the reply. CRCs are computed and checked automatically,
             * leave 2 extra bytes in the buffer and response buffer for CRC
             *
             * buffer - command buffer
             * bufferSize - total size of buffer
             * usedSize - valid data count in buffer
             */
            StatusCode PCD_Mifare_TransceiveWithReply(byte* buffer, byte bufferSize, byte usedSize, byte* readBuffer, byte* readSize) const {
              StatusCode result;

              if (bufferSize < usedSize + 2) {
                return STATUS_NO_ROOM;
              }

              // Calculate CRC_A
              result = this->PCD_CalculateCRC(buffer, usedSize, &buffer[usedSize]);
              if (result != STATUS_OK) {
                return result;
              }

              // Transmit the buffer and receive the response, validate CRC_A.
              if (this->debug) {
                Serial.print("Sent: ");
                println(buffer, usedSize + 2, HEX);
              }

              result = this->PCD_TransceiveData(buffer, usedSize + 2, readBuffer, readSize, NULL, 0, true);

              if (this->debug) {
                Serial.print("Received status: ");
                Serial.print(result);
                Serial.print(" resp. size: ");
                Serial.print(*readSize);
                Serial.print(" resp. code: ");
                println(readBuffer, *readSize, HEX);
              }

              if (result != STATUS_OK) {
                return result;
              }

              *readSize -= 2; // Substract the CRC size
              return STATUS_OK;
            }

          protected:
            /**
             * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
             *
             * @return STATUS_OK on success, STATUS_??? otherwise.
             */
              StatusCode MIFARE_TwoStepHelper(byte command,	///< The command to use
            				  byte blockAddr,	///< The block (0-0xff) number.
            				  int32_t data	///< The data to transfer in step 2
                )
              {
                StatusCode result;
                byte cmdBuffer[2];		// We only need room for 2 bytes.

                // Step 1: Tell the PICC the command and block address
                cmdBuffer[0] = command;
                cmdBuffer[1] = blockAddr;
                result = PCD_MIFARE_Transceive(cmdBuffer, 2);	// Adds CRC_A and checks that the response is MF_ACK.
                if (result != STATUS_OK) {
                  return result;
                }
                // Step 2: Transfer the data
                result = PCD_MIFARE_Transceive((byte *) & data, 4, true);	// Adds CRC_A and accept timeout as success.
                if (result != STATUS_OK) {
                  return result;
                }

                return STATUS_OK;
              }				// End MIFARE_TwoStepHelper()

};

#endif
