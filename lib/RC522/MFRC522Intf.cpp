#include "SPI.h"
#include "MFRC522Intf.h"

void
MFRC522IntfSpi::init()
{
  // Set the chipSelectPin as digital output, do not select the slave yet
  pinMode(_chipSelectPin, OUTPUT);
  digitalWrite(_chipSelectPin, HIGH);
}

void
MFRC522IntfSpi::begin()
{
  digitalWrite(_chipSelectPin, LOW);	// Select slave
}

void
MFRC522IntfSpi::end()
{
  digitalWrite(_chipSelectPin, HIGH);	// Release slave again
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSpi::PCD_WriteRegister(PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
				  byte value	///< The value to write.
  )
{
  begin();
  _spi.transfer(reg << 1); // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  _spi.transfer(value);
  end();
  return true;
}				// End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSpi::PCD_WriteRegister(PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
				  byte count,	///< The number of bytes to write to the register
				  byte * values	///< The values to write. Byte array.
  )
{
  begin();			// Select slave
  _spi.transfer(reg << 1);	// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  for (byte index = 0; index < count; index++) {
    _spi.transfer(values[index]);
  }
  end(); // Release slave again
  return true;
}				// End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
int MFRC522IntfSpi::PCD_ReadRegister(PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
  )
{
  byte
    value;
  begin();			// Select slave
  _spi.transfer(0x80 | (reg << 1));	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  value = _spi.transfer(0);	// Read the value back. Send 0 to stop reading.
  end();			// Release slave again
  return value;
}				// End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSpi::PCD_ReadRegister(PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
				 byte count,	///< The number of bytes to read
				 byte * values,	///< Byte array to store the values in.
				 byte rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
  )
{
  if (count == 0) {
    return true;
  }
  //Serial.print(F("Reading "));  Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | (reg << 1);	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;		// Index in values array.
  begin();			// Select slave
  count--;			// One read is performed outside of the loop
  _spi.transfer(address);	// Tell MFRC522 which address we want to read
  if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
    // Create bit mask for bit positions rxAlign..7
    byte mask = (0xFF << rxAlign) & 0xFF;
    // Read value and tell that we want to read the same address again.
    byte value = _spi.transfer(address);
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask);
    index++;
  }
  while (index < count) {
    values[index] = _spi.transfer(address);	// Read value and tell that we want to read the same address again.
    index++;
  }
  values[index] = _spi.transfer(0);	// Read the final byte. Send 0 to stop reading.
  end();			// Release slave again
  return true;
}				// End PCD_ReadRegister()

void
MFRC522IntfSerial::init()
{
    _serial.setBufferSize(128, 128);
    _serial.begin(9600);
    //flush();
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSerial::PCD_WriteRegister(const PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
                                     const byte value	///< The value to write.
  ) const
{
  begin();
  _serial.write(reg);		// MSB == 0 is for writing. Datasheet section 8.1.3.3.
  _serial.write(value);
  _serial.flush();
  //Serial.print("Writing addr: ");
  //Serial.print(reg, HEX);
  auto checkAddress = waitRead();
  boolean ret;
  if (checkAddress != reg) {
      //Serial.print(" ERR got ");
      //Serial.println(checkAddress, HEX);
      ret = false;
  } else {
      //Serial.print(" OK got ");
      //Serial.println(checkAddress, HEX);
      ret = true;
  }
  end();
  return ret;
}				// End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSerial::PCD_WriteRegister(const PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
                                     const byte count,	///< The number of bytes to write to the register
                                     byte const * const values	///< The values to write. Byte array.
  ) const
{			// Select slave
  boolean ret = true;
  for (byte index = 0; ret && index < count; index++) {
      begin();
      _serial.write(reg);		// MSB == 0 is for writing. Datasheet section 8.1.3.3.
      _serial.write(values[index]);
      //Serial.print("Writing addr: ");
      //Serial.print(reg, HEX);
      auto checkAddress = waitRead();
      if (checkAddress != reg) {
          //Serial.print(" ERR got ");
          //Serial.println(checkAddress, HEX);
          ret = false;
      } else {
          //Serial.print(" OK got ");
          //Serial.println(checkAddress, HEX);
      }
      end();
  }

  return ret;
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
int MFRC522IntfSerial::PCD_ReadRegister(const PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
  ) const
{
  begin();			// Select slave
  _serial.write(0x80 | reg);	// MSB == 1 is for reading. Datasheet section 8.1.3.3.
  const auto value = waitRead();// Read the value back. Send 0 to stop reading.
  end();			// Release slave again
  return value;
}				// End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
boolean
MFRC522IntfSerial::PCD_ReadRegister(const PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
                                    byte count,	///< The number of bytes to read
                                    byte * const values,	///< Byte array to store the values in.
                                    const byte rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
  ) const
{
  if (count == 0) {
    return true;
  }
  Serial.print(F("Reading "));  Serial.print(count); Serial.println(F(" bytes from register."));
  const byte address = 0x80 | reg;	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;		// Index in values array.
  begin();			// Select slave
  count--;			// One read is performed outside of the loop
  _serial.write(address);	// Tell MFRC522 which address we want to read
  if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
    // Create bit mask for bit positions rxAlign..7
    const byte mask = (0xFF << rxAlign) & 0xFF;
    // Read value and tell that we want to read the same address again.
    const int value = waitRead();
    if (value == -1) {
        Serial.print("F1");
        return false;
    }
    _serial.write(address);	// we want to read the same address again.
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask);
    index++;
  }
  while (index < count) {
    const int value = waitRead();
    if (value == -1) {
        Serial.print("F2");
        return false;
    }
    values[index] = value;	// Read the byte
    _serial.write(address);	// we want to read the same address again.
    index++;
  }
  const int value = waitRead();
  if (value == -1) {
      Serial.print("F3");
      return false;
  }
  values[index] = value;	// Read the final byte
  end();			// Release slave again

  return true;
}				// End PCD_ReadRegister()
