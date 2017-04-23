#ifndef MFRC522_INTF_H
#define MFRC522_INTF_H

#include <Arduino.h>
#include <SPI.h>
#include "MFRC522Common.h"

#define MFRC522_SPICLOCK 80	// MFRC522 accept upto 10MHz

class MFRC522IntfSerial
{
public:
  MFRC522IntfSerial(HardwareSerial & serial):_serial(serial)
  {
  }

  void init();
  void begin()
  {
  };
  void PCD_WriteRegister(PCD_Register reg, byte value);
  void PCD_WriteRegister(PCD_Register reg, byte count, byte * values);
  byte PCD_ReadRegister(PCD_Register reg);
  void PCD_ReadRegister(PCD_Register reg, byte count, byte * values,
			byte rxAlign = 0);
  void end()
  {
  };

private:
  HardwareSerial & _serial;
};

class MFRC522IntfSpi
{
public:
  MFRC522IntfSpi(SPIClass & spi, uint8_t chipSelect):_spi(spi),
    _chipSelectPin(chipSelect)
  {
  }

  void init();
  void begin();
  void PCD_WriteRegister(PCD_Register reg, byte value);
  void PCD_WriteRegister(PCD_Register reg, byte count, byte * values);
  byte PCD_ReadRegister(PCD_Register reg);
  void PCD_ReadRegister(PCD_Register reg, byte count, byte * values,
			byte rxAlign = 0);
  void end();

private:
  SPIClass & _spi;
  byte _chipSelectPin;		// Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
};

#endif
