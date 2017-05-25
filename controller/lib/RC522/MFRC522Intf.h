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
  void begin() const
  {
  }

  boolean PCD_WriteRegister(const PCD_Register reg, const byte value) const;
  boolean PCD_WriteRegister(const PCD_Register reg, const byte count, byte const * const values) const;
  int PCD_ReadRegister(const PCD_Register reg) const;
  boolean PCD_ReadRegister(const PCD_Register reg, byte count, byte * const values,
                        const byte rxAlign = 0) const;
  void end() const
  {
      delay(1);
  }

  void flush() const {
      while (_serial.available()) {
          auto b = _serial.read();
          Serial.write("Discarding PCD byte: ");
          Serial.println(b);
      }
  }

private:
  HardwareSerial & _serial;

  int waitRead() const {
      int rx;
      uint32_t timeout = 1e6;

      do {
        rx = _serial.read();
        timeout--;
      } while(rx == -1 && timeout);
      return rx;
  }
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
  boolean PCD_WriteRegister(PCD_Register reg, byte value);
  boolean PCD_WriteRegister(PCD_Register reg, byte count, byte * values);
  int PCD_ReadRegister(PCD_Register reg);
  boolean PCD_ReadRegister(PCD_Register reg, byte count, byte * values,
			byte rxAlign = 0);
  void end();

private:
  SPIClass & _spi;
  byte _chipSelectPin;		// Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
};

#endif
