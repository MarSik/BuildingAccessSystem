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
      uint32_t timeout = millis() + 100;

      do {
        rx = _serial.read();
      } while(rx == -1 && timeout > millis());
      return rx;
  }
};

class MFRC522IntfSpiOver485
{
public:
    MFRC522IntfSpiOver485(HardwareSerial & serial, uint8_t txEn):_serial(serial),_txEn(txEn),_ready(false)
    {
    }

    bool ready() const {
        return _ready;
    }

    void init();
    void begin() const;

    boolean PCD_WriteRegister(const PCD_Register reg, const byte value) const;
    boolean PCD_WriteRegister(const PCD_Register reg, const byte count, byte const * const values) const;
    int PCD_ReadRegister(const PCD_Register reg) const;
    boolean PCD_ReadRegister(const PCD_Register reg, byte count, byte * const values,
                             const byte rxAlign = 0) const;
    void end() const;

    // Toggle LED, no is 1 or 2, value controls the power
    void led(uint8_t no, bool value) const;
    void configureSpi(uint8_t c1, uint8_t baud) const;

    void flush() const {
        _serial.flush();
    }

private:
    HardwareSerial & _serial;
    const uint8_t _txEn;
    bool _ready;

    const uint8_t ESC_XOR = 0x20;
    const uint8_t ESC = 0x7E;
    const uint8_t START_FRAME = '{';
    const uint8_t END_FRAME = '}';

    void tx(void) const;

    void rx(void) const;

    int waitRead() const;

    int waitFrame() const;

    void writeByte(byte x) const;

    int readByte() const;

    void dropFrameData() const;

    void dropFrame() const;

    void boost485Speed() const;
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
