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
    MFRC522IntfSpiOver485(HardwareSerial & serial, uint8_t txEn):_serial(serial),_txEn(txEn)
    {
    }

    void init() const;
    void begin() const;

    boolean PCD_WriteRegister(const PCD_Register reg, const byte value) const;
    boolean PCD_WriteRegister(const PCD_Register reg, const byte count, byte const * const values) const;
    int PCD_ReadRegister(const PCD_Register reg) const;
    boolean PCD_ReadRegister(const PCD_Register reg, byte count, byte * const values,
                             const byte rxAlign = 0) const;
    void end() const;

    // Toggle LED, no is 1 or 2, value controls the power
    void led(uint8_t no, bool value) const;

    void flush() const {
        _serial.flush();
    }

private:
    HardwareSerial & _serial;
    const uint8_t _txEn;

    const uint8_t ESC_XOR = 0x20;
    const uint8_t ESC = '\\';
    const uint8_t START_FRAME = '{';
    const uint8_t END_FRAME = '}';

    void tx(void) const {
      digitalWrite(_txEn, LOW);
    }

    void rx(void) const {
        Serial.print("\r\n");
      digitalWrite(_txEn, HIGH);
    }

    int waitRead() const {
        int rx;
        uint32_t timeout = millis() + 1000;

        do {
            rx = _serial.read();
            if (rx != -1) {
                Serial.print(rx, HEX);
                Serial.print(' ');
                Serial.flush();
            }
        } while(rx == -1 && timeout > millis());
        return rx;
    }

    int waitFrame() const {
        int rx;
        do {
            rx = waitRead();
        } while(rx != -1 && rx != START_FRAME);

        Serial.write(" START[");
        Serial.print(rx, HEX);
        Serial.write("]\r\n");
        Serial.flush();

        return rx;
    };

    void writeByte(byte x) const {
        if (x & 0x70) {
            x ^= ESC_XOR;
            Serial.print(x, HEX);
            Serial.print(' ');
            _serial.write('\\');
        }
        Serial.print(x, HEX);
        Serial.print(' ');
        _serial.write(x);
    }

    int readByte() const {
        int rx = waitRead();
        if (rx == ESC) {
            rx = waitRead();
            if (rx >= 0) {
                rx ^= ESC_XOR;
            }
        }
        return rx;
    }

    void dropFrameData() const {
        int b;
        do {
            b = _serial.peek();
            if (b != -1) {
                Serial.print(b, HEX);
                Serial.print(' ');
                Serial.flush();
            }
            if (b == START_FRAME) {
                Serial.print("< ");
                Serial.flush();
                return;
            } else {
                _serial.read();
            }
        } while (b != END_FRAME);
        Serial.print("\r\nEND\r\n");
        Serial.flush();
    }

    void dropFrame() const {
        waitFrame();
        dropFrameData();
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
