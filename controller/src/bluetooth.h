//
// https://www.itead.cc/wiki/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05
//

#ifndef PROJECT_BLUETOOTH_H
#define PROJECT_BLUETOOTH_H

#include <Arduino.h>

class BluetoothSerialClass {
public:
    BluetoothSerialClass() { }

    void configure() const;
    void begin() const;
    void end() const;

    void at() const;
    void comm() const;

    HardwareSerial& serial() const;
    void dropdata() const;

private:
    void on() const;
    void off() const;
};

extern const BluetoothSerialClass BluetoothSerial;

#endif //PROJECT_BLUETOOTH_H
