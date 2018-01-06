#include "config.h"
#include "bluetooth.h"

const BluetoothSerialClass BluetoothSerial;
constexpr HardwareSerial &btSerial = Serial3;

void BluetoothSerialClass::configure() const {
    at();
    on();
    sleep(250);
    btSerial.begin(38400);
    dropdata();
    btSerial.println("AT+NAME=Opalkova18");
    btSerial.println("AT+PSWD=8642");
    btSerial.flush();
    sleep(100);
    end();
}

void BluetoothSerialClass::begin() const {
    comm();
    on();
    sleep(250);
    btSerial.begin(9600);
    dropdata();
}

void BluetoothSerialClass::end() const {
    off();
    btSerial.end();
    dropdata();
}

void BluetoothSerialClass::at() const {
    digitalWrite(BT_AT, HIGH);
}

void BluetoothSerialClass::comm() const {
    digitalWrite(BT_AT, LOW);
}

HardwareSerial& BluetoothSerialClass::serial() const {
    return btSerial;
}

void BluetoothSerialClass::on() const {
    // TODO XXX digitalWrite(BT_RESET, HIGH);
}

void BluetoothSerialClass::off() const {
    digitalWrite(BT_RESET, LOW);
}

void BluetoothSerialClass::dropdata() const {
    while(btSerial.available() > 0) {
        btSerial.read();
    }
}

