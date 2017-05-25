#ifndef RC522_UTILS_H
#define RC522_UTILS_H

#include <Arduino.h>
#include <stdint.h>

#define CREATE_BUFFER(name, size) byte name[(size+2)] = {(size), 0, }

byte* BUFFER(byte* buff);
byte BUFFER_SIZE(byte* buff);
byte BUFFER_LEN(byte* buff);
void BUFFER_CLEAR(byte* buff);
void SET_BUFFER_LEN(byte* buff, byte len);
void ADD_BUFFER(byte* buff, byte b);
void ADD_BUFFER_PTR(byte* buff, byte* source, byte len);
void rotate_left(byte* dest, byte* source, byte len);
void generate_random(byte* dest, byte len);

#endif
