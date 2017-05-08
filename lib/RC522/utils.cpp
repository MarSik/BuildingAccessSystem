#include "utils.h"

byte* BUFFER(byte* buff) {
  return buff+2;
}

byte BUFFER_SIZE(byte* buff) {
  return buff[0];
}


byte BUFFER_LEN(byte* buff) {
  return buff[1];
}

void BUFFER_CLEAR(byte* buff) {
  buff[1] = 0;
}

void SET_BUFFER_LEN(byte* buff, byte len) {
  buff[1] = len;
}

void ADD_BUFFER(byte* buff, byte b) {
  buff[2 + buff[1]] = b;
  buff[1]++;
}

void ADD_BUFFER_PTR(byte* buff, byte* source, byte len) {
  memcpy(buff + 2 + buff[1], source, len);
  buff[1] += len;
}

void rotate_left(byte* dest, byte* source, byte len) {
    for (byte idx = len - 1; idx > 0; idx--) {
      *(dest + idx - 1) = *(source + idx);
    }
    *(dest + len - 1) = *source;
}

// TODO
void generate_random(byte* dest, byte len) {
  for (byte idx = 0; idx < len; idx++) {
    *(dest + idx) = 0;
  }
}
