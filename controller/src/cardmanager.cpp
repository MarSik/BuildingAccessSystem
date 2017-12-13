#include "cardmanager.h"

bool AccessRule::check_crc() const {
    return compute_crc() == crc;
}

uint8_t AccessRule::compute_crc() const {
    uint8_t expected_crc = CHECKSUM_BASE;
    uint8_t b = default_rule << 7 | dow;
    expected_crc ^= b;
    expected_crc ^= hour;
    return (expected_crc & 0xf) ^ (expected_crc >> 4);
}

void AccessRule::serialize(uint8_t buffer[4]) const {
    buffer[0] = dow | (default_rule == DefaultRule::OPEN_BY_DEFAULT ? 0b10000000 : 0x00);
    buffer[1] = hour;
    buffer[2] = 0x00;
    buffer[3] = compute_crc();
}

bool AccessRule::deserialize(const uint8_t value[4]) {
    default_rule = (value[0] & 0b1000000) ? OPEN_BY_DEFAULT : CLOSED_BY_DEFAULT;
    dow = value[0] & 0b01111111;
    hour = value[1];
    _reserved = 0;
    return true;
}

bool AccessRule::check_access(const uint8_t check_dow, const uint8_t check_hour) const {
    if (!check_crc()) {
        return false;
    }

    if (!(dow & (1 << check_dow))) {
        return false;
    }

    if (!(hour & (1 << (check_hour / 3)))) {
        return false;
    }

    return true;
}