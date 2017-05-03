#ifndef CARDMANAGER_H
#define CARDMANAGER_H

#include <stdint.h>
#include <MFRC522Ultralight.h>

enum DefaultRule {
    CLOSED_BY_DEFAULT = 0,
    OPEN_BY_DEFAULT = 1
};

enum DayOfWeek {
    MON = 1 << 6,
    TUE = 1 << 5,
    WED = 1 << 4,
    THU = 1 << 3,
    FRI = 1 << 2,
    SAT = 1 << 1,
    SUN = 1 << 0
};

struct AccessRule {
    DefaultRule default_rule : 1;
    int dow : 7;
    int hour : 8;
    int _reserved : 8;
    int crc : 8;

    void compute_crc();

    void set_default(DefaultRule rule) { default_rule = rule; }

    void add_day(uint8_t dow) { this->dow |= dow; }
    void add_hour(uint8_t hour) { this->hour |= 1 << (hour / 3); }

    void remove_day(uint8_t dow) { this->dow &= ~dow; }
    void remove_hour(uint8_t hour) { this->hour &= ~(1 << (hour / 3)); }
};

template <typename T>
class CardManager
{
public:
    CardManager(MFRC522Ultralight<T> &pcd) : pcd(pcd) {}

private:
    MFRC522Ultralight<T> &pcd;
};

#endif // CARDMANAGER_H
