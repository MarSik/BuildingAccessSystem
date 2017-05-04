#ifndef CARDMANAGER_H
#define CARDMANAGER_H

#include <stdint.h>
#include <array.h>
#include <MFRC522Ultralight.h>
#include "Secrets.h"

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
    bool check_crc() const;

    void set_default(DefaultRule rule) { default_rule = rule; }

    void add_day(uint8_t dow) { this->dow |= dow; }
    void add_hour(uint8_t hour) { this->hour |= 1 << (hour / 3); }

    void remove_day(uint8_t dow) { this->dow &= ~dow; }
    void remove_hour(uint8_t hour) { this->hour &= ~(1 << (hour / 3)); }
};

template <typename Pcd>
class CardManager
{
public:
    CardManager(Pcd &pcd) : pcd(pcd) {}

    /**
     * @brief personalize_card initializes a new card to be used with this application
     * @return
     */
    bool personalize_card() const;
    bool reset_card() const;

    /**
     * @brief set_rule changes the rule for the resource that is stored on the card
     * @param door
     * @param rule
     * @return
     */
    bool set_rule(uint8_t door, AccessRule rule) const;

    /**
     * @brief check_valid makes sure the card is configured for the application
     * @return true or false
     */
    bool check_valid();

    /**
     * @brief authorize make sure the card is authorized to access the resource
     * @param dow
     * @param hour
     * @param door
     * @return true or false
     */
    bool authorize(uint8_t dow, uint8_t hour, uint8_t door) const;

protected:
    const uint32_t APPID = APPLICATION_ID;
    const uint8_t APPID_PAGE = 0x04;

    const etl::array<uint8_t, 4> DOORS = {0x05, 0x06, 0x07, 0x08};

private:
    Pcd &pcd;
};

#endif // CARDMANAGER_H
