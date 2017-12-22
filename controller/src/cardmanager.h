#ifndef CARDMANAGER_H
#define CARDMANAGER_H

#include "Secrets.h"
#include <Arduino.h>
#undef min
#undef max

#include <array.h>
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

struct CardInfo {
    bool valid;
    uint8_t flat;
    uint8_t cardno;
    uint8_t special_app;

    static const uint8_t ACCESS_BT = 0b10000000;
};

struct AccessRule {
    DefaultRule default_rule : 1;
    unsigned int dow : 7;
    unsigned int hour : 8;
    unsigned int _reserved : 12;
    unsigned int crc : 4; // 0x5a xor all four bytes (with crc = 0x0) and then high nibble xor low nibble

    void set_default(const DefaultRule rule) { default_rule = rule; }

    void add_day(const uint8_t dow) { this->dow |= dow; }
    void add_hour(const uint8_t hour) { this->hour |= 1 << (hour / 3); }

    void remove_day(const uint8_t dow) { this->dow &= ~dow; }
    void remove_hour(const uint8_t hour) { this->hour &= ~(1 << (hour / 3)); }

    bool check_access(const uint8_t dow, const uint8_t hour) const;

    void serialize(uint8_t buffer[4]) const;
    bool deserialize(const uint8_t value[4]);

    bool check_crc() const;

private:
    uint8_t compute_crc() const;
    const static uint8_t CHECKSUM_BASE = 0x5a;
};

template <typename Pcd>
class CardManager
{
public:
    CardManager(MFRC522Ultralight<Pcd> &pcd) : pcd(pcd) {}

    /**
     * @brief personalize_card initializes a new card to be used with this application
     * @return
     */
    bool personalize_card() const;
    bool reset_card() const;
    bool authenticate() const;

    /**
     * @brief set_rule changes the rule for the resource that is stored on the card
     * @param door
     * @param rule
     * @return
     */
    bool set_rule(const uint8_t door, const AccessRule &rule) const;

    /**
     * Read the access rule for specified door
     * @param door door id
     * @param rule AccessRule structure to fill
     * @return true if the get succeeded
     */
    bool get_rule(const uint8_t door, AccessRule &rule) const;

    /**
     * @brief check_valid makes sure the card is configured for the application
     * @return true or false
     */
    const CardInfo read_signature() const;

    /**
     * @brief authorize make sure the card is authorized to access the resource
     * @param dow
     * @param hour
     * @param door
     * @return true or false
     */
    bool authorize(const uint8_t dow, const uint8_t hour, const uint8_t door) const { return false; }

    const uint8_t ALL = 0xff;

protected:
    const uint32_t APPID = APPLICATION_ID;
    static const uint8_t APPID_PAGE = 0x04;

    const etl::array<const uint8_t, 4> DOORS = {{ 0x06, 0x07, 0x08, 0x09 }}; // double braces to workaround GCC bug 65815

    static const uint8_t FLAT_ID_PAGE = 0x05;
    static const uint8_t FLAT_ID_BYTE = 0;
    static const uint8_t FLAT_CHIP_ID_BYTE = 1;

    static const uint8_t SPECIAL_PAGE = 0x05;
    static const uint8_t SPECIAL_APP_ACCESS_BYTE = 2;
    static const uint8_t SPECIAL_RESERVED_BYTE = 3;

private:
    MFRC522Ultralight<Pcd> &pcd;
};

template<typename Pcd>
bool CardManager<Pcd>::authenticate() const {
    byte cardId[18];
    byte cardIdLen = 18;
    if (pcd.MIFARE_Read(0, cardId, &cardIdLen) != STATUS_OK)
    {
        RootLogger.write(DEBUG, "Could not get card header!\r\n");
        return false;
    }

    const CardInfo &signature = read_signature();
    if (!signature.valid) {
        RootLogger.write(DEBUG, "Invalid card signature!\r\n");
        return false;
    }

    // use last two bytes of cardId for flat + cardno
    cardId[16] = signature.flat;
    cardId[17] = signature.cardno;
    personalizeKey(cardId, cardId + 16, cardId);
    RootLogger.write(DEBUG, "Expected key: ");
    RootLogger.println(DEBUG, cardId, 16, HEX);

    if (!pcd.UltralightC_Authenticate(cardId)) // e.g. Error while authenticating with master key
    {
        RootLogger.write(DEBUG, "Card key does not match!\r\n");
        return false;
    }

    return true;
}

template <typename Pcd>
bool CardManager<Pcd>::personalize_card() const
{
    // Allow access, no conditions
    const byte buff[4] = {0xFF, 0xFF, 0x00, 0x0a ^ 0x05};
    byte header[18];
    byte headerLen = 18;
    pcd.MIFARE_Read(0, header, &headerLen);

    pcd.MIFARE_Ultralight_Write(APPID_PAGE, (const byte*)&APPID, 4);

    pcd.UltralightC_SetAuthProtection(0x3);

    const byte special[4] = {0x18, 0x01, 0x80, 0x00};

    // Compute per-card key from app key and uid
    personalizeKey(header, special, header);

    RootLogger.write(DEBUG, "New key: ");
    RootLogger.println(DEBUG, header, 16, HEX);

    pcd.UltralightC_ChangeKey(header);

    pcd.MIFARE_Ultralight_Write(DOORS[0], buff, 4);
    pcd.MIFARE_Ultralight_Write(DOORS[1], buff, 4);
    pcd.MIFARE_Ultralight_Write(DOORS[2], buff, 4);
    pcd.MIFARE_Ultralight_Write(DOORS[3], buff, 4);

    pcd.MIFARE_Ultralight_Write(FLAT_ID_PAGE, special, 4);

    return true; // TODO
}

template <typename Pcd>
bool CardManager<Pcd>::reset_card() const {
    const byte buff[4] = {0x00, 0x00, 0x00, 0x00};

    StatusCode code;

    // Set default code
    code = pcd.UltralightC_ChangeKey(DEFAULT_UL_KEY);
    if (code != STATUS_OK) {
        return code;
    }

    // Clear application ID
    code = pcd.MIFARE_Ultralight_Write(APPID_PAGE, buff, 4);
    if (code != STATUS_OK) {
        return code;
    }

    // Remove write protection
    code = pcd.UltralightC_SetAuthProtection(0x30);
    return code == STATUS_OK;
}

template<typename Pcd>
const CardInfo CardManager<Pcd>::read_signature() const {
    byte buffer[18] = {0, };
    byte bufferLen = 18;
    StatusCode status = pcd.MIFARE_Read(APPID_PAGE, buffer, &bufferLen);
    if (status != STATUS_OK) {
        return {false, 0, 0, 0};
    }

    // buffer bytes 0-3 contain little endian uint32_t app id (byte 0 is LSB)
    uint32_t receivedAppId = buffer[3];
    receivedAppId = (receivedAppId << 8) | buffer[2];
    receivedAppId = (receivedAppId << 8) | buffer[1];
    receivedAppId = (receivedAppId << 8) | buffer[0];

    return {receivedAppId == APPID,
            buffer[4], buffer[5], buffer[6]};
}

template <typename Pcd>
bool CardManager<Pcd>::get_rule(const uint8_t door, AccessRule &rule) const {
    byte buffer[18] = {0, };
    byte bufferLen = 18;
    StatusCode status = pcd.MIFARE_Read(DOORS[door], buffer, &bufferLen);
    if (status != STATUS_OK) {
        return false;
    }

    // buffer bytes 0-3 contain little endian uint32_t app id (byte 0 is LSB)
    rule.deserialize(buffer);
    return true;
}

template <typename Pcd>
bool CardManager<Pcd>::set_rule(const uint8_t door, const AccessRule &rule) const {
    uint8_t buffer[4];
    rule.serialize(buffer);

    StatusCode result = pcd.MIFARE_Ultralight_Write(DOORS[door], buffer, 4);
    return result == STATUS_OK;
}

#endif // CARDMANAGER_H
