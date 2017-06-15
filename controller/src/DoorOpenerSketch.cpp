/**************************************************************************

  @author   Elmü
  DIY electronic RFID Door Lock with Battery Backup (2016)

  Check for a new version of this code on
  http://www.codeproject.com/Articles/1096861/DIY-electronic-RFID-Door-Lock-with-Battery-Backup

**************************************************************************/

// This password will be required when entering via Terminal
// If you define an empty string here, no password is requested.
// If any unauthorized person may access the dooropener hardware phyically you should provide a password!
#define PASSWORD  ""
// The interval of inactivity in minutes after which the password must be entered again (automatic log-off)
#define PASSWORD_TIMEOUT  5

// This Arduino / Teensy pin is connected to the relay that opens the door 1
#define DOOR_PIN       9

// This Arduino / Teensy pin is connected to the transistor that charges the battery
#define CHARGE_PIN       19

#define BT_RESET 37

// The software SPI SCK  pin (Clock)
#define SPI_MODULE        3
// The software SPI SSEL pin (Chip Select)
#define SPI_CS_PIN        24

// This Arduino / Teensy pin is connected to the reader RSTPDN pin (reset the reader)
// When a communication error with the reader is detected the board is reset automatically.
#define RESET_PIN         5


// This Arduino / Teensy pin is connected to the green LED in a two color LED.
// The green LED flashes fast while no card is present and flashes 1 second when opening the door.
#define LED_GREEN_PIN    39

// This Arduino / Teensy pin is connected to the red LED in a two color LED.
// The red LED flashes slowly when a communication error occurred with the reader chip and when an unauthorized person tries to open the door.
// It flashes fast when a power failure has been detected. (Charging battery failed)
#define LED_RED_PIN      30

// This Arduino / Teensy pin is connected to the voltage divider that measures the 13,6V battery voltage
#define VOLTAGE_MEASURE_PIN  A3

// Use 12 bit resolution for the analog input (ADC)
// The Teensy 3.x boards have a 12 bit ADC.
#define ANALOG_RESOLUTION  12

// The analog reference voltage (float) of the CPU (analogReference(DEFAULT) --> 3.3V, analogReference(INTERNAL1V2) --> 1.2V)
#define ANALOG_REFERENCE   1.2

// This factor (float) is used to calculate the battery voltage.
// If the external voltage divider is 220 kOhm / 15 kOhm the factor is theoretically 15.66666 == (220 + 15) / 15.
// You must fine tune this value until the battery voltage is displayed correctly when you hit Enter in the Terminal.
// Therefor you must unplug the 220V power suppply and measure the real voltage at the battery.
#define VOLTAGE_FACTOR   15.9

// The interval in milliseconds that the relay is powered which opens the door
#define OPEN_INTERVAL   1000

// This is the interval that the RF field is switched off to save battery.
// The shorter this interval, the more power is consumed by the reader.
// The longer  this interval, the longer the user has to wait until the door opens.
// The recommended interval is 1000 ms.
// Please note that the slowness of reading a Desfire card is not caused by this interval.
// The SPI bus speed is throttled to 10 kHz, which allows to transmit the data over a long cable, but this obviously makes reading the card slower.
#define RF_OFF_INTERVAL  1000

#ifndef LED_BUILTIN
#define LED_BUILTIN 40
#endif // LED_BUILTIN

// DCF77 receiver
#define DCF77_PIN 11

// ######################################################################################

#include <Arduino.h>
#include <cstdlib>
#include <algorithm>
#include "SPI.h"
#include "EEPROM.h"
#include "DCF77.h"
#include "Time.h"
#include "MFRC522Ultralight.h"
#include "MFRC522Intf.h"
#include "Utils.h"
#include "Secrets.h"
#include "cardmanager.h"

DCF77 DCF = DCF77(DCF77_PIN, DCF77_PIN);

MFRC522IntfSerial mfrcIntf(Serial1);
MFRC522Ultralight<MFRC522IntfSerial> mfrc522(mfrcIntf, RESET_PIN);  // Create MFRC522 instance
CardManager<MFRC522IntfSerial> cardManager(mfrc522);

// The tick counter starts at zero when the CPU is reset.
// This interval is added to the 64 bit tick count to get a value that does not start at zero,
// because gu64_LastPasswd is initialized with 0 and must always be in the past.
#define PASSWORD_OFFSET_MS   (2 * PASSWORD_TIMEOUT * 60 * 1000)

enum eLED
{
    LED_OFF,
    LED_RED,
    LED_GREEN,
};

// global variables
char       gs8_CommandBuffer[500];  // Stores commands typed by the user via Terminal and the password
uint32_t   gu32_CommandPos = 0;     // Index in gs8_CommandBuffer
uint64_t   gu64_LastPasswd = 0;     // Timestamp when the user has enetered the password successfully
uint64_t   gu64_LastID     = 0;     // The last card UID that has been read by the RFID reader
bool       gb_InitSuccess  = false; // true if the reader has been initialized successfully

void SetLED(eLED e_LED)
{
    Utils::WritePin(LED_RED_PIN,   LOW);
    Utils::WritePin(LED_GREEN_PIN, LOW);
    Utils::WritePin(LED_BUILTIN,   LOW);

    switch (e_LED)
    {
        case LED_RED:
            Utils::WritePin(LED_RED_PIN, HIGH);
            //Utils::WritePin(LED_BUILTIN, HIGH); // LED on Teensy
            break;
        case LED_GREEN:
            Utils::WritePin(LED_GREEN_PIN, HIGH);
            //Utils::WritePin(LED_BUILTIN,   HIGH); // LED on Teensy
            break;
        default:  // Just to avoid stupid gcc compiler warning
            break;
    }
}

// returns the voltage at the given pin in Volt multiplied with 10
uint32_t MeasureVoltage(byte u8_Pin)
{
    const uint32_t maxValue = (1 << ANALOG_RESOLUTION) -1;  // == 4095 for 12 bit resolution

    float value = 10.0 * analogRead(u8_Pin);
    return (uint32_t)((value * ANALOG_REFERENCE * VOLTAGE_FACTOR) / maxValue);
}

// returns true if the battery voltage is OK (between 13 and 14 Volt)
// The perfect voltage for a 12V lead-acid battery is 13,6V.
// This voltage guarantees the longest possible life of the battery.
// This function must be called very frequently because at this voltage the battery has a high impedance
// and the voltage rises very quickly when charging with 200mA
bool CheckBattery()
{
    uint32_t u32_Volt = MeasureVoltage(VOLTAGE_MEASURE_PIN);
    if (u32_Volt > 136)
        Utils::WritePin(CHARGE_PIN, LOW); // Stop charging

    if (u32_Volt < 136)
        Utils::WritePin(CHARGE_PIN, HIGH); // Start charging

    return (u32_Volt >= 130 && u32_Volt < 140);
}

// Waits until the user either hits 'Y' or 'N'
// Timeout = 30 seconds
bool WaitForKeyYesNo()
{
    uint64_t u64_Start = Utils::GetMillis64();
    while (true)
    {
        char c_Char = SerialClass::Read();
        if  (c_Char == 'n' || c_Char == 'N' || (Utils::GetMillis64() - u64_Start) > 30000)
        {
            Utils::Print("Aborted.\r\n");
            return false;
        }

        if  (c_Char == 'y' || c_Char == 'Y')
             return true;

        CheckBattery();
                delay(200);
    }
}

// Reads the card in the RF field.
// ATTENTION: If no card is present, this function returns false. This is not an error.
bool ReadCard(uint64_t* uid)
{
  // Enable field
  mfrc522.PCD_AntennaOn();
  delay(100);

  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
      return false;
  }

  RootLogger.println(INFO, "Card detected...");

  // Select one of the cards
  StatusCode result = mfrc522.PICC_Select(&mfrc522.uid);
  if (result == STATUS_INTERNAL_ERROR || result == STATUS_ERROR || result == STATUS_TIMEOUT) {
      RootLogger.print(DEBUG, "Select failed: ");
      RootLogger.println(DEBUG, result, HEX);
      gb_InitSuccess = false;
      return false;
  }
  if (result != STATUS_OK) {
      return false;
  }

  // Convert the card id to binary
  *uid = 0;
  for (uint8_t idx = 0; idx < mfrc522.uid.size; idx++) {
    *uid <<= 8;
    *uid += mfrc522.uid.uidByte[idx];
  }
  Utils::PrintHex32(*uid, LF);

  // Dump debug info about the card; PICC_HaltA() is automatically called
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid), Serial);
  return true;
}

// Waits for the user to approximate the card to the reader
// Timeout = 30 seconds
// Fills in pk_Card competely, but writes only the UID to pk_User.
bool WaitForCard(uint64_t* pk_Card)
{
    Utils::Print("Please approximate the card to the reader now!\r\nYou have 30 seconds. Abort with ESC.\r\n");
    uint64_t u64_Start = Utils::GetMillis64();

    while (true)
    {
        uint64_t uid;
        if (ReadCard(&uid) && uid != 0)
        {
            // Avoid that later the door is opened for this card if the card is a long time in the RF field.
            gu64_LastID = uid;

            // All the stuff in this function takes about 2 seconds because the SPI bus speed has been throttled to 10 kHz.
            Utils::Print("Processing... (please do not remove the card)\r\n");
            return true;
        }

        if ((Utils::GetMillis64() - u64_Start) > 30000)
        {
            Utils::Print("Timeout waiting for card.\r\n");
            return false;
        }

        if (SerialClass::Read() == 27) // ESCAPE
        {
            Utils::Print("Aborted.\r\n");
            return false;
        }

        CheckBattery();
    }
}

// Use this for delays > 100 ms to guarantee that the battery is checked frequently
void LongDelay(int s32_Interval)
{
    while(s32_Interval > 0)
    {
        CheckBattery();

        int s32_Delay = std::min(100, s32_Interval);
        Utils::DelayMilli(s32_Delay);

        s32_Interval -= s32_Delay;
    }
}

// If everything works correctly, the green LED will flash shortly (20 ms).
// If the LED does not flash permanently this means that there is a severe error.
// Additionally the LED will flash long (for 1 second) when the door is opened.
// -----------------------------------------------------------------------------
// The red LED shows a communication error with the reader (flash very slow),
// or someone not authorized trying to open the door (flash for 1 second)
// or on power failure the red LED flashes shortly.
void FlashLED(eLED e_LED, int s32_Interval)
{
    SetLED(e_LED);
    LongDelay(s32_Interval);
    SetLED(LED_OFF);
}

// Reset the reader chip and initialize, set gb_InitSuccess = true on success
// If b_ShowError == true -> flash the red LED very slwoly
void InitReader(bool b_ShowError)
{
    if (b_ShowError)
    {
        SetLED(LED_RED);
        Utils::Print("Communication Error -> Reset reader\r\n");
    }

    do // pseudo loop (just used for aborting with break;)
    {
        gb_InitSuccess = false;

        Serial.println("Initializing reader...");
        if (!mfrc522.PCD_Init()) return;		// Init MFRC522
        Serial.println("Getting reader info..");
        mfrc522.PCD_DumpVersionToSerial(Serial);	// Show details of PCD - MFRC522 Card Reader details
        Serial.println("Setting gain to maximum");
        mfrc522.PCD_SetAntennaGain(PCD_RxGain::RxGain_avg);
        if (!mfrc522.PCD_WriteRegister(PCD_Register::RxThresholdReg, 0x22)) return;
        Serial.println(F("Reader initialized."));

        // XXX Set the max number of retry attempts to read from a card.

        gb_InitSuccess = true;
    }
    while (false);

    if (b_ShowError)
    {
        LongDelay(2000); // a long interval to make the LED flash very slowly
        SetLED(LED_OFF);
        LongDelay(100);
    }
}

// Parse the parameter behind "ADD", "DEL" and "DEBUG" commands and trim spaces
bool ParseParameter(char* s8_Command, char** ps8_Parameter, int minLength, int maxLength)
{
    int P=0;
    if (s8_Command[P++] != ' ')
    {
        // The first char after the command must be a space
        Utils::Print("Invalid command\r\n");
        return false;
    }

    // Trim spaces at the begin
    while (s8_Command[P] == ' ')
    {
        P++;
    }

    char* s8_Param = s8_Command + P;
    int   s32_Len  = strlen(s8_Param);

    // Trim spaces at the end
    while (s32_Len > 0 && s8_Param[s32_Len-1] == ' ')
    {
        s32_Len--;
        s8_Param[s32_Len] = 0;
    }

    if (s32_Len > maxLength)
    {
        Utils::Print("Parameter too long.\r\n");
        return false;
    }
    if (s32_Len < minLength)
    {
        Utils::Print("Parameter too short.\r\n");
        return false;
    }

    *ps8_Parameter = s8_Param;
    return true;
}

// ================================================================================

// Store key to EEPROM
void SetKey(const char* key)
{
  Utils::Print("New master key recorded!", LF);
  Utils::Print(key, LF);
  // TODO
}

// Stores a new user and his card in the EEPROM of the Teensy
void AddCardToEeprom()
{
    uint64_t cardId;
    if (!WaitForCard(&cardId))
        return;

    // First the entire memory of s8_Name is filled with random data.
    // Then the username + terminating zero is written over it.
    // The result is for example: s8_Name[NAME_BUF_SIZE] = { 'P', 'e', 't', 'e', 'r', 0, 0xDE, 0x45, 0x70, 0x5A, 0xF9, 0x11, 0xAB }
    // The string operations like stricmp() will only read up to the terminating zero,
    // but the application master key is derived from user name + random data.
    // Utils::GenerateRandom((byte*)k_User.s8_Name, NAME_BUF_SIZE);
    // strcpy(k_User.s8_Name, s8_UserName);

    // Utils::Print("User + Random data: ");
    // Utils::PrintHexBuf((byte*)k_User.s8_Name, NAME_BUF_SIZE, LF);

    if (!mfrc522.UltralightC_Authenticate(DEFAULT_UL_KEY)) {
        Utils::Print("Card already encrypted, please use blank card", LF);
        mfrc522.PCD_AntennaOff();
        return;
    }

    if (cardManager.check_valid()) {
        Utils::Print("Card already personalized in the past", LF);
    }

    if (mfrc522.UltralightC_ChangeKey(APPLICATION_KEY) != STATUS_OK) {
      Utils::Print("Application key could not be set", LF);
      return;
    }

    cardManager.personalize_card();
}

void ClearEeprom()
{
    Utils::Print("\r\nATTENTION: ALL cards and users will be erased.\r\nIf you are really sure hit 'Y' otherwise hit 'N'.\r\n\r\n");

    if (!WaitForKeyYesNo())
        return;

    Utils::Print("All cards have been deleted.\r\n");
}

void OpenDoor(uint64_t* cardId, uint64_t u64_StartTick)
{
    Utils::Print("Checking authorization.", LF);
    if (!cardManager.authorize(0, 0, DOOR_ID))
    {
        Utils::Print("Unknown person tries to open the door: ");
        Utils::PrintHexBuf((byte*)cardId, 7, LF);
        FlashLED(LED_RED, 1000);
        return;
    }

    SetLED(LED_GREEN);
    Utils::WritePin(DOOR_PIN, HIGH);
    LongDelay(OPEN_INTERVAL);
    Utils::WritePin(DOOR_PIN, LOW);
    LongDelay(1000);
    SetLED(LED_OFF);

    // Avoid that the door is opened twice when the card is in the RF field for a longer time.
    gu64_LastID = *cardId;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}


void OnCommandReceived(bool b_PasswordValid)
{
    char* s8_Parameter;

    gs8_CommandBuffer[gu32_CommandPos++] = 0;
    gu32_CommandPos = 0;
    Utils::Print(LF);

    if (!b_PasswordValid)
    {
        b_PasswordValid = strcmp(gs8_CommandBuffer, PASSWORD) == 0;
        if (!b_PasswordValid)
        {
            Utils::Print("Invalid password.\r\n");
            LongDelay(500);
            return;
        }

        Utils::Print("Welcome to the access authorization terminal.\r\n");
        gs8_CommandBuffer[0] = 0; // clear buffer -> show menu
    }

    // As long as the user is logged in and types anything into the Terminal, the log-in time must be extended.
    gu64_LastPasswd = Utils::GetMillis64() + PASSWORD_OFFSET_MS;

    // This command must work even if gb_InitSuccess == false
    if (strnicmp(gs8_CommandBuffer, "DEBUG", 5) == 0)
    {
        if (!ParseParameter(gs8_CommandBuffer + 5, &s8_Parameter, 1, 1))
            return;

        if (s8_Parameter[0] < '0' || s8_Parameter[0] > '3')
        {
            Utils::Print("Invalid debug level.\r\n");
            return;
        }

        if (s8_Parameter[0] == '2') {
            RootLogger.setLevel(DEBUG);
        } else if (s8_Parameter[0] == '3') {
            RootLogger.setLevel(TRACE);
        } else {
            RootLogger.setLevel(INFO);
        }


        return;
    }

    // This command must work even if gb_InitSuccess == false
    if (stricmp(gs8_CommandBuffer, "RESET") == 0)
    {
        InitReader(false);
        if (gb_InitSuccess)
        {
            Utils::Print("Reader initialized successfully\r\n"); // The chip has reponded (ACK) as expected
            return;
        }
    }

    // This command must work even if gb_InitSuccess == false
    if (stricmp(gs8_CommandBuffer, "TESTAUTH") == 0)
    {
        uint64_t cardId;
        Utils::Print("Waiting for card...", LF);
        if (!WaitForCard(&cardId))
            return;

        if (mfrc522.UltralightC_Authenticate(APPLICATION_KEY)) {
            Utils::Print("Authentication to card succeeded", LF);
        } else {
            Utils::Print("Authentication to card failed", LF);
        }
    }

    // This command must work even if gb_InitSuccess == false
    if (PASSWORD[0] != 0 && stricmp(gs8_CommandBuffer, "EXIT") == 0)
    {
        gu64_LastPasswd = 0;
        Utils::Print("You have logged out.\r\n");
        return;
    }

    if (gb_InitSuccess)
    {
        if (stricmp(gs8_CommandBuffer, "CLEAR") == 0)
        {
            ClearEeprom();
            return;
        }

        if (strnicmp(gs8_CommandBuffer, "KEY", 3) == 0)
        {
          if (!ParseParameter(gs8_CommandBuffer + 3, &s8_Parameter, 32, 32))
              return;

          SetKey(s8_Parameter);
          return;
        }

        if (stricmp(gs8_CommandBuffer, "LIST") == 0)
        {
            // TODO list black and whitelists
            return;
        }

        if (stricmp(gs8_CommandBuffer, "RESTORE") == 0)
        {
            if (cardManager.reset_card()) Utils::Print("Restore success\r\n");
            else                          Utils::Print("Restore failed\r\n");
            mfrc522.PCD_AntennaOff();
            return;
        }

        if (stricmp(gs8_CommandBuffer, "MAKERANDOM") == 0)
        {
                /*if (MakeRandomCard()) Utils::Print("MakeRandom success\r\n");
                else                  Utils::Print("MakeRandom failed\r\n");*/
            mfrc522.PCD_AntennaOff();
            return;
        }

        if (strnicmp(gs8_CommandBuffer, "ADD", 3) == 0)
        {
            AddCardToEeprom();

            // Required! Otherwise the next ReadPassiveTargetId() does not detect the card and the door opens after adding a user.
            mfrc522.PCD_AntennaOff();
            return;
        }

        if (strlen(gs8_CommandBuffer))
            Utils::Print("Invalid command.\r\n\r\n");
        // else: The user pressed only ENTER

        Utils::Print("Usage:\r\n");
        Utils::Print(" KEY    {key}   : Set the application key\r\n");
        Utils::Print(" CLEAR          : Clear all users and their cards from the EEPROM\r\n");
        Utils::Print(" ADD            : Add a user and his card to the EEPROM\r\n");
        Utils::Print(" DEL    {user}  : Delete a user and his card from the EEPROM\r\n");
        Utils::Print(" LIST           : List all users that are stored in the EEPROM\r\n");
        Utils::Print(" RESTORE        : Removes the master key and the application from the card\r\n");
    }
    else // !gb_InitSuccess
    {
        Utils::Print("FATAL ERROR: The reader did not respond. (Board initialization failed)\r\n");
        Utils::Print("Usage:\r\n");
    }

    // In case of a fatal error only these 2 commands are available:
    Utils::Print(" RESET          : Reset the reader and run the chip initialization anew\r\n");
    Utils::Print(" DEBUG {level}  : Set debug level (0= off, 1= normal, 2= RxTx data, 3= details)\r\n");

    if (PASSWORD[0] != 0)
        Utils::Print(" EXIT           : Log out\r\n");
    Utils::Print(LF);
    Utils::Print("Compiled for Ultralight C cards (3K3DES - 168 bit encryption used)\r\n");

    Utils::Print("Terminal access is password protected: ");
    Utils::Print(PASSWORD[0] ? "Yes\r\n" : "No\r\n");

    char Buf[80];
    uint32_t u32_Volt = MeasureVoltage(VOLTAGE_MEASURE_PIN);
    sprintf(Buf, "Battery voltage: %d.%d Volt\r\n",  (int)(u32_Volt/10), (int)(u32_Volt%10));
    Utils::Print(Buf);

    Utils::Print("System is running since ");
    Utils::PrintInterval(Utils::GetMillis64(), LF);
    digitalClockDisplay();
}

// Checks if the user has typed anything in the Terminal program and stores it in gs8_CommandBuffer
// Execute the command when Enter has been hit.
// returns true if any key has been pressed since the last call to this function.
bool ReadKeyboardInput()
{
    uint64_t u64_Now = Utils::GetMillis64() + PASSWORD_OFFSET_MS;

    bool b_KeyPress = false;
    while (SerialClass::Available())
    {
        b_KeyPress = true;
        // Check if the password must be entered
        bool b_PasswordValid = PASSWORD[0] == 0 || (u64_Now - gu64_LastPasswd) < (PASSWORD_TIMEOUT * 60 * 1000);

        byte u8_Char = SerialClass::Read();
        char s8_Echo[] = { (char)u8_Char, 0 };

        if (u8_Char == '\r' || u8_Char == '\n')
        {
            OnCommandReceived(b_PasswordValid);
            Utils::Print("\r\n> ");
            continue;
        }

        if (u8_Char == 8) // backslash
        {
            if (gu32_CommandPos > 0)
            {
                gu32_CommandPos --;
                Utils::Print(s8_Echo); // Terminal Echo
            }
            continue;
        }

        // Ignore all other control characters and characters that the terminal will not print correctly (e.g. umlauts)
        if (u8_Char < 32 || u8_Char > 126)
            continue;

        // Terminal Echo
        if (b_PasswordValid) Utils::Print(s8_Echo);
        else                 Utils::Print("*"); // don't display the password chars in the Terminal

        if (gu32_CommandPos >= sizeof(gs8_CommandBuffer))
        {
            Utils::Print("ERROR: Command too long\r\n");
            gu32_CommandPos = 0;
        }

        gs8_CommandBuffer[gu32_CommandPos++] = u8_Char;
    }
    return b_KeyPress;
}

void setup()
{
    gs8_CommandBuffer[0] = 0;

    Utils::SetPinMode(DOOR_PIN, OUTPUT);
    Utils::WritePin  (DOOR_PIN, LOW);

    Utils::SetPinMode(CHARGE_PIN, OUTPUT);
    Utils::WritePin  (CHARGE_PIN, LOW);

    Utils::SetPinMode(LED_GREEN_PIN, OUTPUT);
    Utils::SetPinMode(LED_RED_PIN,   OUTPUT);
    Utils::SetPinMode(LED_BUILTIN,   OUTPUT);

    Utils::SetPinMode(BT_RESET, OUTPUT);
    Utils::WritePin(BT_RESET, LOW);

    DCF.Start();

    // A longer pause is required to assure that the condensator at VOLTAGE_MEASURE_PIN
    // has been charged before the battery voltage is measured for the first time.
    FlashLED(LED_GREEN, 1000);

    // Open USB serial port
    Serial.setBufferSize(64, 64);
    Serial.begin(115200);
    Utils::Print("System initializing\n");

    // Use 12 bit resolution for the analog input (ADC)
    analogReadResolution(ANALOG_RESOLUTION);
    // Use the internal reference voltage (1.5V) as analog reference
    // analogReference(INTERNAL1V5); // TODO recompute the rest of the code to the proper Tiva references

    InitReader(false);
}

void loop()
{
    bool b_KeyPress  = ReadKeyboardInput();
    bool b_VoltageOK = CheckBattery();

    uint64_t u64_StartTick = Utils::GetMillis64();

    time_t DCFtime = DCF.getTime(); // Check if new DCF77 time is available
    if (DCFtime != 0)
    {
        setTime(DCFtime);
    }

    Serial.print("DCF buff ");
    Serial.println(DCF.bufferPosition);
    //Utils::PrintHex64(DCF.runningBuffer, LF);

    static uint64_t u64_LastRead = 0;
    if (gb_InitSuccess)
    {
        // While the user is typing do not read the card to avoid delays and debug output.
        if (b_KeyPress)
        {
            u64_LastRead = u64_StartTick + 1000; // Give the user 1000 ms + RF_OFF_INTERVAL between each character
            return;
        }

        // Turn on the RF field for 100 ms then turn it off for one second (RF_OFF_INTERVAL) to save battery
        if ((int)(u64_StartTick - u64_LastRead) < RF_OFF_INTERVAL)
            return;
    }

    do // pseudo loop (just used for aborting with break;)
    {
        if (!gb_InitSuccess)
        {
            InitReader(true); // flash red LED for 2.4 seconds
            break;
        }

        uint64_t uid;
        if (ReadCard(&uid))
        {
            if (!mfrc522.UltralightC_Authenticate(APPLICATION_KEY)) // e.g. Error while authenticating with master key
            {
                FlashLED(LED_RED, 1000);
                Utils::Print("Card authentication failed!", LF);
                break;
            } else {
              Utils::Print("Card successfully authenticated!", LF);
            }

            if (!cardManager.check_valid()) {
                Utils::Print("Card not configured for this application.", LF);
                break;
            }

            // TODO check blacklist
            // TODO decode permissions
            Utils::Print("> ");
        } else {
            gu64_LastID = 0;

            // Flash the green LED shortly. On Power Failure flash the red LED shortly.
            FlashLED(b_VoltageOK ? LED_GREEN : LED_RED, 20);
            break;
        }

        // Still the same card present
        if (gu64_LastID == uid) {
            Utils::PrintHex32(gu64_LastID);
            Utils::Print(" == ");
            Utils::PrintHex32(uid);
            Utils::Print(" the same card, ignoring..", LF);
            break;
        }

        // A different card was found in the RF field
        // OpenDoor() needs the RF field to be ON (for CheckDesfireSecret())
      	OpenDoor(&uid, u64_StartTick);
        Utils::Print("> ");
    }
    while (false);

    // Turn off the RF field to save battery
    // When the RF field is on,  the reader board consumes approx 110 mA.
    // When the RF field is off, the reader board consumes approx 18 mA.
    mfrc522.PCD_AntennaOff();

    u64_LastRead = Utils::GetMillis64();
}
