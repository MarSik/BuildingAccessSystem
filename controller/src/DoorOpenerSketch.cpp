/**************************************************************************

  @author   Elmü
  DIY electronic RFID Door Lock with Battery Backup (2016)

  Check for a new version of this code on
  http://www.codeproject.com/Articles/1096861/DIY-electronic-RFID-Door-Lock-with-Battery-Backup

**************************************************************************/

#include "config.h"

// ######################################################################################

#include <Arduino.h>
#undef min
#undef max

#include "SPI.h"
#include "DCF77.h"
#include "Time.h"
#include "MFRC522Ultralight.h"
#include "MFRC522Intf.h"
#include "Utils.h"
#include "Secrets.h"
#include "cardmanager.h"
#include "states.h"
#include "countdown.h"
#include "keyboard.h"
#include "hal.h"

DCF77 DCF = DCF77(DCF77_PIN, DCF77_PIN);

MFRC522IntfSpiOver485 mfrcIntf(Serial1, READER_TX);
MFRC522Ultralight<MFRC522IntfSpiOver485> mfrc522(mfrcIntf, READER_ON);  // Create MFRC522 instance
CardManager<MFRC522IntfSpiOver485> cardManager(mfrc522);

// global variables
uint64_t   gu64_LastID     = 0;     // The last card UID that has been read by the RFID reader
bool       gb_InitSuccess  = false; // true if the reader has been initialized successfully

void _prepareHighCurrentODOutputGPIO(const uint8_t arduinoPort);

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

// Reads the card in the RF field.
// ATTENTION: If no card is present, this function returns false. This is not an error.
bool ReadCard(uint64_t* uid)
{
  // Enable field
  if (!mfrc522.PCD_AntennaOn()) {
      return false;
  }
  //delay(100);

  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
      RootLogger.println(INFO, "No new card present.");
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

  // Dump debug info about the card; PICC_HaltA() is automatically called
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid), Serial);
  return true;
}

// If everything works correctly, the green LED will flash shortly (20 ms).
// If the LED does not flash permanently this means that there is a severe error.
// Additionally the LED will flash long (for 1 second) when the door is opened.
// -----------------------------------------------------------------------------
// The red LED shows a communication error with the reader (flash very slow),
// or someone not authorized trying to open the door (flash for 1 second)
// or on power failure the red LED flashes shortly.
/* void FlashLED(eLED e_LED, int s32_Interval)
{
    SetLED(e_LED);
    LongDelay(s32_Interval);
    SetLED(LED_OFF);
} */

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
        Serial.println("Setting gain");
        mfrc522.PCD_SetAntennaGain(PCD_RxGain::RxGain_avg);
        if (!mfrc522.PCD_WriteRegister(PCD_Register::RxThresholdReg, 0x22)) return;
        Serial.println(F("Reader initialized."));

        // XXX Set the max number of retry attempts to read from a card.

        gb_InitSuccess = true;
    }
    while (false);
}

// ================================================================================

// Store key to EEPROM
void SetKey(const char* key)
{
  Utils::Print("New master key recorded!", LF);
  Utils::Print(key, LF);
  // TODO
}

volatile bool _buzzerActivated = false;

void handleBuzzer(void) {
    const int val = digitalRead(BUZZ_SENSE);
    _buzzerActivated = val == LOW;

    if (val == LOW) {
        digitalWrite(DOOR_PIN, HIGH);
    } else {
        digitalWrite(DOOR_PIN, LOW);
    }
}

void setup()
{
    InitializeKB();

    Utils::SetPinMode(DOOR_PIN, OUTPUT);
    Utils::WritePin  (DOOR_PIN, LOW);

    Utils::SetPinMode(BUZZ_SENSE, INPUT);

    Utils::SetPinMode(CHARGE_PIN, OUTPUT);
    Utils::WritePin  (CHARGE_PIN, LOW);

    Utils::SetPinMode(LED_GREEN_PIN, OUTPUT);
    Utils::SetPinMode(LED_RED_PIN,   OUTPUT);
    Utils::SetPinMode(LED_BUILTIN,   OUTPUT);

    Utils::SetPinMode(BT_RESET, OUTPUT);
    Utils::WritePin(BT_RESET, LOW);

    // Configure open drain outputs with 8mA current
    _prepareHighCurrentODOutputGPIO(READER_ON);
    _prepareHighCurrentODOutputGPIO(READER_TX);

    // Enable DCF receiver
    DCF.Start();

    // Enable buzzer pass through
    attachInterrupt(BUZZ_SENSE, handleBuzzer, CHANGE);

    // Open USB serial port
    Serial.setBufferSize(64, 64);
    Serial.begin(115200);
    Utils::Print("System initializing\n");

    // Use 12 bit resolution for the analog input (ADC)
    analogReadResolution(ANALOG_RESOLUTION);
    // Use the internal reference voltage (1.5V) as analog reference
    // analogReference(INTERNAL1V5); // TODO recompute the rest of the code to the proper Tiva references

    // Initialize countdown timer
    initializeCountdown();

    // Initialize the access system FSM
    AccessSystem::initialize();
}

void _prepareHighCurrentODOutputGPIO(const uint8_t arduinoPort) {
    const uint8_t bit = digitalPinToBitMask(arduinoPort);
    const uint8_t port = digitalPinToPort(arduinoPort);
    const uint32_t portBase = (uint32_t) portBASERegister(port);
    ROM_GPIOPadConfigSet(portBase, bit, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD);
    ROM_GPIOPinWrite(portBase, bit, 1);
    ROM_GPIODirModeSet(portBase, bit, GPIO_DIR_MODE_OUT);
}

void loop()
{
    bool b_KeyPress  = Serial.available() > 0;
    bool b_VoltageOK = CheckBattery();

    // Countdown finished state machine transition
    if (countdownFinished()) {
        AccessSystem::dispatch(CountdownFinished{});
    }

    // User is typing
    if (b_KeyPress) {
        AccessSystem::dispatch(UserTyping{});
    }

    // Battery voltage outside limits
    if (!b_VoltageOK) {
        AccessSystem::dispatch(BatteryBad{});
    }

    if (_buzzerActivated) {
        AccessSystem::dispatch(BuzzerActivated{});
    } else {
        AccessSystem::dispatch(BuzzerReleased{});
    }

    time_t DCFtime = DCF.getTime(); // Check if new DCF77 time is available
    if (DCFtime != 0)
    {
        setTime(DCFtime);
        AccessSystem::dispatch(DCFReceived{});
    }

    // Sleep until something happens
    asm volatile ("wfi              \n"\
     		      "mov r0, #0       \n");  // force bx lr to not start until after clocks back on
}
