#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// This password will be required when entering via Terminal
// If you define an empty string here, no password is requested.
// If any unauthorized person may access the dooropener hardware phyically you should provide a password!
#define PASSWORD  ""
// The interval of inactivity in minutes after which the password must be entered again (automatic log-off)
#define PASSWORD_TIMEOUT  5

// The tick counter starts at zero when the CPU is reset.
// This interval is added to the 64 bit tick count to get a value that does not start at zero,
// because gu64_LastPasswd is initialized with 0 and must always be in the past.
#define PASSWORD_OFFSET_MS   (2 * PASSWORD_TIMEOUT * 60 * 1000)

// This Arduino / Teensy pin is connected to the relay that opens the door 1
#define DOOR_PIN       PA_6
// Sensing whether someone is pushing the door open button
#define BUZZ_SENSE     PA_7

// Tamper detection pin
#define TAMPER         PA_5

// This Arduino / Teensy pin is connected to the transistor that charges the battery
#define CHARGE_PIN       PB_2
#define CHARGE_BATTERY_ADC A3
#define CHARGE_CURR1_ADC A1
#define CHARGE_CURR2_ADC A0
#define CHARGE_VCC_ADC A10

// Bluetooth pins
#define BT_RESET PC_4
#define BT_AT PC_5 // AT log.1, communication log.0
#define BT_SERIAL 3

// SPI flash memory
#define MEM_CS PD_1
#define MEM_SPI 3
#define MEM_WP PE_1 // WP log.0

// Reader
// Make sure the TX pin can provide at least 8mA
#define READER_SERIAL 1
#define READER_TX PB_5 // TX active log.0
#define READER_ON PE_4 // ON log.0

// This Arduino / Teensy pin is connected to the reader RSTPDN pin (reset the reader)
// When a communication error with the reader is detected the board is reset automatically.
#define RESET_PIN         READER_ON


// This Arduino / Teensy pin is connected to the green LED in a two color LED.
// The green LED flashes fast while no card is present and flashes 1 second when opening the door.
#define LED_GREEN_PIN    GREEN_LED

// This Arduino / Teensy pin is connected to the red LED in a two color LED.
// The red LED flashes slowly when a communication error occurred with the reader chip and when an unauthorized person tries to open the door.
// It flashes fast when a power failure has been detected. (Charging battery failed)
#define LED_RED_PIN      RED_LED

// This Arduino / Teensy pin is connected to the voltage divider that measures the 13,6V battery voltage
#define VOLTAGE_MEASURE_PIN  CHARGE_BATTERY_ADC

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
#define RF_OFF_INTERVAL  500

#ifndef LED_BUILTIN
#define LED_BUILTIN BLUE_LED
#endif // LED_BUILTIN

// DCF77 receiver
#define DCF77_PIN PA_2

#endif
