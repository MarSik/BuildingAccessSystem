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
#define DOOR_1_PIN       9

// This Arduino / Teensy pin is connected to the optional relay that opens the door 2
#define DOOR_2_PIN       10

// This Arduino / Teensy pin is connected to the transistor that charges the battery
#define CHARGE_PIN       19


// The software SPI SCK  pin (Clock)
#define SPI_CLK_PIN       23
// The software SPI MISO pin (Master In, Slave Out)
#define SPI_MISO_PIN      25
// The software SPI MOSI pin (Master Out, Slave In)
#define SPI_MOSI_PIN      26
// The software SPI SSEL pin (Chip Select)
#define SPI_CS_PIN        24

// This Arduino / Teensy pin is connected to the PN532 RSTPDN pin (reset the PN532) 
// When a communication error with the PN532 is detected the board is reset automatically.
#define RESET_PIN         10

// TODO use PN532 GPIOs to control LEDs on the terminal side
// This Arduino / Teensy pin is connected to the green LED in a two color LED.
// The green LED flashes fast while no card is present and flashes 1 second when opening the door.
#define LED_GREEN_PIN    39

// TODO use PN532 GPIOs to control LEDs on the terminal side
// This Arduino / Teensy pin is connected to the red LED in a two color LED.
// The red LED flashes slowly when a communication error occurred with the PN532 chip and when an unauthorized person tries to open the door.
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
// The shorter this interval, the more power is consumed by the PN532.
// The longer  this interval, the longer the user has to wait until the door opens.
// The recommended interval is 1000 ms.
// Please note that the slowness of reading a Desfire card is not caused by this interval.
// The SPI bus speed is throttled to 10 kHz, which allows to transmit the data over a long cable, but this obviously makes reading the card slower.
#define RF_OFF_INTERVAL  1000

#ifndef LED_BUILTIN
#define LED_BUILTIN 40
#endif // LED_BUILTIN

// ######################################################################################

#if defined(__MK20DX256__) // the CPU of the Teensy 3.1 / 3.2
    #if !defined(USB_SERIAL)
        #error "Switch the compiler to USB Type = 'Serial'"
    #endif
    #if F_CPU != 24000000
        #error "Switch the compiler to CPU Speed = '24 MHz optimized'"
    #endif
#else
    #warning "This code has not been tested on any other board than Teensy 3.1 / 3.2"
#endif

#include "UserManager.h"

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
bool       gb_InitSuccess  = false; // true if the PN532 has been initialized successfully

void setup() 
{
    gs8_CommandBuffer[0] = 0;

    Utils::SetPinMode(DOOR_1_PIN, OUTPUT);  
    Utils::WritePin  (DOOR_1_PIN, LOW);      

    Utils::SetPinMode(DOOR_2_PIN, OUTPUT);  
    Utils::WritePin  (DOOR_2_PIN, LOW);      

    Utils::SetPinMode(CHARGE_PIN, OUTPUT);  
    Utils::WritePin  (CHARGE_PIN, LOW);      

    Utils::SetPinMode(LED_GREEN_PIN, OUTPUT);
    Utils::SetPinMode(LED_RED_PIN,   OUTPUT);
    Utils::SetPinMode(LED_BUILTIN,   OUTPUT);
    
    // A longer pause is required to assure that the condensator at VOLTAGE_MEASURE_PIN 
    // has been charged before the battery voltage is measured for the first time.
    FlashLED(LED_GREEN, 1000);

    // Open USB serial port
    SerialClass::Begin(115200);
    Utils::Print("Reader initializing\n");

    // Use 12 bit resolution for the analog input (ADC)
    analogReadResolution(ANALOG_RESOLUTION);
    // Use the internal reference voltage (1.5V) as analog reference
    // analogReference(INTERNAL1V5); // TODO recompute the rest of the code to the proper Tiva references

    Utils::Print("UART reader port initialized\n");
    
    InitReader(false);
    Utils::Print("Reader initialized\n");
}

void loop()
{   
    bool b_KeyPress  = ReadKeyboardInput();
    bool b_VoltageOK = CheckBattery();   

    uint64_t u64_StartTick = Utils::GetMillis64();

    static uint64_t u64_LastRead = 0;
    if (gb_InitSuccess)
    {
        // While the user is typing do not read the card to avoid delays and debug output.
        if (b_KeyPress)
        {
            u64_LastRead = u64_StartTick + 1000; // Give the user 1000 ms + RF_OFF_INTERVAL between each character
            return;
        }

        // Turn on the RF field for 100 ms then turn it off for one second (RF_OFF_INTERVAL) to safe battery
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

        kUser k_User;
        kCard k_Card;
        if (!ReadCard(k_User.ID.u8, &k_Card))
        {
            /* XXX
            if (IsDesfireTimeout())
            {
                // Nothing to do here because IsDesfireTimeout() prints additional error message and blinks the red LED
            }
            else if (k_Card.b_PN532_Error) // Another error from PN532 -> reset the chip
            {
                InitReader(true); // flash red LED for 2.4 seconds
            }
            else // e.g. Error while authenticating with master key
            {
                FlashLED(LED_RED, 1000);
            }
            */
            
            Utils::Print("> ");
            break;
        }

        // XXX No card present in the RF field
        if (1) 
        {
            gu64_LastID = 0;
    
            // Flash the green LED shortly. On Power Failure flash the red LED shortly.
            FlashLED(b_VoltageOK ? LED_GREEN : LED_RED, 20);
            break;
        }

        // Still the same card present
        if (gu64_LastID == k_User.ID.u64) 
            break;
        
        // A different card was found in the RF field
        // OpenDoor() needs the RF field to be ON (for CheckDesfireSecret())
      	OpenDoor(k_User.ID.u64, &k_Card, u64_StartTick);
        Utils::Print("> ");
    }
    while (false);

    // Turn off the RF field to save battery
    // When the RF field is on,  the PN532 board consumes approx 110 mA.
    // When the RF field is off, the PN532 board consumes approx 18 mA.
    // XXX gi_PN532.SwitchOffRfField();

    u64_LastRead = Utils::GetMillis64();
}

// Reset the PN532 chip and initialize, set gb_InitSuccess = true on success
// If b_ShowError == true -> flash the red LED very slwoly
void InitReader(bool b_ShowError)
{
    if (b_ShowError)
    {
        SetLED(LED_RED);
        Utils::Print("Communication Error -> Reset PN532\r\n");
    }

    do // pseudo loop (just used for aborting with break;)
    {
        gb_InitSuccess = false;
      
        // XXX Reset the PN532
        gi_PN532.begin(); // delay > 400 ms
    
        byte IC, VersionHi, VersionLo, Flags;
        
        /* XXX
        if (!gi_PN532.GetFirmwareVersion(&IC, &VersionHi, &VersionLo, &Flags))
            break;
        */    

        char Buf[80];
        sprintf(Buf, "Chip: PN5%02X, Firmware version: %d.%d\r\n", IC, VersionHi, VersionLo);
        Utils::Print(Buf);
        sprintf(Buf, "Supports ISO 14443A:%s, ISO 14443B:%s, ISO 18092:%s\r\n", (Flags & 1) ? "Yes" : "No",
                                                                                (Flags & 2) ? "Yes" : "No",
                                                                                (Flags & 4) ? "Yes" : "No");
        Utils::Print(Buf);
         
        // XXX Set the max number of retry attempts to read from a card.
        // XXX configure the PN532 to read RFID tags
    
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

// If everything works correctly, the green LED will flash shortly (20 ms).
// If the LED does not flash permanently this means that there is a severe error.
// Additionally the LED will flash long (for 1 second) when the door is opened.
// -----------------------------------------------------------------------------
// The red LED shows a communication error with the PN532 (flash very slow),
// or someone not authorized trying to open the door (flash for 1 second)
// or on power failure the red LED flashes shortly.
void FlashLED(eLED e_LED, int s32_Interval)
{
    SetLED(e_LED);
    LongDelay(s32_Interval);
    SetLED(LED_OFF);
}

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

void OnCommandReceived(bool b_PasswordValid)
{
    kUser k_User;
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
      
        gi_PN532.SetDebugLevel(s8_Parameter[0] - '0');
        return;
    }    

    // This command must work even if gb_InitSuccess == false
    if (stricmp(gs8_CommandBuffer, "RESET") == 0)
    {
        InitReader(false);
        if (gb_InitSuccess)
        {
            Utils::Print("PN532 initialized successfully\r\n"); // The chip has reponded (ACK) as expected
            return;
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
    
        if (stricmp(gs8_CommandBuffer, "LIST") == 0)
        {
            UserManager::ListAllUsers();
            return;
        }

            if (stricmp(gs8_CommandBuffer, "RESTORE") == 0)
            {
                if (RestoreDesfireCard()) Utils::Print("Restore success\r\n");
                else                      Utils::Print("Restore failed\r\n");
                gi_PN532.SwitchOffRfField();
                return;
            }

            if (stricmp(gs8_CommandBuffer, "MAKERANDOM") == 0)
            {
                if (MakeRandomCard()) Utils::Print("MakeRandom success\r\n");
                else                  Utils::Print("MakeRandom failed\r\n");
                gi_PN532.SwitchOffRfField();
                return;
            }

    
        if (strnicmp(gs8_CommandBuffer, "ADD", 3) == 0)
        {
            if (!ParseParameter(gs8_CommandBuffer + 3, &s8_Parameter, 3, NAME_BUF_SIZE -1))
                return;

            AddCardToEeprom(s8_Parameter);

            // Required! Otherwise the next ReadPassiveTargetId() does not detect the card and the door opens after adding a user.
            gi_PN532.SwitchOffRfField();
            return;
        }
    
        if (strnicmp(gs8_CommandBuffer, "DEL", 3) == 0)
        {
            if (!ParseParameter(gs8_CommandBuffer + 3, &s8_Parameter, 3, NAME_BUF_SIZE -1))
                return;
          
            if (!UserManager::DeleteUser(0, s8_Parameter))
                Utils::Print("Error: User not found.\r\n");
                
            return;
        }    

        if (strnicmp(gs8_CommandBuffer, "DOOR12", 6) == 0) // FIRST !!!
        {
            if (!ParseParameter(gs8_CommandBuffer + 6, &s8_Parameter, 3, NAME_BUF_SIZE -1))
                return;
          
            if (!UserManager::SetUserFlags(s8_Parameter, DOOR_BOTH))
                Utils::Print("Error: User not found.\r\n");

            return;
        }    
        if (strnicmp(gs8_CommandBuffer, "DOOR1", 5) == 0) // AFTER !!!
        {
            if (!ParseParameter(gs8_CommandBuffer + 5, &s8_Parameter, 3, NAME_BUF_SIZE -1))
                return;
          
            if (!UserManager::SetUserFlags(s8_Parameter, DOOR_ONE))
                Utils::Print("Error: User not found.\r\n");

            return;
        }    
        if (strnicmp(gs8_CommandBuffer, "DOOR2", 5) == 0)
        {
            if (!ParseParameter(gs8_CommandBuffer + 5, &s8_Parameter, 3, NAME_BUF_SIZE -1))
                return;
          
            if (!UserManager::SetUserFlags(s8_Parameter, DOOR_TWO))
                Utils::Print("Error: User not found.\r\n");

            return;
        }    

        if (strlen(gs8_CommandBuffer))
            Utils::Print("Invalid command.\r\n\r\n");
        // else: The user pressed only ENTER

        Utils::Print("Usage:\r\n");
        Utils::Print(" CLEAR          : Clear all users and their cards from the EEPROM\r\n");    
        Utils::Print(" ADD    {user}  : Add a user and his card to the EEPROM\r\n");
        Utils::Print(" DEL    {user}  : Delete a user and his card from the EEPROM\r\n");
        Utils::Print(" LIST           : List all users that are stored in the EEPROM\r\n");    
        Utils::Print(" DOOR1  {user}  : Open only door 1 for this user\r\n");
        Utils::Print(" DOOR2  {user}  : Open only door 2 for this user\r\n");
        Utils::Print(" DOOR12 {user}  : Open both doors for this user\r\n");
        
            Utils::Print(" RESTORE        : Removes the master key and the application from the card\r\n");
            Utils::Print(" MAKERANDOM     : Converts the card into a Random ID card (FOREVER!)\r\n");
    }
    else // !gb_InitSuccess
    {
        Utils::Print("FATAL ERROR: The PN532 did not respond. (Board initialization failed)\r\n");
        Utils::Print("Usage:\r\n");
    }

    // In case of a fatal error only these 2 commands are available:
    Utils::Print(" RESET          : Reset the PN532 and run the chip initialization anew\r\n");
    Utils::Print(" DEBUG {level}  : Set debug level (0= off, 1= normal, 2= RxTx data, 3= details)\r\n");

    if (PASSWORD[0] != 0)
        Utils::Print(" EXIT           : Log out\r\n");
    Utils::Print(LF);
    Utils::Print("Compiled for Ultralight C cards (3K3DES - 168 bit encryption used)\r\n");

    int s32_MaxUsers = UserManager::GetMaxUsers();
    char Buf[80];
    sprintf(Buf, "Max %d users with a max name length of %d chars fit into the EEPROM\r\n", s32_MaxUsers, NAME_BUF_SIZE - 1);
    Utils::Print(Buf);

    Utils::Print("Terminal access is password protected: ");
    Utils::Print(PASSWORD[0] ? "Yes\r\n" : "No\r\n");

    uint32_t u32_Volt = MeasureVoltage(VOLTAGE_MEASURE_PIN);
    sprintf(Buf, "Battery voltage: %d.%d Volt\r\n",  (int)(u32_Volt/10), (int)(u32_Volt%10));
    Utils::Print(Buf);

    Utils::Print("System is running since ");   
    Utils::PrintInterval(Utils::GetMillis64(), LF);
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

// Stores a new user and his card in the EEPROM of the Teensy
void AddCardToEeprom(const char* s8_UserName)
{
    kUser k_User;
    kCard k_Card;   
    if (!WaitForCard(&k_User, &k_Card))
        return;
     
    // First the entire memory of s8_Name is filled with random data.
    // Then the username + terminating zero is written over it.
    // The result is for example: s8_Name[NAME_BUF_SIZE] = { 'P', 'e', 't', 'e', 'r', 0, 0xDE, 0x45, 0x70, 0x5A, 0xF9, 0x11, 0xAB }
    // The string operations like stricmp() will only read up to the terminating zero, 
    // but the application master key is derived from user name + random data.
    Utils::GenerateRandom((byte*)k_User.s8_Name, NAME_BUF_SIZE);
    strcpy(k_User.s8_Name, s8_UserName);

    // Utils::Print("User + Random data: ");
    // Utils::PrintHexBuf((byte*)k_User.s8_Name, NAME_BUF_SIZE, LF);

    kUser k_Found;  
    if (UserManager::FindUser(k_User.ID.u64, &k_Found))
    {
        Utils::Print("This card has already been stored for user ");
        Utils::Print(k_Found.s8_Name, LF);
        return;
    }
  
    #if USE_DESFIRE
        if ((k_Card.e_CardType & CARD_Desfire) == 0)
        {
            Utils::Print("The card is not a Desfire card.\r\n");
            return;
        }

        if (!ChangePiccMasterKey())
            return;

        if (k_Card.e_CardType != CARD_DesRandom)
        {
            // The secret stored in a file on the card is not required when using a card with random ID 
            // because obtaining the real card UID already requires the PICC master key. This is enough security.
            if (!StoreDesfireSecret(&k_User))
            {
                Utils::Print("Could not personalize the card.\r\n");
                return;
            }
        }
    #endif

    // By default a new user can open door one
    k_User.u8_Flags = DOOR_ONE;

    UserManager::StoreNewUser(&k_User);
}

void ClearEeprom()
{
    Utils::Print("\r\nATTENTION: ALL cards and users will be erased.\r\nIf you are really sure hit 'Y' otherwise hit 'N'.\r\n\r\n");

    if (!WaitForKeyYesNo())
        return;

    UserManager::DeleteAllUsers();
    Utils::Print("All cards have been deleted.\r\n");
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

// Waits for the user to approximate the card to the reader
// Timeout = 30 seconds
// Fills in pk_Card competely, but writes only the UID to pk_User.
bool WaitForCard(kUser* pk_User, kCard* pk_Card)
{
    Utils::Print("Please approximate the card to the reader now!\r\nYou have 30 seconds. Abort with ESC.\r\n");
    uint64_t u64_Start = Utils::GetMillis64();
    
    while (true)
    {
        if (ReadCard(pk_User->ID.u8, pk_Card) && pk_Card->u8_UidLength > 0)
        {
            // Avoid that later the door is opened for this card if the card is a long time in the RF field.
            gu64_LastID = pk_User->ID.u64;

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

// Reads the card in the RF field.
// In case of a Random ID card reads the real UID of the card (requires PICC authentication)
// ATTENTION: If no card is present, this function returns true. This is not an error. (check that pk_Card->u8_UidLength > 0)
// pk_Card->u8_KeyVersion is > 0 if a random ID card did a valid authentication with SECRET_PICC_MASTER_KEY
// pk_Card->b_PN532_Error is set true if the error comes from the PN532.
bool ReadCard(byte u8_UID[8], kCard* pk_Card)
{
    memset(pk_Card, 0, sizeof(kCard));
  
    if (!gi_PN532.ReadPassiveTargetID(u8_UID, &pk_Card->u8_UidLength, &pk_Card->e_CardType))
    {
        pk_Card->b_PN532_Error = true;
        return false;
    }

    if (pk_Card->e_CardType == CARD_DesRandom) // The card is a Desfire card in random ID mode
    {
        Utils::Print("Cards with random ID are not supported in Classic mode.\r\n");
        return false;    
    }
    return true;
}

// returns true if the cause of the last error was a Timeout.
// XXX This may happen for Desfire cards when the card is too far away from the reader.
bool IsDesfireTimeout()
{
    return false;
}

// b_PiccAuth = true if random ID card with successful authentication with SECRET_PICC_MASTER_KEY
void OpenDoor(uint64_t u64_ID, kCard* pk_Card, uint64_t u64_StartTick)
{
    kUser k_User;  
    if (!UserManager::FindUser(u64_ID, &k_User))
    {
        Utils::Print("Unknown person tries to open the door: ");
        Utils::PrintHexBuf((byte*)&u64_ID, 7, LF);
        FlashLED(LED_RED, 1000);
        return;
    }

    // TODO XXX Check card authorization

    switch (k_User.u8_Flags & DOOR_BOTH)
    {
        case DOOR_ONE:  Utils::Print("Opening door 1 for ");     break;
        case DOOR_TWO:  Utils::Print("Opening door 2 for ");     break;
        case DOOR_BOTH: Utils::Print("Opening door 1 + 2 for "); break;
        default:        Utils::Print("No door specified for ");  break;
    }
    Utils::Print(k_User.s8_Name);
    switch (pk_Card->e_CardType)
    {
        case CARD_DesRandom: Utils::Print(" (Desfire random card)",  LF); break;
        case CARD_Desfire:   Utils::Print(" (Desfire default card)", LF); break;
        default:             Utils::Print(" (Classic card)",         LF); break;
    }

    SetLED(LED_GREEN);
    if (k_User.u8_Flags & DOOR_ONE)
    {
        Utils::WritePin(DOOR_1_PIN, HIGH);
        LongDelay(OPEN_INTERVAL);
        Utils::WritePin(DOOR_1_PIN, LOW);
    }
    if ((k_User.u8_Flags & DOOR_BOTH) == DOOR_BOTH)
    {
        LongDelay(500); // make a pause between activation of the relais
    }
    if (k_User.u8_Flags & DOOR_TWO)
    {
        Utils::WritePin(DOOR_2_PIN, HIGH);
        LongDelay(OPEN_INTERVAL);
        Utils::WritePin(DOOR_2_PIN, LOW);
    }
    LongDelay(1000);
    SetLED(LED_OFF);

    // Avoid that the door is opened twice when the card is in the RF field for a longer time.
    gu64_LastID = u64_ID;
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

// Use this for delays > 100 ms to guarantee that the battery is checked frequently
void LongDelay(int s32_Interval)
{
    while(s32_Interval > 0)
    {
        CheckBattery();

        int s32_Delay = min(100, s32_Interval);
        Utils::DelayMilli(s32_Delay);

        s32_Interval -= s32_Delay;
    }
}


