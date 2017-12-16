//
// Created by msivak on 15.12.17.
//

#include <logging.h>
#include <MFRC522Common.h>
#include <Time.h>
#include "config.h"
#include "keyboard.h"
#include "Utils.h"
#include "states.h"

char       gs8_CommandBuffer[500];  // Stores commands typed by the user via Terminal and the password
uint32_t   gu32_CommandPos = 0;     // Index in gs8_CommandBuffer
uint64_t   gu64_LastPasswd = 0;     // Timestamp when the user has enetered the password successfully

// Temporary prototypes
void SetKey(const char* key);
void digitalClockDisplay();
uint32_t MeasureVoltage(byte u8_Pin);

void InitializeKB() {
    gs8_CommandBuffer[0] = 0;
}

void OnCommandReceived(bool b_PasswordValid);

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
            AccessSystem::dispatch(BadPasswordEntered{});
            return;
        }

        Utils::Print("Welcome to the access authorization terminal.\r\n");
        gs8_CommandBuffer[0] = 0; // clear buffer -> show menu
    }

    // As long as the user is logged in and types anything into the Terminal, the log-in time must be extended.
    gu64_LastPasswd = Utils::GetMillis64() + PASSWORD_OFFSET_MS;

    // This command must work even if gb_InitSuccess == false
    if (strncasecmp(gs8_CommandBuffer, "DEBUG", 5) == 0)
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
    if (strcasecmp(gs8_CommandBuffer, "RESET") == 0)
    {
        InitReader(false);
        if (gb_InitSuccess)
        {
            Utils::Print("Reader initialized successfully\r\n"); // The chip has reponded (ACK) as expected
            return;
        }
    }

    // This command must work even if gb_InitSuccess == false
    if (strcasecmp(gs8_CommandBuffer, "TESTAUTH") == 0)
    {
        AccessSystem::dispatch(TestCard{});
        return;
    }

    // This command must work even if gb_InitSuccess == false
    if (PASSWORD[0] != 0 && strcasecmp(gs8_CommandBuffer, "EXIT") == 0)
    {
        gu64_LastPasswd = 0;
        Utils::Print("You have logged out.\r\n");
        return;
    }

    if (gb_InitSuccess)
    {
        if (strcasecmp(gs8_CommandBuffer, "CLEAR") == 0)
        {
            return;
        }

        if (strncasecmp(gs8_CommandBuffer, "KEY", 3) == 0)
        {
            if (!ParseParameter(gs8_CommandBuffer + 3, &s8_Parameter, 32, 32))
                return;

            SetKey(s8_Parameter);
            return;
        }

        if (strcasecmp(gs8_CommandBuffer, "LIST") == 0)
        {
            // TODO list black and whitelists
            return;
        }

        if (strcasecmp(gs8_CommandBuffer, "RESTORE") == 0)
        {
            AccessSystem::dispatch(ClearCard{});
            return;
        }

        if (strncasecmp(gs8_CommandBuffer, "ADD", 3) == 0)
        {
            AccessSystem::dispatch(AddCard{});
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
