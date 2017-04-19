/************************************************************************************
 * 
 * @author   Elmü
 * 
 * This library has been tested with UltralightC EV1 cards. 
 * It will surely not work with older UltralightC cards (deprecated) because legacy authentication is not implemented.
 * I have older code with lagacy authentication. If you are interested contact me on Codeproject.
 * 
 * This library is based on code from the following open source libraries:
 * https://github.com/nceruchalu/easypay
 * https://github.com/leg0/libfreefare
 * http://liblogicalaccess.islog.com
 * 
 * The open source code has been completely rewritten for the Arduino compiler by Elmü.
 * Check for a new version on:
 * http://www.codeproject.com/Articles/1096861/DIY-electronic-RFID-Door-Lock-with-Battery-Backup
 * 
*************************************************************************************
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "UltralightC.h"
#include "Secrets.h"

UltralightC::UltralightC()
{
    mb_authenticated = false;

    // The key on an empty card is a simple DES key filled with 8 zeros
    const byte ZERO_KEY[24] = {0};
    DES3_DEFAULT_KEY.SetKeyData(ZERO_KEY, 24, 0); // triple DES
}

// Whenever the RF field is switched off, these variables must be reset
bool UltralightC::SwitchOffRfField()
{
    mb_authenticated = false;
    return PN532::SwitchOffRfField();
}

/**************************************************************************
    Does an ISO authentication with a 2K3DES key or an AES authentication with an AES key.
    pi_Key must be an instance of DES or AES.
    The authentication is a 3-pass process where both sides prove that they use the same master key
    without ever exposing that key. Only random values are exchanged.
    Not all commands require authentication.    
    If you want to authenticate for an application you must call SelectApplication() first.
    If you select application 0x000000 pi_Key must be the PICC master key (set u8_KeyNo = 0),
    otherwise one of the up to 14 application keys is chosen with u8_KeyNo.
    IMPORTANT: If the card expects the 3K3DES default key you must pass a 3K3DES key full of 24 zeroes,
    although this is in reality a simple DES key (K1 == K2 == K3). Otherwise the session key is calculated wrong.
**************************************************************************/
bool UltralightC::Authenticate(byte u8_KeyNo, DESFireKey* pi_Key)
{
    if (mu8_DebugLevel > 0)
    {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** Authenticate(KeyNo= %d, Key= ", u8_KeyNo);
        Utils::Print(s8_Buf);
        pi_Key->PrintKey();
        Utils::Print(")\r\n");
    }

    byte u8_Command;
    switch (pi_Key->GetKeyType())
    { 
        case DF_KEY_AES:    u8_Command = DFEV1_INS_AUTHENTICATE_AES; break;
        case DF_KEY_2K3DES:
        case DF_KEY_3K3DES: u8_Command = DFEV1_INS_AUTHENTICATE_ISO; break;
        default:
            Utils::Print("Invalid key\r\n");
            return false;
    }

    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_KeyNo);

    // Request a random of 16 byte, but depending of the key the PICC may also return an 8 byte random
    DESFireStatus e_Status;
    byte u8_RndB_enc[16]; // encrypted random B
    int s32_Read = DataExchange(u8_Command, &i_Params, u8_RndB_enc, 16, &e_Status, MAC_None);
    if (e_Status != ST_MoreFrames || (s32_Read != 8 && s32_Read != 16))
    {
        Utils::Print("Authentication failed (1)\r\n");
        return false;
    }

    int s32_RandomSize = s32_Read;

    byte u8_RndB[16];  // decrypted random B
    pi_Key->ClearIV(); // Fill IV with zeroes !ONLY ONCE HERE!
    if (!pi_Key->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RndB, u8_RndB_enc, s32_RandomSize))
        return false;  // key not set

    byte u8_RndB_rot[16]; // rotated random B
    Utils::RotateBlockLeft(u8_RndB_rot, u8_RndB, s32_RandomSize);

    byte u8_RndA[16];
    Utils::GenerateRandom(u8_RndA, s32_RandomSize);

    TX_BUFFER(i_RndAB, 32); // (randomA + rotated randomB)
    i_RndAB.AppendBuf(u8_RndA,     s32_RandomSize);
    i_RndAB.AppendBuf(u8_RndB_rot, s32_RandomSize);

    TX_BUFFER(i_RndAB_enc, 32); // encrypted (randomA + rotated randomB)
    i_RndAB_enc.SetCount(2*s32_RandomSize);
    if (!pi_Key->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, i_RndAB_enc, i_RndAB, 2*s32_RandomSize))
        return false;

    if (mu8_DebugLevel > 0)
    {
        Utils::Print("* RndB_enc:  ");
        Utils::PrintHexBuf(u8_RndB_enc,  s32_RandomSize, LF);
        Utils::Print("* RndB:      ");
        Utils::PrintHexBuf(u8_RndB,      s32_RandomSize, LF);
        Utils::Print("* RndB_rot:  ");
        Utils::PrintHexBuf(u8_RndB_rot,  s32_RandomSize, LF);
        Utils::Print("* RndA:      ");
        Utils::PrintHexBuf(u8_RndA,      s32_RandomSize, LF);
        Utils::Print("* RndAB:     ");
        Utils::PrintHexBuf(i_RndAB,      2*s32_RandomSize, LF);
        Utils::Print("* RndAB_enc: ");
        Utils::PrintHexBuf(i_RndAB_enc,  2*s32_RandomSize, LF);
    }

    byte u8_RndA_enc[16]; // encrypted random A
    s32_Read = DataExchange(DF_INS_ADDITIONAL_FRAME, &i_RndAB_enc, u8_RndA_enc, s32_RandomSize, &e_Status, MAC_None);
    if (e_Status != ST_Success || s32_Read != s32_RandomSize)
    {
        Utils::Print("Authentication failed (2)\r\n");
        return false;
    }

    byte u8_RndA_dec[16]; // decrypted random A
    if (!pi_Key->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RndA_dec, u8_RndA_enc, s32_RandomSize))
        return false;

    byte u8_RndA_rot[16]; // rotated random A
    Utils::RotateBlockLeft(u8_RndA_rot, u8_RndA, s32_RandomSize);   

    if (mu8_DebugLevel > 0)
    {
        Utils::Print("* RndA_enc:  ");
        Utils::PrintHexBuf(u8_RndA_enc, s32_RandomSize, LF);
        Utils::Print("* RndA_dec:  ");
        Utils::PrintHexBuf(u8_RndA_dec, s32_RandomSize, LF);
        Utils::Print("* RndA_rot:  ");
        Utils::PrintHexBuf(u8_RndA_rot, s32_RandomSize, LF);
    }

    // Last step: Check if the received random A is equal to the sent random A.
    if (memcmp(u8_RndA_dec, u8_RndA_rot, s32_RandomSize) != 0)
    {
        Utils::Print("Authentication failed (3)\r\n");
        return false;
    }

    // The session key is composed from RandA and RndB
    TX_BUFFER(i_SessKey, 24);
    i_SessKey.AppendBuf(u8_RndA, 4);
    i_SessKey.AppendBuf(u8_RndB, 4);

    if (pi_Key->GetKeySize() > 8) // the following block is not required for simple DES
    {
        switch (pi_Key->GetKeyType())
        {  
            case DF_KEY_2K3DES:
                i_SessKey.AppendBuf(u8_RndA + 4, 4);
                i_SessKey.AppendBuf(u8_RndB + 4, 4);
                break;
                
            case DF_KEY_3K3DES:
                i_SessKey.AppendBuf(u8_RndA +  6, 4);
                i_SessKey.AppendBuf(u8_RndB +  6, 4);
                i_SessKey.AppendBuf(u8_RndA + 12, 4);
                i_SessKey.AppendBuf(u8_RndB + 12, 4);
                break;
    
            case DF_KEY_AES:
                i_SessKey.AppendBuf(u8_RndA + 12, 4);
                i_SessKey.AppendBuf(u8_RndB + 12, 4);
                break;
    
            default: // avoid stupid gcc compiler warning
                break;
        }
    }
       
    if (pi_Key->GetKeyType() == DF_KEY_AES) mpi_SessionKey = &mi_AesSessionKey;
    else                                    mpi_SessionKey = &mi_DesSessionKey;
    
    if (!mpi_SessionKey->SetKeyData(i_SessKey, i_SessKey.GetCount(), 0) ||
        !mpi_SessionKey->GenerateCmacSubkeys())
        return false;

    if (mu8_DebugLevel > 0)
    {
        Utils::Print("* SessKey:   ");
        mpi_SessionKey->PrintKey(LF);
    }

    mu8_LastAuthKeyNo = u8_KeyNo;   
    return true;
}

/**************************************************************************
    Reads a block of data from a Standard Data File or a Backup Data File.
    If (s32_Offset + s32_Length > file length) you will get a LimitExceeded error.
    If the file permissins are not set to AR_FREE you must authenticate either
    with the key in e_ReadAccess or the key in e_ReadAndWriteAccess.   
**************************************************************************/
bool UltralightC::ReadPageData(byte u8_PageID, uint32_t[4] u32_DataBuffer)
{
    if (mu8_DebugLevel > 0)
    {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** ReadPageData(ID= %d)\r\n", u8_PageID);
        Utils::Print(s8_Buf);
    }

    int s32_Count = min(s32_Length, 48); // the maximum that can be transferred in one frame (must be a multiple of 16 if encryption is used)

    TX_BUFFER(i_Params, 7);
    i_Params.AppendUint8 (u8_FileID);
    i_Params.AppendUint24(s32_Offset); // only the low 3 bytes are used
    i_Params.AppendUint24(s32_Count);  // only the low 3 bytes are used
    
    DESFireStatus e_Status;
    int s32_Read = DataExchange(DF_INS_READ_DATA, &i_Params, u8_DataBuffer, s32_Count, &e_Status, MAC_TmacRmac);
    if (e_Status != ST_Success || s32_Read <= 0)
        return false; // ST_MoreFrames is not allowed here!

    s32_Length    -= s32_Read;
    s32_Offset    += s32_Read;
    u8_DataBuffer += s32_Read;
    
    return true;
}

/**************************************************************************
    Writes data to a Standard Data File or a Backup Data File.
    If the file permissins are not set to AR_FREE you must authenticate either
    with the key in e_WriteAccess or the key in e_ReadAndWriteAccess.
**************************************************************************/
bool UltralightC::WritePageData(byte u8_PageID, uint32_t data)
{
    if (mu8_DebugLevel > 0)
    {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** WriteFileData(ID= %d, Data= %d)\r\n", u8_PageID, data);
        Utils::Print(s8_Buf);
    }

    int s32_Count = min(s32_Length, MAX_FRAME_SIZE - 8); // DF_INS_WRITE_DATA + u8_FileID + s32_Offset + s32_Count = 8 bytes
          
    TX_BUFFER(i_Params, MAX_FRAME_SIZE); 
    i_Params.AppendUint8 (u8_FileID);
    i_Params.AppendUint24(s32_Offset); // only the low 3 bytes are used
    i_Params.AppendUint24(s32_Count);  // only the low 3 bytes are used
    i_Params.AppendBuf(u8_DataBuffer, s32_Count);

    DESFireStatus e_Status;
    int s32_Read = DataExchange(DF_INS_WRITE_DATA, &i_Params, NULL, 0, &e_Status, MAC_TmacRmac);
    if (e_Status != ST_Success || s32_Read != 0)
        return false; // ST_MoreFrames is not allowed here!

    s32_Length    -= s32_Count;
    s32_Offset    += s32_Count;
    u8_DataBuffer += s32_Count;

    return true;
}

// ########################################################################
// ####                      LOW LEVEL FUNCTIONS                      #####
// ########################################################################

// If this value is != 0, the PN532 has returned an error code while executing the latest command.
// Typically a Timeout error (Value = 0x01) means that the card is too far away from the reader.
// Interestingly a timeout occurres typically when authenticating. 
// The commands that are executed first (GetKeyVersion and SelectApplication) execute without problems.
// But it when it comes to Authenticate() the card suddenly does not respond anymore -> Timeout from PN532.
// Conclusion: It seems that a UltralightC card increases its power consumption in the moment when encrypting data,
// so when it is too far away from the antenna -> the connection dies.
byte UltralightC::GetLastPN532Error()
{
    return mu8_LastPN532Error;
}

/**************************************************************************
    Sends data to the card and receives the response.
    u8_Command    = UltralightC command without additional paramaters
    pi_Command    = UltralightC command + possible additional paramaters that will not be encrypted
    pi_Params     = UltralightC command parameters that may be encrypted (MAC_Tcrypt). This paramater may also be null.
    u8_RecvBuf    = buffer that receives the received data (should be the size of the expected recv data)
   s32_RecvSize   = buffer size of u8_RecvBuf
    pe_Status     = if (!= NULL) -> receives the status byte
    e_Mac         = defines CMAC calculation
    returns the byte count that has been read into u8_RecvBuf or -1 on error
**************************************************************************/
bool UltralightC::Communicate(byte command, byte argc, byte* argv, byte respsize, byte* respv)
{
    
}

