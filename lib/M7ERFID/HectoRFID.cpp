/**
 * @file HectoRFID.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>
#include "HectoRFID.h"

static const uint16_t crctable[] = {
    0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50a5,
    0x60c6,
    0x70e7,
    0x8108,
    0x9129,
    0xa14a,
    0xb16b,
    0xc18c,
    0xd1ad,
    0xe1ce,
    0xf1ef,
};

/**
 * @brief Construct a new RFID::RFID object
 *
 */
RFID::RFID(void)
{
  // Constructor
}

// Initialize the Serial port
void RFID::begin(Stream *port, ThingMagic_Module_t moduleType)
{
  _nanoSerial = port;
  _moduleType = moduleType;
}
void RFID::getVersion(void)
{
  sendMessage(TMR_SR_OPCODE_VERSION);
}
void RFID::setReadPower(int16_t powerSetting)
{
  if (powerSetting > 2700)
    powerSetting = 2700; // Limit to 27dBm

  uint8_t size = sizeof(powerSetting);
  uint8_t data[size];
  for (uint8_t x = 0; x < size; x++)
    data[x] = (uint8_t)(powerSetting >> (8 * (size - 1 - x)));

  sendMessage(TMR_SR_OPCODE_SET_READ_TX_POWER, data, size);
}
void RFID::getReadPower()
{
  uint8_t data[] = {0x00};

  sendMessage(TMR_SR_OPCODE_GET_READ_TX_POWER, data, sizeof(data));
}
void RFID::setWritePower(int16_t powerSetting)
{
  if (powerSetting > 2700)
    powerSetting = 2700; // Limit to 27dBm

  uint8_t size = sizeof(powerSetting);
  uint8_t data[size];
  for (uint8_t x = 0; x < size; x++)
    data[x] = (uint8_t)(powerSetting >> (8 * (size - 1 - x)));

  sendMessage(TMR_SR_OPCODE_SET_WRITE_TX_POWER, data, size);
}
void RFID::getWritePower()
{
  uint8_t data[] = {0x00};

  sendMessage(TMR_SR_OPCODE_GET_WRITE_TX_POWER, data, sizeof(data));
}
void RFID::setRegion(uint8_t region)
{
  if (region == REGION_NORTHAMERICA && _moduleType == ThingMagic_M6E_NANO)
    region = REGION_NORTHAMERICA2;

  sendMessage(TMR_SR_OPCODE_SET_REGION, &region, sizeof(region));
}
void RFID::setTagProtocol(uint8_t protocol)
{
  uint8_t data[2];
  data[0] = 0; // Opcode expects 16-bits
  data[1] = protocol;

  sendMessage(TMR_SR_OPCODE_SET_TAG_PROTOCOL, data, sizeof(data));
}

void RFID::setAntennaPort(void)
{
  uint8_t configBlob[] = {0x01, 0x01}; // TX port = 1, RX port = 1
  sendMessage(TMR_SR_OPCODE_SET_ANTENNA_PORT, configBlob, sizeof(configBlob));
}

void RFID::startReading(void)
{
  disableReadFilter(); // Don't filter for a specific tag, read all tags
  uint8_t configBlob[] = {0x00, 0x00, 0x01, 0x22, 0x00, 0x00, 0x05, 0x07, 0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF};

  /*
    //Timeout should be zero for true continuous reading
    SETU16(newMsg, i, 0);
    SETU8(newMsg, i, (uint8_t)0x1); // TM Option 1, for continuous reading
    SETU8(newMsg, i, (uint8_t)TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE); // sub command opcode
    SETU16(newMsg, i, (uint16_t)0x0000); // search flags, only 0x0001 is supported
    SETU8(newMsg, i, (uint8_t)TMR_TAG_PROTOCOL_GEN2); // protocol ID
  */

  sendMessage(TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP, configBlob, sizeof(configBlob));
}
void RFID::stopReading(void)
{
  uint8_t configBlob[] = {0x00, 0x00, 0x02};

  sendMessage(TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP, configBlob, sizeof(configBlob), false);
}
void RFID::enableReadFilter(void)
{
  setReaderConfiguration(0x0C, 0x01);
}
void RFID::disableReadFilter(void)
{
  setReaderConfiguration(0x0C, 0x00);
}
void RFID::setReaderConfiguration(uint8_t option1, uint8_t option2)
{
  uint8_t data[3];

  // These are parameters gleaned from inspecting the 'Transport Logs' of the Universal Reader Assistant
  // And from serial_reader_l3.c
  data[0] = 1; // Key value form of command
  data[1] = option1;
  data[2] = option2;

  sendMessage(TMR_SR_OPCODE_SET_READER_OPTIONAL_PARAMS, data, sizeof(data));
}
void RFID::getOptionalParameters(uint8_t option1, uint8_t option2)
{
  uint8_t data[2];
  data[0] = option1;
  data[1] = option2;
  sendMessage(TMR_SR_OPCODE_GET_READER_OPTIONAL_PARAMS, data, sizeof(data));
}

uint8_t RFID::parseResponse(void)
{
  // See http://www.thingmagic.com/images/Downloads/Docs/AutoConfigTool_1.2-UserGuide_v02RevA.pdf
  // for a breakdown of the response packet

  // Example response:
  // FF  28  22  00  00  10  00  1B  01  FF  01  01  C4  11  0E  16
  // 40  00  00  01  27  00  00  05  00  00  0F  00  80  30  00  00
  // 00  00  00  00  00  00  00  00  00  15  45  E9  4A  56  1D
  //   [0] FF = Header
  //   [1] 28 = Message length
  //   [2] 22 = OpCode
  //   [3, 4] 00 00 = Status
  //   [5 to 11] 10 00 1B 01 FF 01 01 = RFU 7 bytes
  //   [12] C4 = RSSI
  //   [13] 11 = Antenna ID (4MSB = TX, 4LSB = RX)
  //   [14, 15, 16] 0E 16 40 = Frequency in kHz
  //   [17, 18, 19, 20] 00 00 01 27 = Timestamp in ms since last keep alive msg
  //   [21, 22] 00 00 = phase of signal tag was read at (0 to 180)
  //   [23] 05 = Protocol ID
  //   [24, 25] 00 00 = Number of bits of embedded tag data [M bytes]
  //   [26 to M] (none) = Any embedded data
  //   [26 + M] 0F = RFU reserved future use
  //   [27, 28 + M] 00 80 = EPC Length [N bytes]  (bits in EPC including PC and CRC bits). 128 bits = 16 bytes
  //   [29, 30 + M] 30 00 = Tag EPC Protocol Control (PC) bits
  //   [31 to 42 + M + N] 00 00 00 00 00 00 00 00 00 00 15 45 = EPC ID
  //   [43, 44 + M + N] 45 E9 = EPC CRC
  //   [45, 46 + M + N] 56 1D = Message CRC

  uint8_t msgLength = msg[1] + 7; // Add 7 (the header, length, opcode, status, and CRC) to the LEN field to get total bytes
  uint8_t opCode = msg[2];

  // Check the CRC on this response
  uint16_t messageCRC = calculateCRC(&msg[1], msgLength - 3); // Ignore header (start spot 1), remove 3 bytes (header + 2 CRC)
  if ((msg[msgLength - 2] != (messageCRC >> 8)) || (msg[msgLength - 1] != (messageCRC & 0xFF)))
  {
    return (ERROR_CORRUPT_RESPONSE);
  }

  if (opCode == TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE) // opCode = 0x22
  {
    // Based on the record length identify if this is a tag record, a temperature sensor record, or a keep-alive?
    if (msg[1] == 0x00) // Keep alive
    {
      // We have a Read cycle reset/keep-alive message
      // Sent once per second
      uint16_t statusMsg = 0;
      for (uint8_t x = 0; x < 2; x++)
        statusMsg |= (uint32_t)msg[3 + x] << (8 * (1 - x));

      if (statusMsg == 0x0400)
      {
        return (RESPONSE_IS_KEEPALIVE);
      }
      else if (statusMsg == 0x0504)
      {
        return (RESPONSE_IS_TEMPTHROTTLE);
      }
      else if (statusMsg == 0x0505)
      {
        return (RESPONSE_IS_HIGHRETURNLOSS);
      }
      else
      {
        return (RESPONSE_IS_UNKNOWN);
      }
    }
    else if (msg[1] == 0x08) // Unknown
    {
      return (RESPONSE_IS_UNKNOWN);
    }
    else if (msg[1] == 0x0a) // temperature
    {
      return (RESPONSE_IS_TEMPERATURE);
    }
    else // Full tag record
    {
      // This is a full tag response
      // User can now pull out RSSI, frequency of tag, timestamp, EPC, Protocol control bits, EPC CRC, CRC
      return (RESPONSE_IS_TAGFOUND);
    }
  }
  else
  {
    if (_printDebug == true)
    {
      _debugSerial->print(F("Unknown opcode in response: 0x"));
      _debugSerial->println(opCode, HEX);
    }
    return (ERROR_UNKNOWN_OPCODE);
  }
}

uint8_t RFID::getTagEPCBytes(void)
{
  uint16_t epcBits = 0; // Number of bits of EPC (including PC, EPC, and EPC CRC)

  uint8_t tagDataBytes = getTagDataBytes(); // We need this offset

  for (uint8_t x = 0; x < 2; x++)
    epcBits |= (uint16_t)msg[27 + tagDataBytes + x] << (8 * (1 - x));
  uint8_t epcBytes = epcBits / 8;
  epcBytes -= 4; // Ignore the first two bytes and last two bytes

  return (epcBytes);
}
uint8_t RFID::getTagDataBytes(void)
{
  // Number of bits of embedded tag data
  uint8_t tagDataLength = 0;
  for (uint8_t x = 0; x < 2; x++)
    tagDataLength |= (uint16_t)msg[24 + x] << (8 * (1 - x));
  uint8_t tagDataBytes = tagDataLength / 8;
  if (tagDataLength % 8 > 0)
    tagDataBytes++; // Ceiling trick

  return (tagDataBytes);
}
uint32_t RFID::getTagFreq(void)
{
  // Frequency of the tag detected is loaded over three bytes
  uint32_t freq = 0;
  for (uint8_t x = 0; x < 3; x++)
    freq |= (uint32_t)msg[14 + x] << (8 * (2 - x));

  return (freq);
}
int8_t RFID::getTagRSSI(void)
{
  return (msg[12] - 256);
}
bool RFID::check(void)
{
  while (_nanoSerial->available())
  {
    uint8_t incomingData = _nanoSerial->read();

    // Wait for header byte
    if (_head == 0 && incomingData != 0xFF)
    {
      // Do nothing. Ignore this byte because we need a start byte
    }
    else
    {
      // Load this value into the array
      msg[_head++] = incomingData;

      _head %= MAX_MSG_SIZE; // Wrap variable

      if ((_head > 0) && (_head == msg[1] + 7))
      {
        // We've got a complete sentence!

        // Erase the remainder of the array
        for (uint8_t x = _head; x < MAX_MSG_SIZE; x++)
          msg[x] = 0;

        _head = 0; // Reset

        // Used for debugging: Does the user want us to print the command to serial port?
        if (_printDebug == true)
        {
          _debugSerial->print(F("response: "));
          printMessageArray();
        }

        return (true);
      }
    }
  }

  return (false);
}
uint8_t RFID::readTagEPC(uint8_t *epc, uint8_t &epcLength, uint16_t timeOut)
{
  uint8_t bank = 0x01;    // User data bank
  uint8_t address = 0x02; // Starts at 2

  return (readData(bank, address, epc, epcLength, timeOut));
}
uint8_t RFID::writeTagEPC(char *newID, uint8_t newIDLength, uint16_t timeOut)
{
  uint8_t bank = 0x01;    // EPC memory
  uint8_t address = 0x02; // EPC starts at spot 4

  return (writeData(bank, address, (uint8_t *)newID, newIDLength, timeOut));
}
uint8_t RFID::readData(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t &dataLengthRead, uint16_t timeOut)
{
  // Bank 0
  // response: [00] [08] [28] [00] [00] [10] [00] [00] [EE] [FF] [11] [22] [12] [34] [56] [78]
  //[EE] [FF] [11] [22] = Kill pw
  //[12] [34] [56] [78] = Access pw

  // Bank 1
  // response: [00] [08] [28] [00] [00] [10] [00] [00] [28] [F0] [14] [00] [AA] [BB] [CC] [DD]
  //[28] [F0] = CRC
  //[14] [00] = PC
  //[AA] [BB] [CC] [DD] = EPC

  // Bank 2
  // response: [00] [18] [28] [00] [00] [10] [00] [00] [E2] [00] [34] [12] [01] [6E] [FE] [00] [03] [7D] [9A] [A3] [28] [05] [01] [69] [10] [05] [5F] [FB] [FF] [FF] [DC] [00]
  //[E2] = CIsID
  //[00] [34] [12] = Vendor ID = 003, Model ID == 412
  //[01] [6E] [FE] [00] [03] [7D] [9A] [A3] [28] [05] [01] [69] [10] [05] [5F] [FB] [FF] [FF] [DC] [00] = Unique ID (TID)

  // Bank 3
  // response: [00] [40] [28] [00] [00] [10] [00] [00] [41] [43] [42] [44] [45] [46] [00] [00] [00] [00] [00] [00] ...
  // User data

  uint8_t data[11];

  // Insert timeout
  data[0] = timeOut >> 8 & 0xFF; // Timeout msB in ms
  data[1] = timeOut & 0xFF;      // Timeout lsB in ms

  // A previous version of this library did not include these 3 bytes. It works
  // fine with the M6E, but not the M7E. After reverse engineering the protocol
  // from the Mercury API (TMR_SR_cmdGEN2ReadTagData() in serial_reader_l3.c),
  // it was found that these 3 bytes are required. Not really sure what they do,
  // but it seems to work!
  data[2] = 0x10; // Option byte
  data[3] = 0x00; // Metadata MSB
  data[4] = 0x00; // Metadata LSB

  data[5] = bank; // Bank

  // Splice address into array
  for (uint8_t x = 0; x < sizeof(address); x++)
    data[6 + x] = address >> (8 * (3 - x)) & 0xFF;

  // The last byte is the number of 16-bit words to read. If it's set to zero,
  // then it will read the entire bank. We could set this to dataLengthRead / 2,
  // but it's easier to just set it to zero and truncate the response later.
  // That also helps if dataLengthRead differs from the actual the bank size,
  // which can cause the read to fail entirely.
  data[10] = 0x00;
  // data[10] = dataLengthRead / 2;

  sendMessage(TMR_SR_OPCODE_READ_TAG_DATA, data, sizeof(data), timeOut);

  if (msg[0] == ALL_GOOD) // We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
    {
      // Offset by 3 for the returned option and metadata bytes
      uint8_t responseLength = msg[1] - 3;

      if (responseLength < dataLengthRead) // User wants us to read more than we have available
        dataLengthRead = responseLength;

      // There is a case here where responseLegnth is more than dataLengthRead, in which case we ignore (don't load) the additional bytes
      // Load limited response data into caller's array
      for (uint8_t x = 0; x < dataLengthRead; x++)
        // Data starts at byte 8 (header (1), size (1), opcode (1), status (2), option (1), metadata (2))
        dataRead[x] = msg[8 + x];

      return (RESPONSE_SUCCESS);
    }
  }

  // Else - msg[0] was timeout or other
  dataLengthRead = 0; // Inform caller that we weren't able to read anything

  return (RESPONSE_FAIL);
}
uint8_t RFID::writeData(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut)
{
  // Example: FF  0A  24  03  E8  00  00  00  00  00  03  00  EE  58  9D
  // FF 0A 24 = Header, LEN, Opcode
  // 03 E8 = Timeout in ms
  // 00 = Option initialize
  // 00 00 00 00 = Address
  // 03 = Bank
  // 00 EE = Data
  // 58 9D = CRC

  uint8_t data[8 + dataLengthToRecord];

  // Pre-load array options
  data[0] = timeOut >> 8 & 0xFF; // Timeout msB in ms
  data[1] = timeOut & 0xFF;      // Timeout lsB in ms
  data[2] = 0x00;                // Option initialize

  // Splice address into array
  for (uint8_t x = 0; x < sizeof(address); x++)
    data[3 + x] = address >> (8 * (3 - x)) & 0xFF;

  // Bank 0 = Passwords
  // Bank 1 = EPC Memory Bank
  // Bank 2 = TID
  // Bank 3 = User Memory
  data[7] = bank;

  // Splice data into array
  for (uint8_t x = 0; x < dataLengthToRecord; x++)
    data[8 + x] = dataToRecord[x];

  sendMessage(TMR_SR_OPCODE_WRITE_TAG_DATA, data, sizeof(data), timeOut);

  if (msg[0] == ALL_GOOD) // We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
      return (RESPONSE_SUCCESS);
  }

  // Else - msg[0] was timeout or other
  return (RESPONSE_FAIL);
}
uint8_t RFID::readUserData(uint8_t *userData, uint8_t &userDataLength, uint16_t timeOut)
{
  uint8_t bank = 0x03;    // User data bank
  uint8_t address = 0x00; // Starts at 0

  return (readData(bank, address, userData, userDataLength, timeOut));
}
uint8_t RFID::writeUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut)
{
  uint8_t bank = 0x03; // User memory
  uint8_t address = 0x00;

  return (writeData(bank, address, userData, userDataLength, timeOut));
}
uint8_t RFID::readKillPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00;    // Passwords bank
  uint8_t address = 0x00; // Kill password address

  return (readData(bank, address, password, passwordLength, timeOut));
}
uint8_t RFID::writeKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00;    // Passwords bank
  uint8_t address = 0x00; // Kill password address

  return (writeData(bank, address, password, passwordLength, timeOut));
}
uint8_t RFID::readAccessPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00;    // Passwords bank
  uint8_t address = 0x02; // Access password address

  return (readData(bank, address, password, passwordLength, timeOut));
}
uint8_t RFID::writeAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00;    // Passwords bank
  uint8_t address = 0x02; // Access password address

  return (writeData(bank, address, password, passwordLength, timeOut));
}
uint8_t RFID::readTID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut)
{
  uint8_t bank = 0x02; // Bank for TID
  uint8_t address = 0x02;

  return (readData(bank, address, tid, tidLength, timeOut));
}

uint8_t RFID::readUID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut)
{
  uint8_t bank = 0x02;    // Bank for TID
  uint8_t address = 0x02; // UID of the TID starts at 4

  return (readData(bank, address, tid, tidLength, timeOut));
}
uint8_t RFID::killTag(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t data[4 + passwordLength];

  data[0] = timeOut >> 8 & 0xFF; // Timeout msB in ms
  data[1] = timeOut & 0xFF;      // Timeout lsB in ms
  data[2] = 0x00;                // Option initialize

  // Splice password into array
  for (uint8_t x = 0; x < passwordLength; x++)
    data[3 + x] = password[x];

  data[3 + passwordLength] = 0x00; // RFU

  sendMessage(TMR_SR_OPCODE_KILL_TAG, data, sizeof(data), timeOut);

  if (msg[0] == ALL_GOOD) // We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
      return (RESPONSE_SUCCESS);
  }

  // Else - msg[0] was timeout or other
  return (RESPONSE_FAIL);
}
void RFID::sendMessage(uint8_t opcode, uint8_t *data, uint8_t size, uint16_t timeOut, boolean waitForResponse)
{
  msg[1] = size; // Load the length of this operation into msg array
  msg[2] = opcode;

  // Copy the data into msg array
  for (uint8_t x = 0; x < size; x++)
    msg[3 + x] = data[x];

  sendCommand(timeOut, waitForResponse); // Send and wait for response
}
void RFID::sendCommand(uint16_t timeOut, boolean waitForResponse)
{
  msg[0] = 0xFF; // Universal header
  uint8_t messageLength = msg[1];

  uint8_t opcode = msg[2]; // Used to see if response from module has the same opcode

  // Attach CRC
  uint16_t crc = calculateCRC(&msg[1], messageLength + 2); // Calc CRC starting from spot 1, not 0. Add 2 for LEN and OPCODE bytes.
  msg[messageLength + 3] = crc >> 8;
  msg[messageLength + 4] = crc & 0xFF;

  // Used for debugging: Does the user want us to print the command to serial port?
  if (_printDebug == true)
  {
    _debugSerial->print(F("sendCommand: "));
    printMessageArray();
  }

  // Remove anything in the incoming buffer
  // TODO this is a bad idea if we are constantly readings tags
  while (_nanoSerial->available())
    _nanoSerial->read();

  // Send the command to the module
  for (uint8_t x = 0; x < messageLength + 5; x++)
    _nanoSerial->write(msg[x]);

  // There are some commands (setBaud) that we can't or don't want the response
  if (waitForResponse == false)
  {
    _nanoSerial->flush(); // Wait for serial sending to complete
    return;
  }

  // For debugging, probably remove
  // for (uint8_t x = 0 ; x < 100 ; x++) msg[x] = 0;

  // Wait for response with timeout
  uint32_t startTime = millis();
  while (_nanoSerial->available() == false)
  {
    if (millis() - startTime > timeOut)
    {
      if (_printDebug == true)
        _debugSerial->println(F("Time out 1: No response from module"));
      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }
    delay(1);
  }

  // Layout of response in data array:
  // [0] [1] [2] [3]      [4]      [5] [6]  ... [LEN+4] [LEN+5] [LEN+6]
  // FF  LEN OP  STATUSHI STATUSLO xx  xx   ... xx      CRCHI   CRCLO
  messageLength = MAX_MSG_SIZE - 1; // Make the max length for now, adjust it when the actual len comes in
  uint8_t spot = 0;
  while (spot < messageLength)
  {
    if (millis() - startTime > timeOut)
    {
      if (_printDebug == true)
        _debugSerial->println(F("Time out 2: Incomplete response"));

      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }

    if (_nanoSerial->available())
    {
      msg[spot] = _nanoSerial->read();

      if (spot == 1)                // Grab the length of this response (spot 1)
        messageLength = msg[1] + 7; // Actual length of response is ? + 7 for extra stuff (header, Length, opcode, 2 status bytes, ..., 2 bytes CRC = 7)

      spot++;

      // There's a case were we miss the end of one message and spill into another message.
      // We don't want spot pointing at an illegal spot in the array
      spot %= MAX_MSG_SIZE; // Wrap condition
    }
  }

  // Used for debugging: Does the user want us to print the command to serial port?
  if (_printDebug == true)
  {
    _debugSerial->print(F("response: "));
    printMessageArray();
  }
  // Check CRC
  crc = calculateCRC(&msg[1], messageLength - 3); // Remove header, remove 2 crc bytes
  if ((msg[messageLength - 2] != (crc >> 8)) || (msg[messageLength - 1] != (crc & 0xFF)))
  {
    msg[0] = ERROR_CORRUPT_RESPONSE;
    if (_printDebug == true)
      _debugSerial->println(F("Corrupt response"));
    return;
  }

  // If crc is ok, check that opcode matches (did we get a response to the command we sent or a different one?)
  if (msg[2] != opcode)
  {
    msg[0] = ERROR_WRONG_OPCODE_RESPONSE;
    if (_printDebug == true)
      _debugSerial->println(F("Wrong opcode response"));
    return;
  }

  // If everything is ok, load all ok into msg array
  msg[0] = ALL_GOOD;
}

void RFID::printMessageArray(void)
{
  if (_printDebug == true) // If user hasn't enabled debug we don't know what port to debug to
  {
    uint8_t amtToPrint = msg[1] + 5;
    if (amtToPrint > MAX_MSG_SIZE)
      amtToPrint = MAX_MSG_SIZE; // Limit this size

    for (uint16_t x = 0; x < amtToPrint; x++)
    {
      _debugSerial->print(" [");
      if (msg[x] < 0x10)
        _debugSerial->print("0");
      _debugSerial->print(msg[x], HEX);
      _debugSerial->print("]");
    }
    _debugSerial->println();
  }
}

uint16_t RFID::calculateCRC(uint8_t *u8Buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc = ((crc << 4) | (u8Buf[i] >> 4)) ^ crctable[crc >> 12];
    crc = ((crc << 4) | (u8Buf[i] & 0x0F)) ^ crctable[crc >> 12];
  }

  return crc;
}

boolean RFID::setupModule(void)
{
  while (_nanoSerial->available())
    _nanoSerial->read();

  getVersion();

  Serial.println(msg[0], HEX);

  if (msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    // This happens if the baud rate is correct but the module is doing a ccontinuous read
    stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }

  // Test the connection
  getVersion();
  if (msg[0] != ALL_GOOD)
    return false; // Something is not right

  // The module has these settings no matter what
  setTagProtocol(); // Set protocol to GEN2

  setAntennaPort(); // Set TX/RX antenna ports to 1

  setRegion(REGION_NORTHAMERICA);

  setReadPower(2000);

  setWritePower(2000);

  return true;
}
