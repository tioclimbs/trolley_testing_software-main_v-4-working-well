#ifndef HECTORFID_H
#define HECTORFID_H

/**
 * @file HectoRFID.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>

#define MAX_MSG_SIZE 255

#define TMR_SR_OPCODE_VERSION 0x03
#define TMR_SR_OPCODE_SET_BAUD_RATE 0x06
#define TMR_SR_OPCODE_READ_TAG_ID_SINGLE 0x21
#define TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE 0x22
#define TMR_SR_OPCODE_WRITE_TAG_ID 0x23
#define TMR_SR_OPCODE_WRITE_TAG_DATA 0x24
#define TMR_SR_OPCODE_KILL_TAG 0x26
#define TMR_SR_OPCODE_READ_TAG_DATA 0x28
#define TMR_SR_OPCODE_CLEAR_TAG_ID_BUFFER 0x2A
#define TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP 0x2F
#define TMR_SR_OPCODE_GET_READ_TX_POWER 0x62
#define TMR_SR_OPCODE_GET_WRITE_TX_POWER 0x64
#define TMR_SR_OPCODE_GET_USER_GPIO_INPUTS 0x66
#define TMR_SR_OPCODE_GET_POWER_MODE 0x68
#define TMR_SR_OPCODE_GET_READER_OPTIONAL_PARAMS 0x6A
#define TMR_SR_OPCODE_GET_PROTOCOL_PARAM 0x6B
#define TMR_SR_OPCODE_SET_ANTENNA_PORT 0x91
#define TMR_SR_OPCODE_SET_TAG_PROTOCOL 0x93
#define TMR_SR_OPCODE_SET_READ_TX_POWER 0x92
#define TMR_SR_OPCODE_SET_WRITE_TX_POWER 0x94
#define TMR_SR_OPCODE_SET_USER_GPIO_OUTPUTS 0x96
#define TMR_SR_OPCODE_SET_REGION 0x97
#define TMR_SR_OPCODE_SET_READER_OPTIONAL_PARAMS 0x9A
#define TMR_SR_OPCODE_SET_PROTOCOL_PARAM 0x9B

#define COMMAND_TIME_OUT 2000

// Define all the ways functions can return
#define ALL_GOOD 0
#define ERROR_COMMAND_RESPONSE_TIMEOUT 1
#define ERROR_CORRUPT_RESPONSE 2
#define ERROR_WRONG_OPCODE_RESPONSE 3
#define ERROR_UNKNOWN_OPCODE 4
#define RESPONSE_IS_TEMPERATURE 5
#define RESPONSE_IS_KEEPALIVE 6
#define RESPONSE_IS_TEMPTHROTTLE 7
#define RESPONSE_IS_TAGFOUND 8
#define RESPONSE_IS_NOTAGFOUND 9
#define RESPONSE_IS_UNKNOWN 10
#define RESPONSE_SUCCESS 11
#define RESPONSE_FAIL 12
#define RESPONSE_IS_HIGHRETURNLOSS 13

// Define the allowed regions - these set the internal freq of the module
#define REGION_NORTHAMERICA 0x01
#define REGION_INDIA 0x04
#define REGION_JAPAN 0x05
#define REGION_CHINA 0x06
#define REGION_EUROPE 0x08
#define REGION_KOREA 0x09
#define REGION_AUSTRALIA 0x0B
#define REGION_NEWZEALAND 0x0C
#define REGION_NORTHAMERICA2 0x0D
#define REGION_NORTHAMERICA3 0x0E
#define REGION_OPEN 0xFF

// Enum for different modules
typedef enum
{
  ThingMagic_M6E_NANO,
  ThingMagic_M7E_HECTO,
} ThingMagic_Module_t;

typedef enum
{
  ThingMagic_PinMode_INPUT = 0,
  ThingMagic_PinMode_OUTPUT = 1
} ThingMagic_PinMode_t;

class RFID
{
public:
  /**
   * @brief Construct a new RFID object
   *
   */
  RFID(void);

  // Variables

  // message arre used for all communication to and from the module
  uint8_t msg[MAX_MSG_SIZE];

  /**
   * @brief Sets which serial port to use
   *
   * @param port Serial port to use
   * @param moduleType M7E_HECTO or M6E_NANO
   */
  void begin(Stream *port, ThingMagic_Module_t moduleType = ThingMagic_M7E_HECTO);

  /**
   * @brief Set the Baud rate of the RFID module
   *
   * @param baudRate desired baud rate
   */
  void setBaud(long baudRate);
  /**
   * @brief Get the Version number from module
   *
   */
  void getVersion(void);
  /**
   * @brief Set the Read Power of the module
   *
   * @param powerSetting the power setting max power is 2700 = 27dBm
   */
  void setReadPower(int16_t powerSetting);
  /**
   * @brief Get the Read Power from the module
   *
   */
  void getReadPower();
  /**
   * @brief Set the Write Power of the module
   *
   * @param powerSetting the power setting max power is 2700 = 27dBm
   */
  void setWritePower(int16_t powerSetting);
  /**
   * @brief Get the Write Power of the module
   *
   */
  void getWritePower();
  /**
   * @brief Set the correct frequercy for the given region
   *
   * @param region Geographi region
   */
  void setRegion(uint8_t region);
  /**
   * @brief Set the Tag Protocol of the module
   *
   * @param protocol
   * @details TMR_TAG_PROTOCOL_GEN2 is the defaut protocol
   */
  void setTagProtocol(uint8_t protocol = 0x05);
  /**
   * @brief Begin scanning for tags
   *
   */
  void startReading(void); // Disable filtering and start reading continuously
  /**
   * @brief Stops continuous reading
   *
   */
  void stopReading(void); // Stops continuous read. Give 1000 to 2000ms for the module to stop reading.
  /**
   * @brief
   *
   */
  void enableReadFilter(void);
  /**
   * @brief
   *
   */
  void disableReadFilter(void);

  /**
   * @brief Set the Reader Configuration object
   *
   * @param option1
   * @param option2
   */
  void setReaderConfiguration(uint8_t option1, uint8_t option2);
  /**
   * @brief Get the Optional Parameters object
   *
   * @param option1
   * @param option2
   */
  void getOptionalParameters(uint8_t option1, uint8_t option2);
  /**
   * @brief Set the Protocol Parameters 
   *
   */
  void setProtocolParameters(void);

  /**
   * @brief Set the Antenna Port 
   * 
   */
  void setAntennaPort(void);
  /**
   * @brief Get the Protocol Parameters object
   *
   * @param option1
   * @param option2
   */
  void getProtocolParameters(uint8_t option1, uint8_t option2);
  /**
   * @brief
   *
   * @return uint8_t
   */
  uint8_t parseResponse(void);

  /**
   * @brief Get the Tag E P C Bytes object
   *
   * @return uint8_t
   */
  uint8_t getTagEPCBytes(void); // Pull number of EPC data bytes from record response.
  /**
   * @brief Get the Tag Data Bytes object
   *
   * @return uint8_t
   */
  uint8_t getTagDataBytes(void); // Pull number of tag data bytes from record response. Often zero.
  /**
   * @brief
   *
   */
  uint32_t getTagFreq(void); // Pull Freq value from full record response
  /**
   * @brief Get the Tag R S S I object
   *
   * @return int8_t
   */
  int8_t getTagRSSI(void); // Pull RSSI value from full record response

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool check(void);

  /**
   * @brief
   *
   * @param epc
   * @param epcLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readTagEPC(uint8_t *epc, uint8_t &epcLength, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param newID
   * @param newIDLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t writeTagEPC(char *newID, uint8_t newIDLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param bank
   * @param address
   * @param dataRead
   * @param dataLengthRead
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readData(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t &dataLengthRead, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param bank
   * @param address
   * @param dataToRecord
   * @param dataLengthToRecord
   * @param timeOut
   * @return uint8_t
   */
  uint8_t writeData(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param userData
   * @param userDataLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readUserData(uint8_t *userData, uint8_t &userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param userData
   * @param userDataLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t writeUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param password
   * @param passwordLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readKillPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param password
   * @param passwordLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t writeKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param password
   * @param passwordLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readAccessPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param password
   * @param passwordLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t writeAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param tid
   * @param tidLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readTID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut = COMMAND_TIME_OUT);
  /**
   * @brief
   *
   * @param tid
   * @param tidLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t readUID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param password
   * @param passwordLength
   * @param timeOut
   * @return uint8_t
   */
  uint8_t killTag(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  /**
   * @brief
   *
   * @param opcode
   * @param data
   * @param size
   * @param timeOut
   * @param waitForResponse
   */
  void sendMessage(uint8_t opcode, uint8_t *data = 0, uint8_t size = 0, uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);
  /**
   * @brief
   *
   * @param timeOut
   * @param waitForResponse
   */
  void sendCommand(uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);

  /**
   * @brief
   *
   */
  void printMessageArray(void);

  /**
   * @brief
   *
   * @param u8Buf
   * @param len
   * @return uint16_t
   */
  uint16_t calculateCRC(uint8_t *u8Buf, uint8_t len);

  boolean setupModule(void);

private:
  Stream *_nanoSerial; // The generic connection to user's chosen serial hardware

  Stream *_debugSerial = &Serial; // The stream to send debug messages to if enabled

  uint8_t _head = 0; // Tracks the length of the incoming message as we poll the software serial

  boolean _printDebug = false; // Flag to print the serial commands we are sending to the Serial port for debug

  ThingMagic_Module_t _moduleType;
};

#endif // HECTORFI_H