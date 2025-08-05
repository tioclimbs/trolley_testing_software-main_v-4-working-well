#ifndef _VESCUART_h
#define _VESCUART_h


/**
 * @file VescUart.h
 * @author Richard Robinson  (richard@climbworks.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include <Arduino.h>
 #include "buffer.h"
 #include "datatypes.h"
 #include "crc.h"


class VescUart
{
    // Struct to store telemetry data
    struct dataPackage
    {
        float avgMotorCurrent;
        float avgInputCurrent;
        float dutyCycleNow;
        float rpm;
        float inpVoltage;
        float ampHours;
        float ampHoursCharged;
        float wattHours;
        float wattHoursCharged;
        int tachometer;
        int tachometerAbs;
        float tempMosfet;
        float tempMotor;
        float pidPos;
        uint8_t id;
        mc_fault_code error;
    };

    //Struct to store nunchuck data
    struct nunchuckPackage
    {
        int valueX;
        int valueY;
        bool upperButton;
        bool lowerButton;
    };

    struct FWVersion
    {
        uint8_t major;
        uint8_t minor;
    };

    const uint32_t _TIMEOUT;
    const float _WHEELDIAMETER;
    const int _POLEPAIRS;
    const float _GEARRATIO;

    public:
    
    /**
     * @brief Construct a new Vesc Uart object
     * 
     * @param timeout_ms Amount of time to wait for a response from VESC
     * @param wheelDiameter Diameter of the wheel in inches
     * @param polePairs Number of pole pairs in the motor
     * @param gearRatio Gear ratio of the motor
     */
    VescUart(uint32_t timeout_ms = 100, float wheelDiameter = 11, int polePairs = 23, float gearRatio = 1.0) ;

    // Variable that stors VESC data
    dataPackage data;

    // Variable that stores nunchuck data
    nunchuckPackage nunchuckData;

    // Variable that stores VESC firmware version
    FWVersion fwVersion;

    /**
     * @brief Sets which serial port to use for communication
     * 
     * @param port 
     */
    void setSerialPort(Stream* port);

    /**
     * @brief requests FW version and stores it in fwVersion
     * 
     * @return true if successful
     * @return false if failed
     */
    bool getFWVersion();

    /**
     * @brief requests VESC data and stores it in data
     * 
     * @return true 
     * @return false 
     */
    bool getVescValues();

    /**
     * @brief Set the current for the VESC motor
     * 
     * @param current Desired current in Amps
     */
    void setCurrent(float current);

    /**
     * @brief Set the brake current for the VESC motor 
     * 
     * @param brakecurrent Desired brake current in Amps
     */
    void setCurrentBrake(float brakecurrent);

    /**
     * @brief Set the RPM for the VESC motor
     * 
     * @param rpm Desired eRPM (RPM * pole pairs)
     */
    void setRPM(float rpm);

     /**
     * @brief Set the RPM for the VESC motor
     * 
     * @param mph Desired mph
     */
    void setMPH(float mph);

 
    /**
     * @brief Send a keep alive message to prevent shutdown
     * 
     */
    void sendKeepAlive(void);

    /**
     * @brief Print the VESC values to the serial port for debug
     * 
     */
    void printVescValues(void);

    /**
     * @brief Print the VESC FW to the serial port for debug
     * 
     */
    void printFWValues(void);

    private:

    Stream* serialPort = NULL;

    /**
     * @brief Packs a payload and sends it to the VESC
     * 
     * @param payload the payload to send as a byte array
     * @param lengthpay the length of the payload
     * @return int returns the number of bytes sent
     */
    int packSendPayload(uint8_t * payload, int lengthPay);

    /**
     * @brief Recieves a message from VESC
     * 
     * @param payloadReceived the received payload as a byte array
     * @return int returns the number of bytes received
     */
    int receiveUartMessage(uint8_t * payloadReceived);

    /**
     * @brief Verifies the message (CRC-16) and extracts the payload
     * 
     * @param message The recieved UART message
     * @param lenMes the length of the message
     * @param payload the final data ready to extract data from
     * @return true 
     * @return false 
     */
    bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);

    /**
     * @brief Extracts data from the payload
     * 
     * @param payload the payload to extract data from 
     * @return true if successful
     * @return false if failled
     */
    bool processReadPacket(uint8_t * payload);

};

#endif