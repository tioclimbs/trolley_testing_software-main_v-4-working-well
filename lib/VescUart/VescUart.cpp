/**
 * @file VescUart.cpp
 * @author Richard Robinson  (richard@climbworks.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "VescUart.h"

VescUart::VescUart(uint32_t timeout_ms, float wheelDiameter, int polePairs, float gearRatio) : _TIMEOUT(timeout_ms), _WHEELDIAMETER(wheelDiameter), _POLEPAIRS(polePairs), _GEARRATIO(gearRatio)
{
}

void VescUart::setSerialPort(Stream* port)
{
    serialPort = port;
}

bool VescUart::getFWVersion()
{
    int32_t index = 0;
	const int payloadSize = 1;
	uint8_t payload[payloadSize];
	
	
	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);
	if (messageLength > 0) { 
		return processReadPacket(message); 
	}
	return false;
}

bool VescUart::getVescValues()
{
	int32_t index = 0;
	const int payloadSize = 1;
	uint8_t payload[payloadSize];

	payload[index++] = { COMM_GET_VALUES };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 55) {
		return processReadPacket(message); 
	}
	return false;
}

void VescUart::setCurrent(float current)
{
    int32_t index = 0;
	const int payloadSize = 5;
	uint8_t payload[payloadSize];
	
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setCurrentBrake(float brakeCurrent)
{
    int32_t index = 0;
	const int payloadSize = 5;
	uint8_t payload[payloadSize];
	
	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	packSendPayload(payload, payloadSize);

}

void VescUart::setRPM(float rpm)
{
    int32_t index = 0;
	const int payloadSize = 5;
	uint8_t payload[payloadSize];
	
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setMPH(float mph)
{
    int32_t index = 0;
	const int payloadSize = 5;
	uint8_t payload[payloadSize];
    int32_t rpm = mph * _POLEPAIRS * 1056 * _GEARRATIO / (_WHEELDIAMETER * PI);
	
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepAlive()
{
    int32_t index = 0;
	const int payloadSize = 1;
	uint8_t payload[payloadSize];
	
	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

void VescUart::printVescValues()
{
    Serial.print("avgMotorCurrent: "); 	Serial.println(data.avgMotorCurrent);
    Serial.print("avgInputCurrent: "); 	Serial.println(data.avgInputCurrent);
    Serial.print("dutyCycleNow: "); 	Serial.println(data.dutyCycleNow);
    Serial.print("rpm: "); 				Serial.println(data.rpm);
    Serial.print("inputVoltage: "); 	Serial.println(data.inpVoltage);
    Serial.print("ampHours: "); 		Serial.println(data.ampHours);
    Serial.print("ampHoursCharged: "); 	Serial.println(data.ampHoursCharged);
    Serial.print("wattHours: "); 		Serial.println(data.wattHours);
    Serial.print("wattHoursCharged: "); Serial.println(data.wattHoursCharged);
    Serial.print("tachometer: "); 		Serial.println(data.tachometer);
    Serial.print("tachometerAbs: "); 	Serial.println(data.tachometerAbs);
    Serial.print("tempMosfet: "); 		Serial.println(data.tempMosfet);
    Serial.print("tempMotor: "); 		Serial.println(data.tempMotor);
    Serial.print("error: "); 			Serial.println(data.error);
}

void VescUart::printFWValues() {	
    Serial.print("FW Version: "); 
    Serial.print(fwVersion.major); 	Serial.print("."); Serial.println(fwVersion.minor);
}

int VescUart::packSendPayload(uint8_t * payload, int lengthPay)
{
    uint16_t crcPayload = crc16(payload, lengthPay);
	int count = 0;
	uint8_t messageSend[256];
	
	if (lengthPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lengthPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lengthPay >> 8);
		messageSend[count++] = (uint8_t)(lengthPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lengthPay);
	count += lengthPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	
	// Sending package
	if( serialPort != NULL )
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived)
{
    uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	uint32_t timeout = millis() + _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			messageReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for size + 3 for CRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
                        /* To do figure out messages greater than 256 bytes
                         *
                         * messageReceived[counter++] = serialPort->read();
                         * endMessage = ((static_cast<uint16_t>(messageReceived[1]) << 8) | messageReceived[2] )+ 6; //Payload size + 3 for size + 3 for SRC and End.
                         * lenPayload = (messageReceived[1] << 8) | messageReceived[2];
                         */

                        Serial.println("Message greater than 256 bytes not supported yet");
					break;

					default:
						Serial.println("Invalid start bit");

					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				Serial.println("End of message reached!");
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false) {
		Serial.println("Message read Timeout");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		return lenPayload; 
	}
	else {
		return 0;
	}
}

bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload)
{
    uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	Serial.print("SRC received: "); Serial.println(crcMessage);
	

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	Serial.print("SRC calc: "); Serial.println(crcPayload);
	
	
	if (crcPayload == crcMessage) {
		return true;
	}else{
		return false;
	}
}

bool VescUart::processReadPacket(uint8_t * payload)
{
    COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)payload[0];
	payload++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

            fwVersion.major = payload[index++];
            fwVersion.minor = payload[index++];
			return true;
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(payload, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(payload, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(payload, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(payload, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(payload, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(payload, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(payload, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(payload, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(payload, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)payload[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(payload, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= payload[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;
		default:
			return false;
	}
    
}


