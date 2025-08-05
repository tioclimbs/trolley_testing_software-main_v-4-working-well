#ifndef TESTFUNCTIONS_H
#define TESTFUNCTIONS_H

#include <Arduino.h>
#include "VescUart.h"
#include "TFminiLidar.h"
#include "HectoRFID.h"
#include "config.h"

enum class MotorState { Forward, Reverse, ProportionalBraking,
                        SlowdownBraking, EmergencyStop, FinalHold, Stopped };

void runRFIDTest(RFID*);
void runBackForth(TFMPlus*,TFMPlus*,VescUart*,RFID*);
void runForwardOnce(TFMPlus*,TFMPlus*,VescUart*,RFID*);
void lidarTest   (TFMPlus*,TFMPlus*);

// (plus the long PID helpers you already have)
void buttonStart(int seconds);
void blinkLED(int ledPin, int seconds);
void lidarTest(TFMPlus *frontLidar, TFMPlus *rearLidar);
void drivingLogic(TFMPlus *lidar, VescUart *motor, MotorState state, RFID *rfid);
void approachTargetWithLidar(TFMPlus *lidar, VescUart *motor, MotorState state);
void approachTargetWithLidarPID(TFMPlus *lidar, VescUart *motor, MotorState state, RFID *rfid);
bool readRFID(RFID *rfid);
#endif
