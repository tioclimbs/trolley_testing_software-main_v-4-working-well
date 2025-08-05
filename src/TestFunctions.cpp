#include "TestFunctions.h"
#include "RemoteLogger.h"

void lidarTest(TFMPlus *frontLidar, TFMPlus *rearLidar)
{
  frontLidar->getData();
  rearLidar->getData();

  uint16_t front_distance = frontLidar->data.dist;
  uint16_t rear_distance = rearLidar->data.dist;
  uint16_t front_strength = frontLidar->data.flux;
  uint16_t rear_strength = rearLidar->data.flux;

  Serial.println("------ LiDAR Readings ------");
  Serial.print("Front: ");
  Serial.print(front_distance);
  Serial.print(" cm (Strength: ");
  Serial.print(front_strength);
  Serial.println(")");

  Serial.print("Rear: ");
  Serial.print(rear_distance);
  Serial.print(" cm (Strength: ");
  Serial.print(rear_strength);
  Serial.println(")");
}

void runRFIDTest(RFID *rfid)
{
  if (rfid->check() == true) // Check to see if any new data has come in from module
  {
    byte responseType = rfid->parseResponse(); // Break response into tag ID, RSSI, frequency, and timestamp

    if (responseType == RESPONSE_IS_TAGFOUND)
    {
      byte tagEPCBytes = rfid->getTagEPCBytes();
      Serial.print(F(" epc["));
      for (byte x = 0; x < tagEPCBytes; x++)
      {
        if (rfid->msg[RFID_EPC_OFFSET + x] < 0x10)
          Serial.print(F("0")); // Pretty print
        Serial.print(rfid->msg[RFID_EPC_OFFSET + x], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("]"));

      Serial.println();
      String tagID = "";
      for (byte x = 0; x < tagEPCBytes; x++)
      {
        if (rfid->msg[RFID_EPC_OFFSET + x] < 0x10)
          tagID += "0";
        tagID += String(rfid->msg[RFID_EPC_OFFSET + x], HEX);
      }

      updateRFIDData(tagID, millis());

      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
    }
  }
  if (digitalRead(PC_13) == HIGH)
  {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
}

void runBackForth(TFMPlus *frontLidar, TFMPlus *rearLidar, VescUart *motor, RFID *rfid)

{
  approachTargetWithLidarPID(frontLidar, motor, MotorState::Forward, rfid);
  approachTargetWithLidarPID(rearLidar, motor, MotorState::Reverse, rfid);
}

void runForwardOnce(TFMPlus *frontLidar, TFMPlus *rearLidar, VescUart *motor, RFID *rfid)
{
  approachTargetWithLidarPID(frontLidar, motor, MotorState::Forward, rfid);
}

void buttonStart(int seconds)
{
  digitalWrite(LED_RED, LOW);
  while (digitalRead(PC_13) == LOW)
    ;
  digitalWrite(LED_RED, HIGH);
  blinkLED(LED_GREEN, seconds);
}

void blinkLED(int ledPin, int seconds)
{
  for (int i = 0; i < seconds; i++)
  {
    digitalWrite(LED_GREEN, LOW);
    delay(500);
    digitalWrite(LED_GREEN, HIGH);
    delay(500);
    Serial.println(seconds - i);
  }
}

void approachTargetWithLidarPID(TFMPlus *lidar, VescUart *motor, MotorState state, RFID *rfid)
{
  MotorState currentState = state;
  MotorState nextState = state;
  MotorState initialState = state;

  unsigned long finalHoldStartTime = 0;
  const unsigned long FINAL_HOLD_DURATION_MS = 1000; // Apply braking current for this

  unsigned long lastRFIDRead = 0;
  const unsigned long RFID_READ_INTERVAL_MS = 250; // How often to read RFID

  float setRPM = MAX_RPM;
  float setReverseRPM = MAX_REVERSE_RPM;

  float speed_MPH = 0.0f; // Speed in MPH from RFID

  bool isFirstEmergencyStop = true; // Flag to check if it's the first emergency stop

  while (currentState != MotorState::Stopped)
  {
    lidar->getData();
    int distance = lidar->data.dist;
    if (distance == 0)
      distance = LIDAR_ERROR_DIST_CM;

    motor->getVescValues();
    int currentRpm = motor->data.rpm;

    if (millis() - lastRFIDRead >= RFID_READ_INTERVAL_MS)
    {
      lastRFIDRead = millis();
      if (readRFID(rfid)) // Check if RFID tag is found
      {
        byte speedByte = rfid->msg[RFID_EPC_OFFSET + 4]; // Assuming speed limit is stored in the first byte of EPC
        Serial.print("Speed limit from RFID: ");
        Serial.println(speedByte);
        speed_MPH = float(speedByte);
        int calculatedRPM = speed_MPH * MPH_TO_RPM_FACTOR; // Convert MPH to RPM
        setRPM = constrain(calculatedRPM, 0, MAX_RPM);     // Constrain RPM to max limit
        setReverseRPM = -setRPM;                           // Set reverse RPM to negative of forward RPM
        Serial.print("Set RPM: ");
        Serial.println(setRPM);
      }
    }

    if (distance < HYSTERISIS_LOW_CM)
    {
      motor->setCurrentBrake(MAX_BRAKE_CURRENT_A);
      nextState = MotorState::EmergencyStop;
    }

    switch (currentState)
    {
    case MotorState::Forward:

      Serial.println("In Forward State");

      if (distance <= SLOWDOWN_DISTANCE_CM)
      {
        nextState = MotorState::SlowdownBraking;
      }
      else
      {
        motor->setRPM(setRPM);

        Serial.println("Moving forward at set RPM.");
        Serial.print("Current set RPM: ");
        Serial.println(setRPM);
        Serial.print("Current set MPH: ");
        Serial.println(speed_MPH);

        nextState = MotorState::Forward;
      }
      break;
    case MotorState::Reverse:

      Serial.println("In Reverse State");

      if (distance <= SLOWDOWN_DISTANCE_CM)
      {
        nextState = MotorState::SlowdownBraking;
      }
      else
      {
        motor->setRPM(setReverseRPM);

        Serial.println("Moving reverse at set RPM.");

        nextState = MotorState::Reverse;
      }
      break;
    case MotorState::SlowdownBraking:

      Serial.println("In Slowdown Braking State");

      if (distance <= STOP_DISTANCE_CM)
      {
        motor->setCurrentBrake(FINAL_BRAKE_CURRENT_A);
        finalHoldStartTime = millis();
        nextState = MotorState::FinalHold;
      }
      else if (distance > SLOWDOWN_DISTANCE_CM)
      {
        nextState = initialState; // Return to initial state if distance is safe
      }
      else
      {
        int targetRPM;
        if (initialState == MotorState::Reverse)
        {
          targetRPM = map(distance, SLOWDOWN_DISTANCE_CM, STOP_DISTANCE_CM, setReverseRPM, 0);
          targetRPM = constrain(targetRPM, setReverseRPM, 0);
        }
        else
        {
          targetRPM = map(distance, SLOWDOWN_DISTANCE_CM, STOP_DISTANCE_CM, setRPM, 0);
          targetRPM = constrain(targetRPM, 0, setRPM);
        }
        motor->setRPM(targetRPM);

        Serial.print("Slowing down to target RPM: ");
        Serial.println(targetRPM);

        nextState = MotorState::SlowdownBraking;
      }
      break;
    case MotorState::FinalHold:
      Serial.println("In Final Hold State");
      if (millis() - finalHoldStartTime >= FINAL_HOLD_DURATION_MS)
      {
        motor->setRPM(0);
        nextState = MotorState::Stopped;
      }
      else
      {
        nextState = MotorState::FinalHold;
      }
      break;
    case MotorState::EmergencyStop:
      if (isFirstEmergencyStop)
      {
        Serial.println("In Emergency Stop State");
        motor->setCurrentBrake(MAX_BRAKE_CURRENT_A);
        isFirstEmergencyStop = false; // Set the flag to false after the first emergency stop
      }
      if (abs(currentRpm) < MIN_ERPM_FOR_STOP)
      {
        motor->setCurrentBrake(FINAL_BRAKE_CURRENT_A);
        finalHoldStartTime = millis();
        nextState = MotorState::FinalHold;
      }
      else
      {
        nextState = MotorState::EmergencyStop;
      }
      break;
    case MotorState::Stopped:
      Serial.println("In Stopped State");
      nextState = MotorState::Stopped;
      break;
    case MotorState::ProportionalBraking:
      nextState = MotorState::SlowdownBraking; // Transition to Slowdown Braking
      break;
    }
    currentState = nextState;
  }
  // Ensure we stop the motor when exiting the loop
  motor->setRPM(0);
  Serial.println("Control loop exited. Motor stopped.");
}

void approachTargetWithLidar(TFMPlus *lidar, VescUart *motor, MotorState state)
{

  MotorState currentState = state;
  MotorState nextState = state;

  unsigned long lastProportionalBrakeCalcTime = 0;
  const unsigned long PROPORTIONAL_BRAKE_INTERVAL_MS = 10; // How often to run P-brake logic

  unsigned long finalHoldStartTime = 0;
  const unsigned long FINAL_HOLD_DURATION_MS = 200; // Apply braking current for this duration to stop

  while (currentState != MotorState::Stopped)
  {
    lidar->getData();
    int distance = lidar->data.dist;
    if (distance == 0)
      distance = LIDAR_ERROR_DIST_CM;

    Serial.print("Front Distance: ");
    Serial.println(distance);

    motor->getVescValues();
    int currentRpm = motor->data.rpm;

    if (distance < HYSTERISIS_LOW_CM)
    {
      currentState = MotorState::EmergencyStop;
      nextState = currentState;
    }

    switch (currentState)
    {
    case MotorState::Forward:
      Serial.println("In Forward State");

      if (distance <= SLOWDOWN_DISTANCE_CM)
      {
        nextState = MotorState::ProportionalBraking;
      }
      else
      {
        motor->setRPM(MAX_RPM);
        Serial.println("Moving forward at max RPM.");
        nextState = MotorState::Forward;
      }
      break;

    case MotorState::Reverse:
      Serial.println("In Reverse State");

      if (distance <= SLOWDOWN_DISTANCE_CM)
      {
        nextState = MotorState::ProportionalBraking;
      }
      else
      {
        motor->setRPM(MAX_REVERSE_RPM);
        Serial.println("Moving reverse at max RPM.");
        nextState = MotorState::Reverse;
      }
      break;
    case MotorState::ProportionalBraking:
      Serial.println("In Proportional Braking State");

      if (millis() - lastProportionalBrakeCalcTime >= PROPORTIONAL_BRAKE_INTERVAL_MS)
      {
        lastProportionalBrakeCalcTime = millis();

        if (abs(currentRpm) < MIN_ERPM_FOR_STOP)
        {
          motor->setCurrentBrake(FINAL_BRAKE_CURRENT_A);
          finalHoldStartTime = millis();
          nextState = MotorState::FinalHold;
        }
        else
        {
          float remainingDistance_m = (float)(distance - STOP_DISTANCE_CM) / 100.0;
          if (remainingDistance_m < 0)
            remainingDistance_m = 0;

          float currentSpeedMagnitude_m_s = abs((float)currentRpm * ERPM_TO_M_S_FACTOR);
          float idealSpeed_m_s = sqrt(2 * DESIRED_DECELERATION_MS2 * remainingDistance_m);
          float speedError_m_s = currentSpeedMagnitude_m_s - idealSpeed_m_s;
          float brakeCurrent = KP_BRAKE * speedError_m_s;
          brakeCurrent = constrain(brakeCurrent, 0, MAX_BRAKE_CURRENT_A);
          motor->setCurrentBrake(brakeCurrent);

          nextState = MotorState::ProportionalBraking;
        }
      }
      else
      {
        nextState = MotorState::ProportionalBraking;
      }
      break;

    case MotorState::FinalHold:
      Serial.println("In Final Hold State");
      if (millis() - finalHoldStartTime >= FINAL_HOLD_DURATION_MS)
      {
        motor->setRPM(0);
        nextState = MotorState::Stopped;
      }
      else
      {
        nextState = MotorState::FinalHold;
      }
      break;

    case MotorState::EmergencyStop:
      Serial.println("In Emergency Stop State");
      motor->setCurrentBrake(MAX_BRAKE_CURRENT_A);

      if (abs(currentRpm) < MIN_ERPM_FOR_STOP)
      {
        motor->setCurrentBrake(FINAL_BRAKE_CURRENT_A);
        finalHoldStartTime = millis();
        nextState = MotorState::FinalHold;
      }
      else
      {
        nextState = MotorState::EmergencyStop;
      }
      break;

    case MotorState::Stopped:
      Serial.println("In Stopped State");
      nextState = MotorState::Stopped;
      break;
    case MotorState::SlowdownBraking:
      nextState = MotorState::ProportionalBraking;
      break;
    }

    currentState = nextState;
  }
}

bool readRFID(RFID *rfid)
{
  if (rfid->check() == true) // Check to see if any new data has come in from module
  {
    byte responseType = rfid->parseResponse(); // Break response into tag ID, RSSI, frequency, and timestamp
    if (responseType == RESPONSE_IS_TAGFOUND)
    {
      return true; // Tag found, return true
    }
    return false; // No tag found, return false
  }
  return false; // No new data, return false
}
/*
void runForwardOnce(TFMPlus *frontLidar, TFMPlus *rearLidar, VescUart *motor)

{
  bool motorStopped = false;
  bool movingForward = true;

  while (movingForward)
  {
    frontLidar->getData();
    int frontDistance = frontLidar->data.dist;
    if (frontDistance == 0)
      frontDistance = 1200;

    Serial.print("Front Distance: ");
    Serial.println(frontDistance);

    if (frontDistance < HYSTERISIS_LOW_CM)
    {
      motor->setCurrentBrake(MAX_BRAKE_CURRENT_A);
      delay(100);
      motor->setRPM(0);
      motorStopped = true;
      movingForward = false;
      Serial.println("Stopping due to below hysterisis distance.");
    }
    else if (frontDistance > SLOWDOWN_DISTANCE_CM || motorStopped == false)
    {
      motor->setRPM(MAX_RPM);
      Serial.println("Moving forward at max RPM.");
    }
    else
    {
      motorStopped = true;

      motor->getVescValues();
      int currentRpm = motor->data.rpm;
      while (currentRpm > MIN_ERPM_FOR_STOP && movingForward)
      {
        motor->getVescValues();
        currentRpm = motor->data.rpm;
        frontLidar->getData();
        frontDistance = frontLidar->data.dist;
        if (frontDistance == 0)
          frontDistance = 1200;
        if (frontDistance < HYSTERISIS_LOW_CM)
        {
          movingForward = false;
          Serial.println("Stopping due to below hysterisis distance.");
          break;
        }

        float remainingDistance_m = (float)(frontDistance - STOP_DISTANCE_CM) / 100.0;
        if (remainingDistance_m < 0)
          remainingDistance_m = 0;

        float currentSpeed_m_s = (float)currentRpm * ERPM_TO_M_S_FACTOR;
        float idealSpeed_m_s = sqrt(2 * DESIRED_DECELERATION_MS2 * remainingDistance_m);
        float speedError_m_s = currentSpeed_m_s - idealSpeed_m_s;
        float brakeCurrent = KP_BRAKE * speedError_m_s;
        brakeCurrent = constrain(brakeCurrent, 0, MAX_BRAKE_CURRENT_A);
        motor->setCurrentBrake(brakeCurrent);

        Serial.print("   Dist: ");
        Serial.print(frontDistance);
        Serial.print(" | RPM: ");
        Serial.print(currentRpm);
        Serial.print(" | IdealSpd(m/s): ");
        Serial.print(idealSpeed_m_s);
        Serial.print(" | Err(m/s): ");
        Serial.print(speedError_m_s);
        Serial.print(" | Brake(A): ");
        Serial.println(brakeCurrent);
        delay(10);
      }

      Serial.println("Braking loop finished. Commanding final stop.");
      motor->setCurrentBrake(1.0); // Apply a small holding brake for a moment
      delay(200);
      motor->setCurrentBrake(0); // Set zero current
      movingForward = false;     // Ensure we exit the outer loop
    }
    delay(10);
  }
}
*/

/*
void drivingLogic(TFMPlus *lidar, VescUart *motor, MotorState state)
{

  while (state != MotorState::Stopped)
  {
    lidar->getData();
    int distance = lidar->data.dist;
    if (distance == 0)
      distance = LIDAR_ERROR_DIST_CM;

    Serial.print("Front Distance: ");
    Serial.println(distance);

    if (distance < HYSTERISIS_LOW_CM)
    {
      motor->setCurrentBrake(MAX_BRAKE_CURRENT_A);
      delay(100);
      motor->setRPM(0);
      state = MotorState::Stopped;
      Serial.println("Stopped due to below hysterisis distance.");
    }
    else if (distance > SLOWDOWN_DISTANCE_CM)
    {
      switch (state)
      {
      case MotorState::Forward:

        motor->setRPM(MAX_RPM);
        Serial.println("Moving forward at max RPM.");
        break;
      case MotorState::Reverse:
        motor->setRPM(MAX_REVERSE_RPM);
        Serial.println("Moving reverse at max RPM.");
        break;
      }
    }
    else
    {
      state = MotorState::Braking;

      while (state == MotorState::Braking)
      {
        motor->getVescValues();
        int currentRpm = motor->data.rpm;
        while (abs(currentRpm) > MIN_ERPM_FOR_STOP)
        {
          motor->getVescValues();
          currentRpm = motor->data.rpm;
          lidar->getData();
          distance = lidar->data.dist;
          if (distance == 0)
            distance = LIDAR_ERROR_DIST_CM;

          if (distance < HYSTERISIS_LOW_CM)
          {
            state = MotorState::Stopped;
            Serial.println("Stopped due to below hysterisis distance.");
            break;
          }
          else
          {
            float remainingDistance_m = (float)(distance - STOP_DISTANCE_CM) / 100.0;
            if (remainingDistance_m < 0)
              remainingDistance_m = 0;

            float currentSpeedMagnitude_m_s = abs((float)currentRpm * ERPM_TO_M_S_FACTOR);
            float idealSpeed_m_s = sqrt(2 * DESIRED_DECELERATION_MS2 * remainingDistance_m);
            float speedError_m_s = currentSpeedMagnitude_m_s - idealSpeed_m_s;
            float brakeCurrent = KP_BRAKE * speedError_m_s;
            brakeCurrent = constrain(brakeCurrent, 0, MAX_BRAKE_CURRENT_A);
            motor->setCurrentBrake(brakeCurrent);

            Serial.print("   Dist: ");
            Serial.print(distance);
            Serial.print(" | RPM: ");
            Serial.print(currentRpm);
            Serial.print(" | IdealSpd(m/s): ");
            Serial.print(idealSpeed_m_s);
            Serial.print(" | Err(m/s): ");
            Serial.print(speedError_m_s);
            Serial.print(" | Brake(A): ");
            Serial.println(brakeCurrent);
            delay(10);
          }
        }

        Serial.println("Braking loop finished. Commanding final stop.");
        motor->setCurrentBrake(1.0); // Apply a small holding brake for a moment
        delay(200);
        motor->setRPM(0);            // Set zero current
        state = MotorState::Stopped; // Ensure we exit the outer loop
      }
    }
  }
}
*/
