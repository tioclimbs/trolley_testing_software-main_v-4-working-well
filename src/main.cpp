#include "TestFunctions.h"
#include "RemoteLogger.h"

TFMPlus frontLidar;
TFMPlus rearLidar;
VescUart motor;
RFID rfid;

void setup()
{
  Serial.begin(115200);
  delay(100);

  // Setup LEDs or status indicators
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Initialize motor and sensors
  Serial1.begin(115200);

  motor.setSerialPort(&Serial1);
  motor.getFWVersion();
  Serial.println("VESC communication started");
  motor.printFWValues();

  Serial2.begin(115200);

  frontLidar.begin(&Serial2);
  Serial.println("Front LiDAR communication started");
  frontLidar.printFWValues();

  Serial3.begin(115200);

  rearLidar.begin(&Serial3);
  Serial.println("Rear LiDAR communication started");
  rearLidar.printFWValues();

  connectToWiFi(); // NEW

  Serial4.begin(115200);
  rfid.begin(&Serial4);

  if (TEST_MODE == RFID_TEST || TEST_MODE == BACK_FORTH || TEST_MODE == FORWARD_ONCE)
  {

    if (!rfid.setupModule())
    {
      Serial.println("RFID setup failed");
      while (true)
        ;
    }
    else
    {
      Serial.println("✅ RFID setup successful");
      rfid.startReading();
    }
  }
}

void loop()
{
  switch (TEST_MODE)
  {
  case RFID_TEST:
    runRFIDTest(&rfid);
    break;
  case BACK_FORTH:
    buttonStart(START_DELAY);
    for (int i = 0; i < NUMBER_OF_REPS; i++)
    {
      runBackForth(&frontLidar, &rearLidar, &motor, &rfid);
    }
    break;
  case LIDAR_TEST:
    lidarTest(&frontLidar, &rearLidar);
    break;
  case FORWARD_ONCE:
    buttonStart(START_DELAY);
    runForwardOnce(&frontLidar, &rearLidar, &motor, &rfid);
    break;
  }

  handleClient(&frontLidar, &rearLidar);
}
