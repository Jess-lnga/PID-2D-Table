#include <Arduino.h>
#include "servo_control.h"
#include "wifi_interface.h"
#include "imu_data.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("PID 2D Table - MPU I2C");
  ServoControl::begin();
  ImuData::begin();
  WifiInterface::begin();
}

void loop()
{
  ImuData::update();
  if (ImuData::isEnabled())
  {
    // Keep the last servo positions; orientation is displayed only.
    if (ServoControl::isDemoRunning()) ServoControl::stopDemo();
  }
  else
  {
    if (!ServoControl::isDemoRunning()) ServoControl::startDemo();
    ServoControl::updateDemo();
  }
  WifiInterface::handleClient();
}
