#include <Arduino.h>
#include "servo_control.h"
#include "wifi_interface.h"
#include "imu_data.h"
#include "servo_calibration.h"
#include "platform_control.h"
#include "runtime_lock.h"
#include "tch_screen.h"

void setup()
{
  RuntimeLock::begin();
  Serial.begin(115200);
  delay(1000);
  Serial.println("PID 2D Table - MPU I2C");
  ServoControl::begin();
  ServoCalibration::begin();
  ImuData::begin();
  TchScreen::begin();
  WifiInterface::begin();
}

void loop()
{
  {
    RuntimeLock::Guard guard;
    ImuData::update();
    TchScreen::update();
    ServoCalibration::update();
    PlatformControl::update(ImuData::getState());
  }
  delay(1); // HTTP runs independently on core 0; sensor/control run on core 1.
}
