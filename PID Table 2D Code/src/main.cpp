#include <Arduino.h>

#include "servo_control.h"
#include "wifi_interface.h"

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("PID 2D Table");
  Serial.println("==============================");

  ServoControl::begin();
  WifiInterface::begin();
}

void loop()
{
  WifiInterface::handleClient();

  int connectedClients = WifiInterface::connectedClients();

  if (connectedClients == 0)
  {
    if (!ServoControl::isDemoRunning())
    {
      ServoControl::startDemo();
    }

    ServoControl::updateDemo();
  }
  else
  {
    if (ServoControl::isDemoRunning())
    {
      ServoControl::stopDemo();

      Serial.println();
      Serial.println("Client connecte");
      Serial.println("Mode manuel");
    }
  }
}
