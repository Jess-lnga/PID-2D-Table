#ifndef SERVO_CALIBRATION_WEB_H
#define SERVO_CALIBRATION_WEB_H
#include <Arduino.h>
#include <WebServer.h>
namespace ServoCalibrationWeb
{
void registerRoutes(WebServer& server);
String stateJson();
const char* html();
const char* script();
}
#endif
