#include "wifi_interface.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "imu_data.h"
#include "servo_control.h"

namespace WifiInterface
{
namespace
{
const char* ssid = "PID2DTable";
const char* password = "delusions1234";

IPAddress local_IP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 4);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
WiFiUDP imuUdp;

constexpr uint16_t IMU_UDP_PORT = 4210;
constexpr size_t IMU_UDP_BUFFER_SIZE = 128;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width, initial-scale=1.0">

<title>PID 2D Table</title>

<style>

body
{
  font-family: Arial, sans-serif;
  background: #f2f2f2;
  text-align: center;
  margin: 0;
  padding: 20px;
}

h1
{
  margin-bottom: 5px;
}

.subtitle
{
  color: #666;
  margin-bottom: 30px;
}

.servo-box
{
  background: white;
  max-width: 600px;
  margin: 20px auto;
  padding: 25px;
  border-radius: 12px;
  box-shadow:
    0 2px 10px
    rgba(0,0,0,0.1);
}

input[type=range]
{
  width: 90%;
  margin: 20px 0;
}

input[type=number]
{
  width: 110px;
  font-size: 20px;
  text-align: center;
  padding: 8px;
}

button
{
  font-size: 17px;
  padding: 10px 15px;
  margin: 5px;
  cursor: pointer;
}

.value
{
  font-size: 22px;
  font-weight: bold;
}

.imu-grid
{
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 12px;
  margin: 18px 0;
}

.imu-reading
{
  background: #f4f6f8;
  border-radius: 8px;
  padding: 12px;
}

.imu-reading span
{
  display: block;
  font-size: 28px;
  font-weight: bold;
  margin-top: 4px;
}

.imu-status
{
  color: #666;
  min-height: 22px;
}

.imu-note
{
  color: #666;
  font-size: 14px;
  line-height: 1.4;
  margin-top: 10px;
}

.imu-button-active
{
  background: #1f7a4d;
  color: white;
}

.viewer
{
  width: 260px;
  height: 180px;
  margin: 24px auto 10px;
  perspective: 650px;
}

.platform
{
  width: 220px;
  height: 140px;
  margin: 0 auto;
  transform-style: preserve-3d;
  transition: transform 80ms linear;
}

.platform-top
{
  position: absolute;
  width: 220px;
  height: 140px;
  border-radius: 8px;
  background:
    linear-gradient(135deg, #e8eef6, #9cb3c9);
  border: 2px solid #43566b;
  box-shadow:
    0 18px 28px
    rgba(0,0,0,0.22);
  transform: rotateX(68deg);
}

.platform-axis
{
  position: absolute;
  font-size: 13px;
  font-weight: bold;
  color: #253140;
}

.manual-imu
{
  margin-top: 22px;
}

.manual-imu label
{
  display: block;
  margin-top: 14px;
  color: #333;
}

.platform-axis-x
{
  right: 12px;
  top: 58px;
}

.platform-axis-y
{
  left: 96px;
  top: 10px;
}

@media (max-width: 520px)
{
  .imu-grid
  {
    grid-template-columns: 1fr;
  }
}

</style>

</head>

<body>

<h1>PID 2D Table</h1>

<div class="subtitle">
ESP32 Servo Controller
</div>

<div class="servo-box">

<h2>IMU Smartphone</h2>

<button id="imuButton" onclick="toggleImuTransmission()">
Activer transmission IMU
</button>

<div class="imu-status" id="imuStatus">
Transmission inactive
</div>

<div class="imu-note" id="imuNote">
UDP IMU: envoyer pitch=12.3,roll=-4.5 vers 192.168.1.4:4210.
</div>

<div class="imu-grid">

<div class="imu-reading">
Pitch
<span id="pitchDisplay">0.0 deg</span>
</div>

<div class="imu-reading">
Roll
<span id="rollDisplay">0.0 deg</span>
</div>

</div>

<div class="manual-imu">

<label for="manualPitch">
Pitch test
</label>

<input
type="range"
id="manualPitch"
min="-45"
max="45"
value="0"
step="0.5"
oninput="manualImuChanged()"
>

<label for="manualRoll">
Roll test
</label>

<input
type="range"
id="manualRoll"
min="-45"
max="45"
value="0"
step="0.5"
oninput="manualImuChanged()"
>

</div>

<div class="viewer">
<div class="platform" id="platform3d">
<div class="platform-top">
<div class="platform-axis platform-axis-x">Roll</div>
<div class="platform-axis platform-axis-y">Pitch</div>
</div>
</div>
</div>

</div>

<!-- SERVO 1 -->

<div class="servo-box">

<h2>Servo 1 - GPIO 25</h2>

<div class="value">
<span id="display1">1300</span> us
</div>

<input
type="range"
id="slider1"
min="1090"
max="1510"
value="1300"
step="1"
oninput="sliderChanged(1)"
>

<br>

<button onclick="incrementServo(1,-10)">
-10 us
</button>

<input
type="number"
id="number1"
min="1090"
max="1510"
value="1300"
step="1"
onchange="numberChanged(1)"
>

<button onclick="incrementServo(1,10)">
+10 us
</button>

</div>

<!-- SERVO 2 -->

<div class="servo-box">

<h2>Servo 2 - GPIO 26</h2>

<div class="value">
<span id="display2">1215</span> us
</div>

<input
type="range"
id="slider2"
min="965"
max="1465"
value="1215"
step="1"
oninput="sliderChanged(2)"
>

<br>

<button onclick="incrementServo(2,-10)">
-10 us
</button>

<input
type="number"
id="number2"
min="965"
max="1465"
value="1215"
step="1"
onchange="numberChanged(2)"
>

<button onclick="incrementServo(2,10)">
+10 us
</button>

</div>

<!-- SERVO 3 -->

<div class="servo-box">

<h2>Servo 3 - GPIO 27</h2>

<div class="value">
<span id="display3">1340</span> us
</div>

<input
type="range"
id="slider3"
min="1130"
max="1550"
value="1340"
step="1"
oninput="sliderChanged(3)"
>

<br>

<button onclick="incrementServo(3,-10)">
-10 us
</button>

<input
type="number"
id="number3"
min="1130"
max="1550"
value="1340"
step="1"
onchange="numberChanged(3)"
>

<button onclick="incrementServo(3,10)">
+10 us
</button>

</div>

<script>

let imuTransmissionEnabled = false;
let imuSendTimer = null;
let latestPitch = 0.0;
let latestRoll = 0.0;
let lastImuSendMs = 0;
let imuSensorAvailable =
  typeof DeviceOrientationEvent !== "undefined";

function clamp(value, minValue, maxValue)
{
  return Math.min(
    Math.max(value, minValue),
    maxValue
  );
}

function setImuUI(enabled)
{
  imuTransmissionEnabled = enabled;

  let button =
    document.getElementById("imuButton");

  button.innerHTML =
    enabled ?
    "Arreter transmission IMU" :
    "Activer transmission IMU";

  button.classList.toggle(
    "imu-button-active",
    enabled
  );

  document.getElementById("imuStatus").innerHTML =
    enabled ?
    "Transmission active" :
    "Transmission inactive";
}

function setImuNote(message)
{
  document.getElementById("imuNote").innerHTML =
    message;
}

function updateImuDisplay(pitch, roll)
{
  latestPitch = pitch;
  latestRoll = roll;

  document.getElementById("pitchDisplay").innerHTML =
    pitch.toFixed(1) + " deg";

  document.getElementById("rollDisplay").innerHTML =
    roll.toFixed(1) + " deg";

  let visualPitch =
    clamp(pitch, -45, 45);

  let visualRoll =
    clamp(roll, -45, 45);

  document.getElementById("platform3d").style.transform =
    "rotateX(" +
    (-visualPitch).toFixed(1) +
    "deg) rotateY(" +
    visualRoll.toFixed(1) +
    "deg)";
}

function sendImuSample()
{
  if (!imuTransmissionEnabled)
  {
    return;
  }

  fetch(
    "/imu?pitch=" +
    latestPitch.toFixed(3) +
    "&roll=" +
    latestRoll.toFixed(3)
  );
}

function manualImuChanged()
{
  let pitch =
    parseFloat(
      document.getElementById("manualPitch").value
    );

  let roll =
    parseFloat(
      document.getElementById("manualRoll").value
    );

  updateImuDisplay(
    pitch,
    roll
  );

  if (!imuTransmissionEnabled)
  {
    fetch("/imu/mode?enabled=1")
    .then(() =>
    {
      setImuUI(true);
      setImuNote(
        "Mode test manuel actif. Ces valeurs passent par la meme route /imu que les donnees smartphone."
      );
      sendImuSample();
    });

    return;
  }

  sendImuSample();
}

function onDeviceOrientation(event)
{
  if (!imuTransmissionEnabled)
  {
    return;
  }

  let pitch =
    event.beta || 0.0;

  let roll =
    event.gamma || 0.0;

  updateImuDisplay(
    pitch,
    roll
  );

  let now =
    Date.now();

  if (now - lastImuSendMs >= 50)
  {
    lastImuSendMs = now;
    sendImuSample();
  }
}

function startImuTransmission()
{
  fetch("/imu/mode?enabled=1")
  .then(() =>
  {
    setImuUI(true);
    setImuNote(
      "Capteur navigateur actif. Les donnees pitch/roll sont envoyees vers l'ESP32."
    );

    window.addEventListener(
      "deviceorientation",
      onDeviceOrientation
    );

    if (imuSendTimer === null)
    {
      imuSendTimer =
        setInterval(
          sendImuSample,
          200
        );
    }
  });
}

function stopImuTransmission()
{
  fetch("/imu/mode?enabled=0")
  .then(() =>
  {
    setImuUI(false);
    setImuNote(
      "Transmission arretee. Les sliders restent disponibles pour tester la pipeline."
    );

    window.removeEventListener(
      "deviceorientation",
      onDeviceOrientation
    );

    if (imuSendTimer !== null)
    {
      clearInterval(imuSendTimer);
      imuSendTimer = null;
    }
  });
}

function toggleImuTransmission()
{
  if (imuTransmissionEnabled)
  {
    stopImuTransmission();
    return;
  }

  if (
    imuSensorAvailable &&
    typeof DeviceOrientationEvent.requestPermission === "function"
  )
  {
    DeviceOrientationEvent.requestPermission()
    .then(permissionState =>
    {
      if (permissionState === "granted")
      {
        startImuTransmission();
      }
      else
      {
        document.getElementById("imuStatus").innerHTML =
          "Permission IMU refusee";
        setImuNote(
          "Le navigateur voit l'API IMU, mais l'autorisation capteur a ete refusee."
        );
      }
    })
    .catch(() =>
    {
      document.getElementById("imuStatus").innerHTML =
        "Permission IMU indisponible";
      setImuNote(
        "Le navigateur ne permet pas d'obtenir la permission capteur sur cette page."
      );
    });

    return;
  }

  if (!imuSensorAvailable)
  {
    document.getElementById("imuStatus").innerHTML =
      "IMU non disponible dans ce navigateur";
    setImuNote(
      "Essaie depuis un smartphone avec Chrome/Firefox Android. Sur iPhone ou certains navigateurs, les capteurs sont bloques sur les pages HTTP comme celle de l'ESP32."
    );
    return;
  }

  startImuTransmission();
}

function setUI(servo, value)
{
  document.getElementById(
    "slider" + servo
  ).value = value;

  document.getElementById(
    "number" + servo
  ).value = value;

  document.getElementById(
    "display" + servo
  ).innerHTML = value;
}

function sendServo(servo, value)
{
  value = parseInt(value);

  setUI(
    servo,
    value
  );

  fetch(
    "/set?servo=" +
    servo +
    "&us=" +
    value
  );
}

function sliderChanged(servo)
{
  let slider =
    document.getElementById(
      "slider" + servo
    );

  sendServo(
    servo,
    slider.value
  );
}

function numberChanged(servo)
{
  let number =
    document.getElementById(
      "number" + servo
    );

  sendServo(
    servo,
    number.value
  );
}

function incrementServo(
  servo,
  increment
)
{
  let number =
    document.getElementById(
      "number" + servo
    );

  let value =
    parseInt(number.value);

  value += increment;

  sendServo(
    servo,
    value
  );
}

function updateState()
{
  fetch("/state")

  .then(response =>
    response.json()
  )

  .then(data =>
  {
    setUI(
      1,
      data.servo1
    );

    setUI(
      2,
      data.servo2
    );

    setUI(
      3,
      data.servo3
    );

    setImuUI(
      data.imuTransmissionEnabled
    );

    if (data.imuHasData)
    {
      updateImuDisplay(
        data.pitch,
        data.roll
      );
    }
  });
}

window.onload =
  updateState;

setInterval(
  updateState,
  200
);

</script>

</body>

</html>
)rawliteral";

void handleRoot()
{
  server.send(200, "text/html", index_html);
}

void handleState()
{
  ImuData::State imuState = ImuData::getState();

  String json = "{";

  json += "\"servo1\":";
  json += ServoControl::getServoMicroseconds(1);

  json += ",";

  json += "\"servo2\":";
  json += ServoControl::getServoMicroseconds(2);

  json += ",";

  json += "\"servo3\":";
  json += ServoControl::getServoMicroseconds(3);

  json += ",";

  json += "\"imuTransmissionEnabled\":";
  json += imuState.transmissionEnabled ? "true" : "false";

  json += ",";

  json += "\"imuHasData\":";
  json += imuState.hasData ? "true" : "false";

  json += ",";

  json += "\"pitch\":";
  json += String(imuState.pitchDeg, 3);

  json += ",";

  json += "\"roll\":";
  json += String(imuState.rollDeg, 3);

  json += ",";

  json += "\"imuAgeMs\":";
  json += imuState.hasData ? String(millis() - imuState.lastUpdateMs) : "0";

  json += "}";

  server.send(200, "application/json", json);
}

void handleImuMode()
{
  if (!server.hasArg("enabled"))
  {
    server.send(400, "text/plain", "Parametre manquant");
    return;
  }

  bool enabled = server.arg("enabled").toInt() != 0;

  ImuData::setTransmissionEnabled(enabled);

  Serial.println();
  Serial.println(
    enabled ?
    "Transmission IMU activee" :
    "Transmission IMU arretee");

  server.send(200, "text/plain", "OK");
}

void handleImuData()
{
  if (!server.hasArg("pitch") || !server.hasArg("roll"))
  {
    server.send(400, "text/plain", "Parametres manquants");
    return;
  }

  float pitch = server.arg("pitch").toFloat();
  float roll = server.arg("roll").toFloat();

  ImuData::update(pitch, roll);

  server.send(200, "text/plain", "OK");
}

void handleSetServo()
{
  if (!server.hasArg("servo") || !server.hasArg("us"))
  {
    server.send(400, "text/plain", "Parametres manquants");
    return;
  }

  int servoNumber = server.arg("servo").toInt();
  int pulseWidth = server.arg("us").toInt();

  if (!ServoControl::setServoMicroseconds(servoNumber, pulseWidth))
  {
    server.send(400, "text/plain", "Servo invalide");
    return;
  }

  server.send(200, "text/plain", "OK");
}

bool parseFloatAfterKey(const char* message, const char* key, float& value)
{
  const char* keyPosition = strstr(message, key);

  if (keyPosition == nullptr)
  {
    return false;
  }

  const char* valueStart = strchr(keyPosition, ':');

  if (valueStart == nullptr)
  {
    valueStart = strchr(keyPosition, '=');
  }

  if (valueStart == nullptr)
  {
    return false;
  }

  value = atof(valueStart + 1);
  return true;
}

bool parseImuMessage(const char* message, float& pitch, float& roll)
{
  bool hasPitch = parseFloatAfterKey(message, "pitch", pitch);
  bool hasRoll = parseFloatAfterKey(message, "roll", roll);

  if (hasPitch && hasRoll)
  {
    return true;
  }

  char* endPointer = nullptr;
  pitch = strtof(message, &endPointer);

  if (endPointer == message)
  {
    return false;
  }

  while (*endPointer == ' ' || *endPointer == ',' || *endPointer == ';')
  {
    endPointer++;
  }

  char* rollStart = endPointer;
  roll = strtof(rollStart, &endPointer);

  return endPointer != rollStart;
}

void handleImuUdp()
{
  int packetSize = imuUdp.parsePacket();

  if (packetSize <= 0)
  {
    return;
  }

  char buffer[IMU_UDP_BUFFER_SIZE];
  int bytesRead = imuUdp.read(buffer, IMU_UDP_BUFFER_SIZE - 1);

  if (bytesRead <= 0)
  {
    return;
  }

  buffer[bytesRead] = '\0';

  float pitch = 0.0f;
  float roll = 0.0f;

  if (!parseImuMessage(buffer, pitch, roll))
  {
    Serial.print("Message UDP IMU invalide : ");
    Serial.println(buffer);
    return;
  }

  if (!ImuData::isTransmissionEnabled())
  {
    ImuData::setTransmissionEnabled(true);
    Serial.println("Transmission IMU activee par UDP");
  }

  ImuData::update(pitch, roll);
}
} // namespace

void begin()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);

  delay(500);

  Serial.println();
  Serial.print("SSID : ");
  Serial.println(ssid);

  Serial.print("IP : ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSetServo);
  server.on("/state", handleState);
  server.on("/imu/mode", handleImuMode);
  server.on("/imu", handleImuData);

  server.begin();

  Serial.println("Serveur web demarre");

  imuUdp.begin(IMU_UDP_PORT);

  Serial.print("UDP IMU : port ");
  Serial.println(IMU_UDP_PORT);
}

void handleClient()
{
  server.handleClient();
  handleImuUdp();
}

int connectedClients()
{
  return WiFi.softAPgetStationNum();
}
} // namespace WifiInterface
