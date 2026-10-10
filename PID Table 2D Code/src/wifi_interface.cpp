#include "wifi_interface.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

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

.viewer { height: 250px; display: grid; place-items: center; perspective: 700px; }
.cube-view { transform: rotateX(-20deg) rotateY(-25deg); transform-style: preserve-3d; }
/* Sensor +Z is CSS -Y: positive 90 degrees about Z uses rotateY(-90deg). */
.cube-heading { transform: rotateY(90deg); transform-style: preserve-3d; }
button { padding: 10px 20px; margin: 12px; font-size: 16px; cursor: pointer; }
button:disabled { cursor: default; opacity: .5; }
.cube { width: 120px; height: 120px; position: relative; transform-style: preserve-3d; }
.face { position: absolute; width: 120px; height: 120px; box-sizing: border-box; border: 2px solid #253140; display: grid; place-items: center; font-weight: bold; opacity: .9; backface-visibility: hidden; }
.front { background: #91c9ed; transform: translateZ(60px); }
.back { background: #91c9ed; transform: rotateY(180deg) translateZ(60px); }
.right { background: #f4aa78; transform: rotateY(90deg) translateZ(60px); }
.left { background: #f4aa78; transform: rotateY(-90deg) translateZ(60px); }
.top { background: #88d4ab; transform: rotateX(90deg) translateZ(60px); }
.bottom { background: #88d4ab; transform: rotateX(-90deg) translateZ(60px); }
.viewer.inactive { opacity: .3; }
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

<h2>Orientation MPU</h2>
<div class="imu-status" id="imuStatus">Connexion...</div>
<div class="imu-note" id="imuNote">Interrupteur OFF : automatique. ON : affichage IMU.</div>
<div class="imu-grid">
<div class="imu-reading">Pitch<span id="pitchDisplay">—</span></div>
<div class="imu-reading">Roll<span id="rollDisplay">—</span></div>
</div>
<button id="calibrateButton" onclick="calibrateImu()" disabled>Calibrate</button>
<div class="imu-note" id="calibrateFeedback" role="status"></div>
<progress id="calibrationProgress" max="200" value="0" style="width:100%"></progress>
<div class="imu-note" id="calibrationDetail">Calibration en attente</div>
<details open><summary>Diagnostics MPU</summary>
<pre id="imuDiagnostics" style="text-align:left;white-space:pre-wrap;overflow-wrap:anywhere">En attente...</pre>
</details>
<div class="viewer inactive" id="viewer">
<div class="cube-view"><div class="cube-heading"><div class="cube" id="cube">
<div class="face front">Avant</div><div class="face back">Arrière</div>
<div class="face right">X</div><div class="face left">−X</div>
<div class="face top">Z ↑</div><div class="face bottom">−Z</div>
</div></div></div></div>
<div class="imu-note">Roll : rotation autour de X. Pitch : rotation autour de Y.
Le cap (yaw) n'est pas représenté. Les servos gardent leur dernière position en mode IMU.</div>
</div>

<div class="servo-box">
<h2>Positions des servos</h2>
<p>Affichage uniquement : le mode est choisi par l'interrupteur physique.</p>
<p>Servo 1 · GPIO 25 : <strong id="display1">—</strong> µs</p>
<p>Servo 2 · GPIO 26 : <strong id="display2">—</strong> µs</p>
<p>Servo 3 · GPIO 27 : <strong id="display3">—</strong> µs</p>
</div>

<script>

let calibrationRequestPending = false;
let latestImuState = null;

function updateCalibrateButton()
{
  document.getElementById("calibrateButton").disabled = calibrationRequestPending ||
    !latestImuState || !latestImuState.imuEnabled || !latestImuState.imuConnected;
}

async function calibrateImu()
{
  if (calibrationRequestPending || !latestImuState || !latestImuState.imuEnabled || !latestImuState.imuConnected) return;
  calibrationRequestPending = true;
  updateCalibrateButton();
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  const feedback = document.getElementById("calibrateFeedback");
  try
  {
    const response = await fetch("/imu/calibrate", {method: "POST", signal: controller.signal});
    if (!response.ok) throw new Error(await response.text());
    feedback.textContent = "Calibration relancee : garder le MPU immobile environ 2 secondes.";
  }
  catch (error)
  {
    feedback.textContent = "Calibration non confirmee : " + error.message;
  }
  finally
  {
    clearTimeout(timeout);
    calibrationRequestPending = false;
    updateCalibrateButton();
  }
}

function updateImuDisplay(data)
{
  latestImuState = data;
  updateCalibrateButton();
  const valid = data.imuEnabled && data.imuConnected && data.imuHasData && data.imuAgeMs < 250;
  document.getElementById("imuStatus").textContent = !data.imuEnabled ? "Mode automatique — interrupteur OFF" :
    !data.imuConnected ? "Mode IMU — " + data.imuDiagnostic :
    !data.imuCalibrated ? "Calibration — " + data.imuDiagnostic :
    !valid ? "Mode IMU — en attente de mesures" : "Mode IMU — mesures actives";
  document.getElementById("imuNote").textContent = data.imuConnected ?
    "MPU : adresse 0x" + data.imuAddress.toString(16) + ", identification 0x" + data.imuDeviceId.toString(16) :
    "SDA : GPIO 21 · SCL : GPIO 22 · interrupteur : GPIO 32";
  document.getElementById("calibrationProgress").value = data.imuCalibrationSamples;
  document.getElementById("calibrationProgress").hidden = !data.imuEnabled || data.imuCalibrated;
  document.getElementById("calibrationDetail").textContent = !data.imuEnabled ? "Acquisition arretee" :
    data.imuCalibrated ? "Calibration terminee — angles relatifs au zero choisi" :
    data.imuCalibrationSamples + "/200 mesures stables — " + data.imuCalibrationRestarts + " redemarrages. Angles provisoires issus de l'accelerometre.";
  const vector = (values, unit) => values.map(value => value.toFixed(3)).join(", ") + " " + unit;
  document.getElementById("imuDiagnostics").textContent =
    "Mesures recues : " + data.imuSamples + " | age : " + (data.imuSamples ? data.imuAgeMs + " ms" : "aucune") +
    "\nAccel XYZ : " + vector(data.accelG, "g") + " | norme : " + data.gravityG.toFixed(3) + " g" +
    "\nGyro XYZ : " + vector(data.gyroDps, "deg/s") +
    "\nBiais gyro : " + vector(data.gyroBiasDps, "deg/s") +
    "\nZero manipulation : roll " + data.rollZeroDeg.toFixed(2) + " deg, pitch " + data.pitchZeroDeg.toFixed(2) + " deg" +
    "\nINT_STATUS : 0x" + data.imuInterruptStatus.toString(16) +
    " | erreurs I2C : " + data.imuI2cErrors +
    " | dernier registre : 0x" + data.imuLastRegister.toString(16) +
    " | code : " + data.imuLastI2cError + " (255 = lecture incomplete)" +
    "\n" + data.imuDiagnostic;
  document.getElementById("viewer").classList.toggle("inactive", !valid);
  document.getElementById("pitchDisplay").textContent = valid ? data.pitch.toFixed(1) + " °" : "—";
  document.getElementById("rollDisplay").textContent = valid ? data.roll.toFixed(1) + " °" : "—";
  if (valid)
  {
    // Sensor X -> screen right, Y -> screen depth, Z -> screen up.
    // Matrix is R_y(pitch)*R_x(roll), expressed in CSS coordinates.
    const r = data.roll * Math.PI / 180, p = -data.pitch * Math.PI / 180;
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p);
    const matrix = [cp, -sp, 0, 0, sp*cr, cp*cr, -sr, 0, sp*sr, cp*sr, cr, 0, 0, 0, 0, 1];
    document.getElementById("cube").style.transform = "matrix3d(" + matrix.join(",") + ")";
  }
}

function setUI(servo, value)
{
  document.getElementById("display" + servo).textContent = value;
}

async function updateState()
{
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try
  {
    const response = await fetch("/state", {cache: "no-store", signal: controller.signal});
    if (!response.ok) throw new Error("HTTP " + response.status);
    const data = await response.json();
    setUI(1, data.servo1); setUI(2, data.servo2); setUI(3, data.servo3);
    updateImuDisplay(data);
  }
  catch (error)
  {
    latestImuState = null;
    updateCalibrateButton();
    document.getElementById("imuStatus").textContent = "Connexion ESP32 interrompue";
    document.getElementById("viewer").classList.add("inactive");
    document.getElementById("pitchDisplay").textContent = "—";
    document.getElementById("rollDisplay").textContent = "—";
  }
  finally { clearTimeout(timeout); setTimeout(updateState, 100); }
}
window.onload = updateState;

</script>

</body>

</html>
)rawliteral";

void handleRoot()
{
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", index_html);
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

  json += "\"imuEnabled\":";
  json += imuState.enabled ? "true" : "false";

  json += ",";

  json += "\"imuConnected\":";
  json += imuState.connected ? "true" : "false";
  json += ",\"imuCalibrated\":";
  json += imuState.calibrated ? "true" : "false";
  json += ",\"imuAddress\":";
  json += String(imuState.address);
  json += ",\"imuDeviceId\":";
  json += String(imuState.deviceId);
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
  json += imuState.samples ? String(millis() - imuState.lastUpdateMs) : "0";

  json += ",\"imuDiagnostic\":\"";
  json += imuState.diagnostic; // Fixed internal messages, no user input.
  json += "\",\"imuSamples\":" + String(imuState.samples);
  json += ",\"imuCalibrationSamples\":" + String(imuState.calibrationSamples);
  json += ",\"imuCalibrationRestarts\":" + String(imuState.calibrationRestarts);
  json += ",\"imuI2cErrors\":" + String(imuState.i2cErrors);
  json += ",\"imuInterruptStatus\":" + String(imuState.interruptStatus);
  json += ",\"imuLastRegister\":" + String(imuState.lastRegister);
  json += ",\"imuLastI2cError\":" + String(imuState.lastI2cError);
  json += ",\"rollZeroDeg\":" + String(imuState.rollZeroDeg, 4);
  json += ",\"pitchZeroDeg\":" + String(imuState.pitchZeroDeg, 4);
  json += ",\"gravityG\":" + String(imuState.gravityG, 4);
  const float* vectors[] = {imuState.accelG, imuState.gyroDps, imuState.gyroBiasDps};
  const char* names[] = {"accelG", "gyroDps", "gyroBiasDps"};
  for (unsigned v = 0; v < 3; ++v)
  {
    json += ",\""; json += names[v]; json += "\":[";
    for (unsigned axis = 0; axis < 3; ++axis)
    {
      if (axis) json += ",";
      json += String(vectors[v][axis], 4);
    }
    json += "]";
  }
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleCalibrate()
{
  server.sendHeader("Cache-Control", "no-store");
  if (!ImuData::calibrate())
  {
    server.send(409, "text/plain", "Activer le mode IMU et verifier la connexion du MPU.");
    return;
  }
  server.send(200, "text/plain", "Calibration relancee");
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
  server.on("/state", handleState);
  server.on("/imu/calibrate", HTTP_POST, handleCalibrate);

  server.begin();

  Serial.println("Serveur web demarre");


}

void handleClient()
{
  server.handleClient();
}

} // namespace WifiInterface
