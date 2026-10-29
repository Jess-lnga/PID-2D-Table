#include "wifi_interface.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "imu_data.h"
#include "servo_control.h"
#include "servo_calibration.h"
#include "servo_calibration_web.h"
#include "platform_control.h"
#include "runtime_lock.h"
#include "tch_screen.h"
#include <errno.h>

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
.touch-area { position:relative; width:100%; aspect-ratio:1; background:linear-gradient(90deg,transparent 49.7%,#cbd5e1 50%,transparent 50.3%),linear-gradient(transparent 49.7%,#cbd5e1 50%,transparent 50.3%),#f4f6f8; border:2px solid #64748b; box-sizing:border-box; }
.touch-ball { position:absolute; width:18px; height:18px; border-radius:50%; background:#e85d25; transform:translate(-50%,-50%); box-shadow:0 2px 5px #0004; }
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
<div id="mainMenu">
<button id="openServoCal" onclick="openCalMenu()">Calibrate servo</button>

<div class="subtitle">
ESP32 Servo Controller
</div>

<div class="servo-box">
<h2>Position de la balle</h2>
<p id="touchStatus" role="status">Connexion à l’ESP32…</p>
<div class="touch-area" role="img" aria-label="Position sur la surface tactile, X vers la droite, Y vers le bas">
<div id="touchBall" class="touch-ball" hidden></div>
</div>
<p id="touchCoordinates">X : — | Y : —</p>
<p id="touchRaw" class="imu-note"></p>
<p id="touchContact" class="imu-note"></p>
<p class="imu-note">X : couche 2–4, vers la droite. Y : couche 1–3, vers le bas. Position indicative avant calibration des limites. La balle doit exercer une pression suffisante pour établir un contact.</p>
</div>

<div class="servo-box">

<h2>Orientation MPU</h2>
<button id="calibrateButton" onclick="calibrateImu()" disabled>Calibrate</button>
<div id="calibrateFeedback" role="status"></div>
<p id="imuStatus" class="imu-status" role="status">Connexion à l’ESP32…</p>
<div class="imu-grid">
<div class="imu-reading">Roll<span id="imuRoll">—</span></div>
<div class="imu-reading">Pitch<span id="imuPitch">—</span></div>
</div>
<div class="imu-note">Calibration uniquement sur demande : cliquer sur Calibrate et garder le MPU immobile.</div>
<pre id="imuDiagnostics" style="text-align:left;white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px;line-height:1.6" aria-label="Diagnostics IMU"></pre>
<div class="viewer inactive" id="viewer">
<div class="cube-view"><div class="cube-heading"><div class="cube" id="cube">
<div class="face front">Avant</div><div class="face back">Arrière</div>
<div class="face right">X</div><div class="face left">−X</div>
<div class="face top">Z ↑</div><div class="face bottom">−Z</div>
</div></div></div></div>

</div>

<div class="servo-box">
<h2>Hauteur de la plateforme</h2>
<label>z au-dessus de h0 (mm) : <input id="heightInput" type="number" min="0" step="1" value="25"></label>
<button id="heightButton" onclick="setHeight()">Appliquer</button>
<p id="heightFeedback" role="status"></p>
</div>

</div>
<!--SERVO_CAL_MENU-->
<script>
/*SERVO_CAL_SCRIPT*/

let calibrationRequestPending = false;
let latestImuState = null;

function updateTouchDisplay(touch) {
  const fresh = touch && touch.samples > 0 && touch.ageMs < 250;
  document.getElementById('touchContact').textContent = fresh ? 'Test contact LOW / HIGH : GPIO13 = ' + touch.contactLow13 + ' / ' + touch.contactHigh13 + ' ; GPIO34 = ' + touch.contactLow34 + ' / ' + touch.contactHigh34 : '';
  const present = fresh && touch.present;
  const ball = document.getElementById('touchBall');
  ball.hidden = !present;
  document.getElementById('touchStatus').textContent = !fresh ? 'Mesure tactile indisponible' : present ? 'Contact détecté' : 'Aucune balle détectée';
  document.getElementById('touchCoordinates').textContent = present ? 'X : ' + (touch.x*100).toFixed(1) + ' % | Y : ' + (touch.y*100).toFixed(1) + ' %' : 'X : — | Y : —';
  document.getElementById('touchRaw').textContent = present ? 'ADC brut : axe 1–3 = ' + touch.rawY + ' | axe 2–4 = ' + touch.rawX : '';
  if (present) {
    ball.style.left = (touch.x*100) + '%';
    ball.style.top = (touch.y*100) + '%';
  }
}

function updateCalibrateButton()
{
  document.getElementById("calibrateButton").textContent = latestImuState && latestImuState.imuEnabled && latestImuState.imuConnected && latestImuState.imuCalibrationActive ?
    "Calibration " + Math.round((latestImuState.imuCalibrationSamples || 0)/2) + "%" : "Calibrate";
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
  const number = (value, digits=3) => Number.isFinite(value) ? value.toFixed(digits) : "—";
  const hex = value => Number.isFinite(value) ? "0x" + value.toString(16).toUpperCase().padStart(2,"0") : "—";
  const vector = values => Array.isArray(values) ? values.map(value => number(value)).join(" / ") : "—";
  const calibration = data.imuCalibrationActive ? "EN COURS" : data.imuCalibrationFailed ? "INTERROMPUE" : "inactive";
  document.getElementById("imuStatus").textContent = data.imuDiagnostic || "État indisponible";
  document.getElementById("imuRoll").textContent = number(data.roll,2) + "°";
  document.getElementById("imuPitch").textContent = number(data.pitch,2) + "°";
  document.getElementById("imuDiagnostics").textContent = [
    "Interrupteur GPIO 32 : " + (data.imuEnabled ? "ON" : "OFF"),
    "MPU connecté : " + (data.imuConnected ? "oui" : "non") + " | Adresse : " + hex(data.imuAddress) + " | WHO_AM_I : " + hex(data.imuDeviceId),
    "Calibration : " + calibration + " | Référence valide : " + (data.imuCalibrated ? "oui" : "non"),
    "Référence enregistrée pour les prochains démarrages : " + (data.imuCalibrationSaved ? "oui" : "non"),
    "Progression : " + data.imuCalibrationSamples + "/200 échantillons | Échecs de cette tentative : " + data.imuCalibrationRestarts,
    "Données valides : " + (data.imuHasData ? "oui" : "non") + " | Échantillons lus : " + data.imuSamples + " | Âge : " + (data.imuSamples ? data.imuAgeMs + " ms" : "aucune mesure"),
    "Accélération X / Y / Z (g) : " + vector(data.imuAccelG),
    "Norme accélération (g) : " + number(data.imuGravityG),
    "Gyroscope X / Y / Z (°/s) : " + vector(data.imuGyroDps),
    "Biais gyro X / Y / Z (°/s) : " + vector(data.imuGyroBiasDps),
    "Zéro roll / pitch (°) : " + number(data.imuRollZeroDeg) + " / " + number(data.imuPitchZeroDeg),
    "Erreurs I²C : " + data.imuI2cErrors + " | Dernier registre : " + hex(data.imuLastRegister) + " | Code : " + data.imuLastI2cError,
    "Tentatives I²C supplémentaires : " + data.imuI2cRetries + " | Récupérations bus : " + data.imuI2cRecoveries + " | Trames perdues consécutives : " + data.imuI2cConsecutiveFailures,
    "INT_STATUS : " + hex(data.imuInterruptStatus)
  ].join("\n");
  if (data.imuCalibrationFailed) document.getElementById("calibrateFeedback").textContent = "Calibration interrompue. Immobiliser le MPU puis cliquer sur Calibrate pour réessayer.";
  updateCalibrateButton();
  const valid = data.imuEnabled && data.imuConnected && data.imuHasData && data.imuAgeMs < 250;
  document.getElementById("viewer").classList.toggle("inactive", !valid);
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

async function setHeight()
{
  const button=document.getElementById('heightButton'); button.disabled=true;
  const controller=new AbortController(), timeout=setTimeout(()=>controller.abort(),3000);
  try {
    const response=await fetch('/platform/height',{method:'POST',body:new URLSearchParams({z:document.getElementById('heightInput').value}),signal:controller.signal});
    const message=await response.text();
    document.getElementById('heightFeedback').textContent=message;
  } catch(error) { document.getElementById('heightFeedback').textContent='Commande non confirmée : connexion interrompue.'; }
  finally {clearTimeout(timeout);button.disabled=false;}
}

async function updateState()
{
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try
  {
    const response = await fetch("/state?calSession=" + calToken, {cache: "no-store", signal: controller.signal});
    if (!response.ok) throw new Error("HTTP " + response.status);
    const data = await response.json();
    if (document.activeElement!==document.getElementById('heightInput')) document.getElementById('heightInput').value=data.heightMm;
    updateImuDisplay(data);
    updateTouchDisplay(data.touch);
    renderServoCalibration(data.servoCalibration);
  }
  catch (error)
  {
    calDisableCommands(true);
    latestImuState = null;
    updateTouchDisplay(null);
    updateCalibrateButton();
    document.getElementById("viewer").classList.add("inactive");
    document.getElementById("imuStatus").textContent = "Connexion web interrompue : " + error.message + " (dernières mesures conservées à l’écran)";
  }
  finally { clearTimeout(timeout); setTimeout(updateState, calState && calState.menuActive ? 200 : 100); }
}
window.onload = updateState;

</script>

</body>

</html>
)rawliteral";

void handleRoot()
{
  server.sendHeader("Cache-Control", "no-store");
  String page = FPSTR(index_html);
  page.replace("<!--SERVO_CAL_MENU-->", ServoCalibrationWeb::html());
  page.replace("/*SERVO_CAL_SCRIPT*/", ServoCalibrationWeb::script());
  server.send(200, "text/html; charset=utf-8", page);
}

void handleState()
{
  RuntimeLock::Guard guard;
  // A valid session heartbeat prevents calibration motion from surviving a lost UI.
  if (server.hasArg("calSession"))
  {
    const String token = server.arg("calSession");
    char* end = nullptr;
    const unsigned long value = strtoul(token.c_str(), &end, 10);
    if (end != token.c_str() && *end == 0) ServoCalibration::heartbeat(value);
  }
  ImuData::State imuState = ImuData::getState();

  String json = "{";
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

  json += ",\"imuCalibrationActive\":"; json += imuState.calibrationActive ? "true" : "false";
  json += ",\"imuCalibrationFailed\":"; json += imuState.calibrationFailed ? "true" : "false";
  json += ",\"imuCalibrationSaved\":"; json += imuState.calibrationSaved ? "true" : "false";
  json += ",\"imuCalibrationSamples\":" + String(imuState.calibrationSamples);
  json += ",\"imuDiagnostic\":\"" + String(imuState.diagnostic) + "\"";
  json += ",\"imuSamples\":" + String(imuState.samples);
  json += ",\"imuCalibrationRestarts\":" + String(imuState.calibrationRestarts);
  json += ",\"imuI2cErrors\":" + String(imuState.i2cErrors);
  json += ",\"imuI2cRetries\":" + String(imuState.i2cRetries);
  json += ",\"imuI2cRecoveries\":" + String(imuState.i2cRecoveries);
  json += ",\"imuI2cConsecutiveFailures\":" + String(imuState.i2cConsecutiveFailures);
  json += ",\"imuLastRegister\":" + String(imuState.lastRegister);
  json += ",\"imuLastI2cError\":" + String(imuState.lastI2cError);
  json += ",\"imuInterruptStatus\":" + String(imuState.interruptStatus);
  json += ",\"imuGravityG\":" + String(imuState.gravityG, 4);
  json += ",\"imuRollZeroDeg\":" + String(imuState.rollZeroDeg, 3);
  json += ",\"imuPitchZeroDeg\":" + String(imuState.pitchZeroDeg, 3);
  const float* vectors[] = {imuState.accelG, imuState.gyroDps, imuState.gyroBiasDps};
  const char* keys[] = {"imuAccelG", "imuGyroDps", "imuGyroBiasDps"};
  for (unsigned i = 0; i < 3; ++i) {
    json += ",\""; json += keys[i]; json += "\":[";
    for (unsigned axis = 0; axis < 3; ++axis) {
      if (axis) json += ",";
      json += String(vectors[i][axis], 4);
    }
    json += "]";
  }
  json += ",\"heightMm\":" + String(PlatformControl::heightMm(), 3);
  json += ",\"servoCalibration\":" + ServoCalibrationWeb::stateJson();
  const auto touch = TchScreen::getState();
  json += ",\"touch\":{\"present\":"; json += touch.present ? "true" : "false";
  json += ",\"contactLow13\":" + String(touch.contactLow13) + ",\"contactHigh13\":" + String(touch.contactHigh13);
  json += ",\"contactLow34\":" + String(touch.contactLow34) + ",\"contactHigh34\":" + String(touch.contactHigh34);
  json += ",\"rawX\":" + String(touch.rawX) + ",\"rawY\":" + String(touch.rawY);
  json += ",\"x\":" + String(touch.x, 4) + ",\"y\":" + String(touch.y, 4);
  json += ",\"samples\":" + String(touch.samples);
  json += ",\"ageMs\":" + String(touch.samples ? millis() - touch.lastUpdateMs : 0);
  json += "}";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  guard.release();
  server.send(200, "application/json", json);
}

void handleCalibrate()
{
  server.sendHeader("Cache-Control", "no-store");
  bool ok; { RuntimeLock::Guard guard; ok = ImuData::calibrate(); }
  if (!ok)
  {
    server.send(409, "text/plain", "Activer le mode IMU et verifier la connexion du MPU.");
    return;
  }
  server.send(200, "text/plain", "Calibration relancee");
}

void handleHeight()
{
  const String text=server.arg("z"); char* end=nullptr; errno=0;
  float z=strtof(text.c_str(), &end);
  bool ok=false;
  if (!errno && end!=text.c_str() && *end==0) { RuntimeLock::Guard guard; ok=PlatformControl::setHeightMm(z); }
  server.send(ok?200:409,"text/plain; charset=utf-8",ok?"Hauteur mise à jour.":"Hauteur refusée : pose impossible, calibration indisponible ou menu servo actif.");
}
void webTask(void*)
{
  for (;;) { server.handleClient(); vTaskDelay(1); }
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
  server.on("/platform/height", HTTP_POST, handleHeight);
  ServoCalibrationWeb::registerRoutes(server);
  server.on("/imu/calibrate", HTTP_POST, handleCalibrate);

  server.begin();
  if (xTaskCreatePinnedToCore(webTask,"http",8192,nullptr,1,nullptr,0)!=pdPASS) Serial.println("ERREUR creation tache HTTP");

  Serial.println("Serveur web demarre");


}

void handleClient()
{
  server.handleClient();
}

} // namespace WifiInterface
