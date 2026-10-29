#include "servo_calibration.h"
#include "servo_control.h"
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

namespace ServoCalibration
{
namespace
{
Calibration records[3];
State state;
const char* lastError = "";
unsigned long lastHeartbeat = 0, testStart = 0, lastTestWrite = 0;
int testPoint = 3;
bool moving = false;
bool leadIn = false;
int leadTicks = 0;
float leadAngle = 0;
constexpr unsigned long HOLD_MS = 1000, MOVE_MS = 600, TIMEOUT_MS = 5000;
#pragma pack(push, 1)
struct Storage
{
  uint32_t magic;
  uint16_t version;
  Calibration calibration[3];
  uint32_t checksum;
};
#pragma pack(pop)

uint32_t checksum(const Storage& storage)
{
  uint32_t value = 2166136261u;
  const auto* bytes = reinterpret_cast<const uint8_t*>(&storage);
  for (size_t i = 0; i < offsetof(Storage, checksum); ++i) value = (value ^ bytes[i])*16777619u;
  return value;
}
bool fail(const char* message) { lastError = message; return false; }
bool validCurve(const Calibration& c)
{
  if (c.valid != 1 || c.joint < 1 || c.joint > 3) return false;
  int direction = c.ticks[1] > c.ticks[0] ? 1 : -1;
  for (int i = 0; i < 4; ++i)
  {
    if (c.ticks[i] < MIN_TICKS || c.ticks[i] > MAX_TICKS) return false;
    if (i && direction*(int(c.ticks[i])-int(c.ticks[i-1])) <= 0) return false;
  }
  return true;
}
void changed() { ++state.revision; lastHeartbeat = millis(); lastError = ""; }
bool storeRecords(const Calibration* next)
{
  Storage storage{};
  storage.magic = 0x5343414C;
  storage.version = 1;
  memcpy(storage.calibration, next, sizeof(records));
  storage.checksum = checksum(storage);
  Preferences nvs;
  if (!nvs.begin("servo-cal", false)) return fail("Ouverture de la memoire NVS impossible.");
  const bool written = nvs.putBytes("records", &storage, sizeof(storage)) == sizeof(storage);
  nvs.end();
  if (!written) return fail("Echec d'enregistrement NVS; donnees precedentes conservees.");
  memcpy(records, storage.calibration, sizeof(records));
  return true;
}
void startTest()
{
  state.stage = Stage::Testing;
  testPoint = 3;
  moving = false;
  testStart = millis();
  lastTestWrite = testStart;
  leadTicks = ServoControl::getServoTicks(state.servo);
  leadAngle = state.targetDeg;
  leadIn = leadTicks != state.draft.ticks[3];
  if (!leadIn) state.targetDeg = ANGLES[3];
}
}

void begin()
{
  // Factory fallback from the user's measured -30 / 0 / 45 / 90 degree points.
  // A valid saved NVS record takes precedence, including newer calibrations.
  const uint16_t defaults[3][4] = {
    {2337, 3037, 4137, 5237}, // GPIO 25 -> q2
    {1872, 2522, 3672, 4822}, // GPIO 26 -> q1
    {2208, 2918, 3968, 5068}  // GPIO 27 -> q3
  };
  const uint8_t defaultJoints[3] = {2, 1, 3};
  for (int i = 0; i < 3; ++i) {
    records[i] = Calibration{};
    records[i].valid = 1;
    records[i].joint = defaultJoints[i];
    memcpy(records[i].ticks, defaults[i], sizeof(defaults[i]));
  }
  state = State{};
  lastError = "";
  Preferences nvs;
  if (!nvs.begin("servo-cal", true)) return;
  Storage storage{};
  bool loaded = nvs.getBytesLength("records") == sizeof(storage)
    && nvs.getBytes("records", &storage, sizeof(storage)) == sizeof(storage);
  nvs.end();
  if (!loaded || storage.magic != 0x5343414C || storage.version != 1 || storage.checksum != checksum(storage)) return;
  unsigned joints = 0;
  for (const auto& c : storage.calibration)
  {
    if (c.valid ? !validCurve(c) : c.joint > 3) return;
    if (c.joint) {
      if (joints & (1u << c.joint)) return;
      joints |= 1u << c.joint;
    }
  }
  memcpy(records, storage.calibration, sizeof(records));
  Serial.println("Calibrations servos chargees depuis NVS.");
}
bool isActive() { return state.menuActive; }
State getState() { return state; }
Calibration getCalibration(int servo) { return servo >= 1 && servo <= 3 ? records[servo-1] : Calibration{}; }
const char* error() { return lastError; }
bool openMenu()
{
  if (state.menuActive) return fail("Une session de calibration est deja ouverte.");
  state = State{};
  state.menuActive = true;
  state.session = esp_random();
  if (!state.session) state.session = 1;
  ServoControl::stopDemo();
  changed();
  return true;
}
bool checkCommand(uint32_t session, uint32_t revision)
{
  if (!state.menuActive || session != state.session || revision != state.revision)
    return fail("Session ou commande obsolete: actualiser l'etat.");
  heartbeat(session);
  return true;
}
void heartbeat(uint32_t session)
{
  if (state.menuActive && session == state.session) lastHeartbeat = millis();
}
bool chooseServo(int servo)
{
  if (!state.menuActive || state.stage != Stage::Choosing || servo < 1 || servo > 3)
    return fail("Selection du servo invalide.");
  state.servo = servo;
  state.point = 0;
  state.targetDeg = ANGLES[0];
  state.draft = Calibration{};
  state.stage = Stage::Adjusting;
  // No automatic jump: start adjusting from the current actual PWM value.
  changed();
  return true;
}
bool adjust(int delta)
{
  if (!state.menuActive || state.stage != Stage::Adjusting || delta == 0 || delta < -100 || delta > 100)
    return fail("Ajustement indisponible ou pas invalide.");
  const int ticks = ServoControl::getServoTicks(state.servo)+delta;
  if (ticks < MIN_TICKS || ticks > MAX_TICKS) return fail("Limite PWM atteinte.");
  if (!ServoControl::setServoTicks(state.servo, ticks)) return fail("Commande PWM refusee.");
  changed();
  return true;
}
bool done()
{
  if (!state.menuActive || state.stage != Stage::Adjusting) return fail("Aucun point a valider.");
  int tick = ServoControl::getServoTicks(state.servo);
  if (state.point > 0)
  {
    int delta = tick-state.draft.ticks[state.point-1];
    if (!delta || (state.point > 1 && delta*(int(state.draft.ticks[1])-int(state.draft.ticks[0])) <= 0))
      return fail("Les quatre points PWM doivent etre strictement monotones.");
  }
  state.draft.ticks[state.point] = tick;
  if (state.point < 3) state.targetDeg = ANGLES[++state.point];
  else
  {
    state.draft.valid = 1;
    state.draft.joint = 1; // Temporary placeholder until the operator assigns q.
    startTest();
  }
  changed();
  return true;
}
bool testAgain()
{
  if (!state.menuActive || state.stage != Stage::Validate) return fail("Verification indisponible.");
  startTest(); changed(); return true;
}
void update()
{
  if (!state.menuActive) return;
  if (state.stage != Stage::Paused && millis()-lastHeartbeat > TIMEOUT_MS)
  {
    state.stage = Stage::Paused;
    lastError = "Connexion perdue: mouvements suspendus. Recommencer ou quitter.";
    return; // Hold the last PWM; never resume the demo on connection loss.
  }
  if (state.stage != Stage::Testing) return;
  unsigned long elapsed = millis()-testStart;
  if (leadIn)
  {
    // Replay starts at -30: return to 90 gradually before the four 1-second holds.
    if (millis()-lastTestWrite < 20) return;
    lastTestWrite = millis();
    const float ratio = elapsed >= 1200 ? 1.0f : float(elapsed)/1200;
    state.targetDeg = leadAngle+ratio*(ANGLES[3]-leadAngle);
    int ticks;
    if (angleToTicks(state.draft, state.targetDeg, ticks)) ServoControl::setServoTicks(state.servo, ticks);
    if (ratio >= 1) { leadIn = false; testStart = millis(); }
    return;
  }
  if (!moving)
  {
    if (elapsed < HOLD_MS) return;
    if (testPoint == 0) { state.stage = Stage::Validate; return; }
    moving = true; testStart = millis(); elapsed = 0;
  }
  if (millis()-lastTestWrite < 20) return;
  lastTestWrite = millis();
  const float ratio = elapsed >= MOVE_MS ? 1.0f : float(elapsed)/MOVE_MS;
  state.targetDeg = ANGLES[testPoint]+ratio*(ANGLES[testPoint-1]-ANGLES[testPoint]);
  int ticks;
  if (angleToTicks(state.draft, state.targetDeg, ticks)) ServoControl::setServoTicks(state.servo, ticks);
  if (ratio >= 1) { --testPoint; moving = false; testStart = millis(); }
}
bool save(int joint)
{
  if (!state.menuActive || state.stage != Stage::Validate || joint < 1 || joint > 3)
    return fail("Terminer la verification et choisir q1, q2 ou q3.");
  int otherServo = -1;
  for (int i = 0; i < 3; ++i)
    if (i != state.servo-1 && records[i].joint == joint) otherServo = i;
  if (otherServo >= 0 && !records[state.servo-1].joint)
    return fail("Cet identifiant q est deja affecte; choisir un identifiant libre.");
  Calibration candidate = state.draft;
  candidate.joint = joint;
  if (!validCurve(candidate)) return fail("Calibration invalide.");
  Calibration next[3];
  memcpy(next, records, sizeof(records));
  next[state.servo-1] = candidate;
  if (otherServo >= 0) next[otherServo].joint = records[state.servo-1].joint;
  if (!storeRecords(next)) return false;
  state.stage = Stage::Choosing; state.servo = 0;
  changed();
  return true;
}
bool assignJoint(int servo, int joint)
{
  if (!state.menuActive || state.stage != Stage::Choosing || servo < 1 || servo > 3 || joint < 1 || joint > 3)
    return fail("Choisir un servo et un identifiant depuis le menu de selection.");
  if (records[servo-1].joint == joint) { changed(); return true; }
  int other = -1;
  for (int i = 0; i < 3; ++i)
    if (i != servo-1 && records[i].joint == joint) other = i;
  if (other >= 0 && !records[servo-1].joint)
    return fail("Cet identifiant est occupe; choisir un identifiant libre.");
  Calibration next[3];
  memcpy(next, records, sizeof(records));
  if (other >= 0) next[other].joint = records[servo-1].joint;
  next[servo-1].joint = joint;
  if (!storeRecords(next)) return false;
  changed();
  return true;
}
bool backToSelection()
{
  if (!state.menuActive) return fail("Aucune session ouverte.");
  state.stage = Stage::Choosing; state.servo = 0; state.draft = Calibration{};
  changed(); return true;
}
bool closeMenu()
{
  if (!state.menuActive) return fail("Aucune session ouverte.");
  state.menuActive = false; state.stage = Stage::Choosing; state.servo = 0;
  changed(); return true;
}
bool angleToTicks(const Calibration& c, float angle, int& ticks)
{
  if (!validCurve(c) || !isfinite(angle) || angle < ANGLES[0] || angle > ANGLES[3]) return false;
  int i = angle <= ANGLES[1] ? 0 : angle <= ANGLES[2] ? 1 : 2;
  const float ratio = (angle-ANGLES[i])/(ANGLES[i+1]-ANGLES[i]);
  ticks = lroundf(c.ticks[i]+ratio*(int(c.ticks[i+1])-int(c.ticks[i])));
  return true;
}
bool angleToTicks(int servo, float angle, int& ticks) { return angleToTicks(getCalibration(servo), angle, ticks); }
bool setServoAngle(int servo, float angle)
{
  if (state.menuActive) return false;
  int ticks;
  return angleToTicks(servo, angle, ticks) && ServoControl::setServoTicks(servo, ticks);
}
bool setJointAngles(float q1, float q2, float q3)
{
  if (state.menuActive) return false;
  const float angles[] = {q1, q2, q3};
  int ticks[3];
  for (int i = 0; i < 3; ++i)
    if (!records[i].valid || !angleToTicks(records[i], angles[records[i].joint-1], ticks[i])) return false;
  for (int i = 0; i < 3; ++i) ServoControl::setServoTicks(i+1, ticks[i]);
  return true;
}
}
