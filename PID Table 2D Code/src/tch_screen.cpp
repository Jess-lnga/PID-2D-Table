#include "tch_screen.h"
#include <initializer_list>

namespace TchScreen {
namespace {
constexpr uint32_t SETTLE_US = 1000;
constexpr uint32_t CONTACT_SETTLE_US = 5000;
constexpr uint32_t PERIOD_US = 10000;
enum class Phase { Idle, ContactLowBefore, ContactBefore, Layer13, Layer24, ContactLowAfter, ContactAfter };
Phase phase = Phase::Idle;
State state;
uint32_t changedUs = 0, startedUs = 0;
int rawX = 0, rawY = 0;
unsigned contactFrames = 0;
bool lowBefore = false, lowAfter = false;
int low13 = 0, low34 = 0;

void releasePanel() {
  for (int pin : {P1, P2, P3, P4}) pinMode(pin, INPUT);
  pinMode(P2_SENSE, INPUT);
}
void drive(int pin, int level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}
void detectContact(int level) {
  releasePanel();
  // The sensing layer must follow LOW then HIGH, not merely read HIGH.
  drive(P1, level);
  drive(P3, level);
  pinMode(P2, INPUT_PULLDOWN);
}
int readFiltered(int pin) {
  analogRead(pin); // Discard first conversion after switching electrodes.
  int values[5];
  for (int i = 0; i < 5; ++i) {
    values[i] = analogRead(pin);
    for (int j = i; j > 0 && values[j] < values[j-1]; --j) {
      int tmp = values[j]; values[j] = values[j-1]; values[j-1] = tmp;
    }
  }
  return values[2];
}
void finish(bool contact) {
  contactFrames = contact ? (contactFrames < 2 ? contactFrames + 1 : 2) : 0;
  state.present = contactFrames >= 2;
  if (state.present) {
    state.rawX = rawX; state.rawY = rawY;
    state.x = constrain(float(rawX - X_MIN) / (X_MAX - X_MIN), 0.0f, 1.0f);
    state.y = constrain(float(rawY - Y_MIN) / (Y_MAX - Y_MIN), 0.0f, 1.0f);
  }
  ++state.samples;
  state.lastUpdateMs = millis();
  releasePanel();
  phase = Phase::Idle;
}
}
void begin() {
  state = State{};
  contactFrames = 0;
  phase = Phase::Idle;
  analogReadResolution(12);
  analogSetPinAttenuation(P1, ADC_11db);
  analogSetPinAttenuation(P2_SENSE, ADC_11db);
  releasePanel();
  startedUs = micros() - PERIOD_US;
}
void update() {
  const uint32_t now = micros();
  if (phase == Phase::Idle) {
    if (uint32_t(now - startedUs) < PERIOD_US) return;
    startedUs = now;
    detectContact(LOW);
    phase = Phase::ContactLowBefore;
    changedUs = micros();
    return;
  }
  const bool axisPhase = phase == Phase::Layer13 || phase == Phase::Layer24;
  if (uint32_t(now - changedUs) < (axisPhase ? SETTLE_US : CONTACT_SETTLE_US)) return;
  switch (phase) {
    case Phase::ContactLowBefore:
      low13 = digitalRead(P2);
      low34 = digitalRead(P2_SENSE);
      lowBefore = low13 == LOW && low34 == LOW;
      detectContact(HIGH);
      phase = Phase::ContactBefore;
      break;
    case Phase::ContactBefore:
      state.contactLow13 = low13;
      state.contactLow34 = low34;
      state.contactHigh13 = digitalRead(P2);
      state.contactHigh34 = digitalRead(P2_SENSE);
      if (!lowBefore || state.contactHigh13 != HIGH || state.contactHigh34 != HIGH) { finish(false); return; }
      releasePanel(); drive(P1, LOW); drive(P3, HIGH);
      phase = Phase::Layer13;
      break;
    case Phase::Layer13:
      rawY = readFiltered(P2_SENSE);
      releasePanel(); drive(P2, LOW); drive(P4, HIGH);
      phase = Phase::Layer24;
      break;
    case Phase::Layer24:
      rawX = readFiltered(P1);
      detectContact(LOW);
      phase = Phase::ContactLowAfter;
      break;
    case Phase::ContactLowAfter:
      lowAfter = digitalRead(P2) == LOW && digitalRead(P2_SENSE) == LOW;
      detectContact(HIGH);
      phase = Phase::ContactAfter;
      break;
    case Phase::ContactAfter:
      finish(lowAfter && digitalRead(P2) == HIGH && digitalRead(P2_SENSE) == HIGH);
      return;
    default: return;
  }
  changedUs = micros();
}
State getState() { return state; }
}
