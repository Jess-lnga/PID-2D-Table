#include "servo_control.h"

#include <math.h>

namespace ServoControl
{
namespace
{
constexpr int MAIN_STEPS = 60;
constexpr unsigned long MAIN_STEP_DELAY_MS = 50;
constexpr int MAIN_CYCLES = 3;

constexpr int WAVE_AMPLITUDE_US = 90;
constexpr int WAVE_STEPS_PER_CYCLE = 90;
constexpr int WAVE_CYCLES = 3;
constexpr unsigned long WAVE_STEP_DELAY_MS = 50;

constexpr int TRANSITION_STEPS = 40;
constexpr unsigned long TRANSITION_DELAY_MS = 50;

enum DemoState
{
  DEMO_MOVE_TO_MAX,
  DEMO_MAIN_SWEEP,
  DEMO_MOVE_TO_WAVE_CENTER,
  DEMO_WAVE,
  DEMO_RETURN_TO_WAVE_CENTER
};

DemoState demoState = DEMO_MOVE_TO_MAX;
bool demoRunning = false;

int servo1_us = SERVO1_MID_US;
int servo2_us = SERVO2_MID_US;
int servo3_us = SERVO3_MID_US;

int mainIndex = 0;
int mainDirection = 1;
int mainCycleCount = 0;
unsigned long lastMainUpdate = 0;

int waveStep = 0;
unsigned long lastWaveUpdate = 0;

int transitionStart1 = 0;
int transitionStart2 = 0;
int transitionStart3 = 0;

int transitionTarget1 = 0;
int transitionTarget2 = 0;
int transitionTarget3 = 0;

int transitionIndex = 0;
unsigned long lastTransitionUpdate = 0;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
constexpr bool USE_LEDC_PIN_API = true;
#else
constexpr bool USE_LEDC_PIN_API = false;
constexpr int SERVO1_CHANNEL = 0;
constexpr int SERVO2_CHANNEL = 1;
constexpr int SERVO3_CHANNEL = 2;
#endif

uint32_t microsecondsToDuty(int pulse_us)
{
  return (uint32_t)(((uint64_t)pulse_us * 65535ULL) / 20000ULL);
}

int servoPin(int servoNumber)
{
  switch (servoNumber)
  {
    case 1:
      return SERVO1_PIN;
    case 2:
      return SERVO2_PIN;
    case 3:
      return SERVO3_PIN;
    default:
      return -1;
  }
}

int servoChannel(int servoNumber)
{
  if (USE_LEDC_PIN_API)
  {
    return servoPin(servoNumber);
  }

#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3)
  switch (servoNumber)
  {
    case 1:
      return SERVO1_CHANNEL;
    case 2:
      return SERVO2_CHANNEL;
    case 3:
      return SERVO3_CHANNEL;
    default:
      return -1;
  }
#else
  return -1;
#endif
}

void setServoPulseByNumber(int servoNumber, int pulse_us)
{
  uint32_t duty = microsecondsToDuty(pulse_us);
  ledcWrite(servoChannel(servoNumber), duty);
}

int interpolateValue(int startValue, int endValue, int index, int totalSteps)
{
  float ratio = (float)index / (float)(totalSteps - 1);
  return startValue + (int)((endValue - startValue) * ratio);
}

void startTransition(int target1, int target2, int target3)
{
  transitionStart1 = servo1_us;
  transitionStart2 = servo2_us;
  transitionStart3 = servo3_us;

  transitionTarget1 = target1;
  transitionTarget2 = target2;
  transitionTarget3 = target3;

  transitionIndex = 0;
  lastTransitionUpdate = 0;
}

bool updateTransition()
{
  unsigned long now = millis();

  if (now - lastTransitionUpdate < TRANSITION_DELAY_MS)
  {
    return false;
  }

  lastTransitionUpdate = now;

  servo1_us = interpolateValue(
    transitionStart1,
    transitionTarget1,
    transitionIndex,
    TRANSITION_STEPS);

  servo2_us = interpolateValue(
    transitionStart2,
    transitionTarget2,
    transitionIndex,
    TRANSITION_STEPS);

  servo3_us = interpolateValue(
    transitionStart3,
    transitionTarget3,
    transitionIndex,
    TRANSITION_STEPS);

  writeAllServos();

  transitionIndex++;

  if (transitionIndex >= TRANSITION_STEPS)
  {
    servo1_us = transitionTarget1;
    servo2_us = transitionTarget2;
    servo3_us = transitionTarget3;

    writeAllServos();

    return true;
  }

  return false;
}

void initialiseMainSweep()
{
  mainIndex = 0;
  mainDirection = 1;
  mainCycleCount = 0;
  lastMainUpdate = 0;

  Serial.println();
  Serial.println("Debut mouvement haut/bas");
}

void updateMainSweep()
{
  unsigned long now = millis();

  if (now - lastMainUpdate < MAIN_STEP_DELAY_MS)
  {
    return;
  }

  lastMainUpdate = now;

  servo1_us = interpolateValue(SERVO1_MAX_US, SERVO1_MIN_US, mainIndex, MAIN_STEPS);
  servo2_us = interpolateValue(SERVO2_MAX_US, SERVO2_MIN_US, mainIndex, MAIN_STEPS);
  servo3_us = interpolateValue(SERVO3_MAX_US, SERVO3_MIN_US, mainIndex, MAIN_STEPS);

  writeAllServos();

  if (mainDirection > 0)
  {
    if (mainIndex >= MAIN_STEPS - 1)
    {
      mainDirection = -1;
    }
    else
    {
      mainIndex++;
    }
  }
  else
  {
    if (mainIndex <= 0)
    {
      mainCycleCount++;

      Serial.print("Cycle haut/bas : ");
      Serial.println(mainCycleCount);

      if (mainCycleCount >= MAIN_CYCLES)
      {
        Serial.println("Passage vers centre de vague");

        demoState = DEMO_MOVE_TO_WAVE_CENTER;

        startTransition(
          SERVO1_WAVE_CENTER_US,
          SERVO2_WAVE_CENTER_US,
          SERVO3_WAVE_CENTER_US);

        return;
      }

      mainDirection = 1;
    }
    else
    {
      mainIndex--;
    }
  }
}

void initialiseWave()
{
  waveStep = 0;
  lastWaveUpdate = 0;

  Serial.println();
  Serial.println("Debut mouvement de vague");
}

void updateWave()
{
  unsigned long now = millis();

  if (now - lastWaveUpdate < WAVE_STEP_DELAY_MS)
  {
    return;
  }

  lastWaveUpdate = now;

  const int totalWaveSteps = WAVE_STEPS_PER_CYCLE * WAVE_CYCLES;

  float phase = 2.0f * PI * ((float)waveStep / (float)WAVE_STEPS_PER_CYCLE);
  float phase1 = phase;
  float phase2 = phase - (2.0f * PI / 3.0f);
  float phase3 = phase - (4.0f * PI / 3.0f);

  servo1_us = SERVO1_WAVE_CENTER_US + (int)(WAVE_AMPLITUDE_US * sinf(phase1));
  servo2_us = SERVO2_WAVE_CENTER_US + (int)(WAVE_AMPLITUDE_US * sinf(phase2));
  servo3_us = SERVO3_WAVE_CENTER_US + (int)(WAVE_AMPLITUDE_US * sinf(phase3));

  servo1_us = constrain(servo1_us, SERVO1_MIN_US, SERVO1_MAX_US);
  servo2_us = constrain(servo2_us, SERVO2_MIN_US, SERVO2_MAX_US);
  servo3_us = constrain(servo3_us, SERVO3_MIN_US, SERVO3_MAX_US);

  writeAllServos();

  waveStep++;

  if (waveStep >= totalWaveSteps)
  {
    Serial.println("3 cycles de vague termines");

    demoState = DEMO_RETURN_TO_WAVE_CENTER;

    startTransition(
      SERVO1_WAVE_CENTER_US,
      SERVO2_WAVE_CENTER_US,
      SERVO3_WAVE_CENTER_US);
  }
}
} // namespace

void begin()
{
  bool servo1_OK = false;
  bool servo2_OK = false;
  bool servo3_OK = false;

  if (USE_LEDC_PIN_API)
  {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    servo1_OK = ledcAttach(SERVO1_PIN, SERVO_FREQ, SERVO_RESOLUTION);
    servo2_OK = ledcAttach(SERVO2_PIN, SERVO_FREQ, SERVO_RESOLUTION);
    servo3_OK = ledcAttach(SERVO3_PIN, SERVO_FREQ, SERVO_RESOLUTION);
#endif
  }
  else
  {
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcSetup(SERVO1_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcSetup(SERVO2_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcSetup(SERVO3_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);

    ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL);
    ledcAttachPin(SERVO2_PIN, SERVO2_CHANNEL);
    ledcAttachPin(SERVO3_PIN, SERVO3_CHANNEL);

    servo1_OK = true;
    servo2_OK = true;
    servo3_OK = true;
#endif
  }

  if (!servo1_OK || !servo2_OK || !servo3_OK)
  {
    Serial.println("ERREUR configuration PWM");
  }
  else
  {
    Serial.println("PWM initialise");
  }

  servo1_us = SERVO1_MID_US;
  servo2_us = SERVO2_MID_US;
  servo3_us = SERVO3_MID_US;

  writeAllServos();
}

int getServoMicroseconds(int servoNumber)
{
  switch (servoNumber)
  {
    case 1:
      return servo1_us;
    case 2:
      return servo2_us;
    case 3:
      return servo3_us;
    default:
      return 0;
  }
}

bool setServoMicroseconds(int servoNumber, int pulseWidth)
{
  switch (servoNumber)
  {
    case 1:
      servo1_us = constrain(pulseWidth, SERVO1_MIN_US, SERVO1_MAX_US);
      setServoPulseByNumber(1, servo1_us);
      return true;

    case 2:
      servo2_us = constrain(pulseWidth, SERVO2_MIN_US, SERVO2_MAX_US);
      setServoPulseByNumber(2, servo2_us);
      return true;

    case 3:
      servo3_us = constrain(pulseWidth, SERVO3_MIN_US, SERVO3_MAX_US);
      setServoPulseByNumber(3, servo3_us);
      return true;

    default:
      return false;
  }
}

void writeAllServos()
{
  setServoPulseByNumber(1, servo1_us);
  setServoPulseByNumber(2, servo2_us);
  setServoPulseByNumber(3, servo3_us);
}

bool isDemoRunning()
{
  return demoRunning;
}

void startDemo()
{
  demoRunning = true;

  Serial.println();
  Serial.println("=== MODE DEMONSTRATION ===");

  demoState = DEMO_MOVE_TO_MAX;

  startTransition(SERVO1_MAX_US, SERVO2_MAX_US, SERVO3_MAX_US);
}

void stopDemo()
{
  demoRunning = false;
}

void updateDemo()
{
  switch (demoState)
  {
    case DEMO_MOVE_TO_MAX:
      if (updateTransition())
      {
        initialiseMainSweep();
        demoState = DEMO_MAIN_SWEEP;
      }
      break;

    case DEMO_MAIN_SWEEP:
      updateMainSweep();
      break;

    case DEMO_MOVE_TO_WAVE_CENTER:
      if (updateTransition())
      {
        initialiseWave();
        demoState = DEMO_WAVE;
      }
      break;

    case DEMO_WAVE:
      updateWave();
      break;

    case DEMO_RETURN_TO_WAVE_CENTER:
      if (updateTransition())
      {
        demoState = DEMO_MOVE_TO_MAX;
        startTransition(SERVO1_MAX_US, SERVO2_MAX_US, SERVO3_MAX_US);
      }
      break;
  }
}
} // namespace ServoControl
