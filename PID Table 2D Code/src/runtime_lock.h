#ifndef RUNTIME_LOCK_H
#define RUNTIME_LOCK_H
#ifdef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
namespace RuntimeLock {
extern SemaphoreHandle_t mutex;
void begin();
class Guard {
  bool held = true;
public:
  Guard() { xSemaphoreTakeRecursive(mutex, portMAX_DELAY); }
  void release() { if (held) { xSemaphoreGiveRecursive(mutex); held=false; } }
  ~Guard() { release(); }
  Guard(const Guard&) = delete;
};
}
#endif
#endif
