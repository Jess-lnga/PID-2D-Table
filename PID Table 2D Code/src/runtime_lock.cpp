#include "runtime_lock.h"
#ifdef ARDUINO
namespace RuntimeLock {
SemaphoreHandle_t mutex = nullptr;
void begin() { mutex = xSemaphoreCreateRecursiveMutex(); configASSERT(mutex); }
}
#endif
