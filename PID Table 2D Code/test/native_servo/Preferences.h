#pragma once
#include <vector>
#include <cstring>
#include <cstdint>
inline std::vector<uint8_t> fakeNvs;
inline bool failNvsOpen=false, failNvsWrite=false;
inline unsigned nvsWrites=0;
class Preferences {
public:
  bool begin(const char*,bool=false) {return !failNvsOpen;}
  void end() {}
  size_t getBytesLength(const char*) {return fakeNvs.size();}
  size_t getBytes(const char*,void* target,size_t size) {
    if(size!=fakeNvs.size())return 0;
    std::memcpy(target,fakeNvs.data(),size);return size;
  }
  size_t putBytes(const char*,const void* source,size_t size) {
    if(failNvsWrite)return 0;
    const auto* bytes=static_cast<const uint8_t*>(source);
    fakeNvs.assign(bytes,bytes+size);++nvsWrites;return size;
  }
};
