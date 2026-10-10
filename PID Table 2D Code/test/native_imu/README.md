# Native MPU regression tests

These tests compile the production `src/imu_data.cpp` against a fake clock, switch and I2C register bus. No sensor or ESP32 is needed. They cover a stationary 5 degrees/s bias, calibration progress, motion and variance rejection, zero acceleration before/after calibration, missing DATA_RDY, incomplete reads, I2C errors, reconnection, mode switching and configuration read-back failure.

From the PlatformIO project directory, in a Visual Studio Developer Command Prompt:

```bat
cl /nologo /EHsc /std:c++17 /Itest\native_imu test\native_imu\test_imu.cpp /Fo"%TEMP%\pid-imu-test.obj" /Fe"%TEMP%\pid-imu-test.exe"
"%TEMP%\pid-imu-test.exe"
```

Or with a host C++ compiler such as g++:

```sh
g++ -std=c++17 -Itest/native_imu test/native_imu/test_imu.cpp -o /tmp/pid-imu-test
/tmp/pid-imu-test
```

These are host tests only; real sensor timing, noise, power and wiring still require hardware validation.
