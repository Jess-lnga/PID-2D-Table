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

New regression cases cover modest stationary gyro noise, preserving 190 accepted samples through a single severe spike, rejecting sustained movement and excessive window variance, and detecting slowly changing gravity.

Persistence cases simulate a power cycle, restore gyro biases and roll/pitch references,
verify no writes during normal sampling, preserve the saved reference after a failed
recalibration, and reject corrupt/truncated records, unsupported versions and a different
MPU model. NVS open/write failures are also covered.

On hardware, the first successful manual calibration saves the reference in the
`imu-cal` NVS namespace. Subsequent boots reuse it; Calibrate replaces it only after
a successful measurement. The web diagnostics show whether the current reference
was saved. If saving fails, the calibration remains usable for the current session.
The default platform height is 25 mm above h0. Verify retention with a power cycle.
