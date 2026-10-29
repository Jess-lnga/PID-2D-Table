# Servo calibration regression tests

The tests compile the actual `servo_control.cpp` and `servo_calibration.cpp` against fake LEDC, time and Preferences/NVS implementations. They never command hardware.

Coverage: exact PWM ticks, the four calibration points, increasing and decreasing curves, invalid/repeated/reversed points, interval interpolation, 1-second holds and gradual replay, q mapping and atomic swaps, rejection of partial angle commands, NVS save failure, restart reload, corrupted data, cancellation, stale session/revision checks and connection timeout.

From the project folder in a Visual Studio Developer Command Prompt:

```bat
cl /nologo /EHsc /std:c++17 /Itest\native_servo src\servo_control.cpp src\servo_calibration.cpp src\kinematics.cpp src\platform_control.cpp test\native_servo\test_servo.cpp /Fo"%TEMP%\\" /Fe"%TEMP%\pid-servo-test.exe"
"%TEMP%\pid-servo-test.exe"
```

An offline browser preview can be generated without connecting to the ESP32:

```powershell
python test/native_servo/generate_preview.py "$env:TEMP/servo-preview.html" --stage 1
```

Stages: 0 selection, 1 adjustment, 2 verification, 3 final save, 4 paused. The preview buttons are illustrative only; backend behavior is exercised by the C++ tests. Actual servo motion and flash retention require validation on the physical ESP32.

Also verifies measured fallback curves, NVS precedence without boot-time writes, platform-pose IK/PWM agreement across multiple cycles and no PWM writes on an unreachable pose.

IMU-control regression cases also cover calibrated pose-to-PWM mapping, no repeated commands for an unchanged sample, stale data, recalibration holds, servo-menu priority, impossible/non-finite poses and return to automatic mode.
