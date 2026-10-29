# PID-2D-Table
This is the code and other ressources for a second version of a ball balancing robot. This time, the ball will be allowed to move in two dimensions instead of one. Go check my other projects to see the 1 D version.

The system will have a ball rolling freely on a plane. The goal is to bring the ball to the center of the table as fast as possible everytime there is a disturbance. Some fun implementation can be done later, for example, make the ball follow a circle, or other curves. This kind of project has been done multiple times, with many different architectures. Here are some good implementations that I saw:
1- https://youtu.be/kAaYaZcpbLo?si=GLI-FVm3SjZnmLwM from the AED MUSA
2-

So, not a revolution in robotics, but still a cool project. I decided to do my own implementation, but inspired from what I saw

To be able to bring the ball back to the center, we will need a way to actuate the surface on roll and pitch. To do so, we implemented a sort of Stewart platform but with less degrees of freedom (DOF). A Steward platform has 6 actuators that are placed around a surface. Most of the implementations use linear actuators. But at the end, we can move the end effector with 6 DOF. Here is an example of a Stewart platform: https://fr.wikipedia.org/wiki/Plateforme_de_Stewart

For our system, we don't need all the 6 degrees of freedom, but only two. I decided to implement a system with only 3 actuators placed regularly around the plate. We can have 3 DOF with this disposition: Translation along z, pitch and roll rotations. Theoretically, we do not need the translation on the z axis, but the height of the platform is going to affect the amplitude of the roll and pitch movement.

<img width="1536" height="2040" alt="WhatsApp Image 2026-09-24 at 22 25 52" src="https://github.com/user-attachments/assets/67091cfa-fb0b-437f-bc8a-d6e74ca5c729" />

Here are the sketchs that I did for the first implementation.


# **Actuators**

I used TIAKONGRC motors with 20 kg of torque, but my main problem is the speed of actuation. I do not know if the platform will react fast enough (I did not do enough previsions, so I am going to test and see if its enough). Here is the reference of the motors: https://bayhobbies.co.nz/tiankongac-20kg-digital-servo-high-torque-full-metal-gear-waterproof-servo-270-degree-with-25t-servo-horn/?srsltid=AU7gw4U7lks168itvwmimeYvdkK77_uPc8vz9mkhQFHDOvutGmBhPWa1

The way I am going to test it, is by manually control the motors to balance the ball instead of using a controller. To do so, I will link the system to an accelerometer.
The accelerometer will provide data for roll and pitch. The esp32 will receive it, and compute the inverse kinematics so that the platform matches the inclination of the sensor. A wired MPU-6500/9250 now provides the IMU values over I2C.

# **Software and sensors**

The MPU is always powered. A physical switch on GPIO 32 selects the mode:

- **OFF (LOW):** the existing automatic servo sequence runs, even when a Wi-Fi client is connected.
- **ON (HIGH):** the sequence stops and calibrated MPU roll/pitch commands the platform through inverse kinematics and the saved servo curves. The web page displays sensor angles and control status.

### Wiring

| MPU pin | ESP32 connection |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA/SDI | GPIO 21 |
| SCL/SCLK | GPIO 22 |
| AD0/SDO | GND (address 0x68) |
| NCS | 3V3 |
| FSYNC | GND |
| INT, EDA, ECL | Unconnected |

Switch: **3V3 -> switch -> GPIO 32**, with a **10 kOhm resistor from GPIO 32 to GND**. The firmware also enables the internal pull-down. Switch changes are debounced for 30 ms.

### Reading and display

1. Upload with PlatformIO (`pio run -t upload` from `PID Table 2D Code`).
2. Connect to the existing **PID2DTable** Wi-Fi access point and open **http://192.168.1.4**. No internet connection or external JavaScript library is required.
3. Close the switch and keep the MPU completely still for about **2 seconds**. Detected movement restarts the gyro-bias calibration. Stability is checked from sample changes and gyro variance, not from a near-zero raw gyro reading: a stable offset is what this calibration estimates. Calibration occurs on the first activation and after sensor reconnection; normal OFF/ON switching retains the gyro bias.
4. Tilt the MPU: pitch/roll readings and the cube update about 10 times per second. The serial monitor at **115200 baud** reports identification, calibration and I2C errors.

The driver checks addresses 0x68/0x69 and accepts MPU-6500 (WHO_AM_I 0x70), MPU-9250 (0x71), and MPU-9255 (0x73). It samples at 100 Hz, using +/-2 g, +/-250 degrees/s, and approximately 41 Hz internal low-pass filters. A quaternion complementary filter combines all three gyro axes with the gravity reference; only pitch and roll are displayed. No magnetometer or yaw reference is used.

Axis convention: roll rotates about sensor X, pitch about sensor Y. Flat with Z pointing up corresponds to approximately zero pitch/roll. The cube front points toward sensor -Y, its right face toward +X, and its top toward +Z. The viewer uses a fixed perspective to show the cube faces. Euler pitch is limited to +/-90 degrees, with the usual ambiguity near vertical. Accelerometer offsets are not calibrated by this version, and motion/temperature can still affect accuracy. Gyro calibration assumes genuine stillness, so do not rotate the sensor slowly during calibration.

An absent sensor or I2C failure is shown in the interface; reconnection is attempted once per second while in IMU mode. Invalid or stale readings dim the cube and hide the angles. An IMU fault never starts the automatic sequence while the switch remains ON.

The former smartphone orientation API, HTTP IMU submission routes, UDP receiver, and browser test sliders have been removed. The physical switch owns mode selection. The web **Calibrate** button restarts gyro-bias calibration when IMU mode is ON and the sensor is connected; hold it still for approximately two seconds. This recalibrates the gyro bias and sets the stationary pose as the pitch/roll manipulation zero, using the mean acceleration of the 200 accepted samples. The cube view has a fixed +90 degree rotation about sensor Z, without changing reported pitch/roll.

Register references: [TDK MPU-6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf), [TDK MPU-9250 register map](https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf).

### MPU diagnostics

The web page displays calibration progress (200 samples), restarts and their reason, received sample count and age, raw XYZ acceleration (g), its norm, raw XYZ gyro (degrees/s), measured gyro bias, INT_STATUS and I2C errors. Values and gravity-based provisional angles are visible even before calibration finishes. A completely zero acceleration vector is rejected rather than silently restarting forever. Calibration tolerates stationary gyro offsets up to 20 degrees/s; it still requires actual stillness and cannot distinguish every slow constant rotation from bias.

The serial monitor (115200 baud) logs verified configuration registers at connection, then raw measurements and calibration status once per second. I2C codes are Wire transmission errors, except 255 which means an incomplete read. DATA_RDY is enabled in INT_ENABLE even though INT is not wired; no new frame for 500 ms triggers a reported reconnect instead of leaving the UI indefinitely in calibration. Register writes are read back to verify the selected range, filters, rate, and power state.

Web requests time out after 3 seconds and retry; stale cube data is hidden. After updating the firmware, reload the web page to load the diagnostics UI.

### Manipulation zero

Both startup and manual calibration capture the stationary roll/pitch reference. Displayed angles subtract this reference (roll wraps to +/-180 degrees); the cube uses the corrected angles. Place the controller on your chosen reference surface and click Calibrate, then hold it still until completion. Diagnostics show the captured roll/pitch offsets. These offsets are retained across normal OFF/ON switching, but are recomputed after reboot or sensor reconnection. They are not saved to flash.

This is a pitch/roll angle offset for manual control, not a full change of the sensor coordinate frame or an accelerometer scale/offset calibration. For large combined rotations the Euler angle differences are not the exact rotation relative to the reference pose. The raw accelerometer and gyro diagnostics are unchanged.

## Inverse kinematics (geometry only)

`src/kinematics.h` and `.cpp` provide `Kinematics::compute_kinematics(z, rollDeg, pitchDeg)`. Configure `Kinematics::parameters` with measured L1, L2, Rb, Rp and h0, all in the same length unit. Defaults are deliberately invalid; no dimensions of this device are guessed. Servo axes are at base Z=0, at 0/120/240 degrees, and joint A starts at (Rp,0,0). Height is h0+z with z >= 0. q=0 is horizontal outward; positive q raises L1.

Rotation convention: column vectors, `R = Rz(yaw) * Ry(pitch) * Rx(roll)`. The planar constraints imply R21=R12, hence:

```text
yaw = atan2(sin(pitch)*sin(roll), cos(pitch)+cos(roll))   [branch near yaw=0]
xP = Rp/2 * (R11-R22)
yP = -Rp*R21
```

For each leg at azimuth alpha, transform its platform joint into the base frame, obtaining (X,Y,Z). Set `u = cos(alpha)*X + sin(alpha)*Y - Rb`, `v = Z`, `d = hypot(u,v)`. Then:

```text
q = atan2(v,u) + elbowSign * acos((d*d+L1*L1-L2*L2)/(2*L1*d))
```

The default elbowSign is -1 for each leg; +1 selects the other assembly branch. Set it according to the actual linkage and keep it fixed during operation. The default q limits are mathematical placeholders (-180 to 180 degrees), not measured servo travel. Configure each leg's real bounds in qMinDeg/qMaxDeg. The yaw branch is restricted near the level assembly (between -90 and 90 degrees); it is not a global solver for arbitrary flipped platform assemblies.

```cpp
#include "kinematics.h"

// Configure these from actual measurements before calling the solver:
// Kinematics::parameters.L1 = ...;
// Kinematics::parameters.L2 = ...;
// Kinematics::parameters.Rb = ...;
// Kinematics::parameters.Rp = ...;
// Kinematics::parameters.h0 = ...;
if (Kinematics::compute_kinematics(z, rollDeg, pitchDeg)) {
  // Kinematics::q1, q2, q3 are the geometric servo angles in degrees.
  // Convert via each servo's measured PWM calibration before commanding hardware.
} else {
  // Inspect Kinematics::lastError and failedLeg (0=A, 1=B, 2=C).
  // Outputs are unchanged; never treat them as a solution to the rejected pose.
}
```

The solver has static outputs q1/q2/q3 and xP/yP/yawDeg, rejects invalid parameters, non-finite inputs, unreachable link lengths, indeterminate configurations and joint-limit violations. It does not check collisions, passive-joint articulation limits or parallel-platform stiffness singularities. It is intentionally not connected to the servos or the IMU yet.

Host regression tests in `test/native_kinematics/test_kinematics.cpp` independently verify radial-plane constraints and L2 link closure over a combined roll/pitch grid, for both assembly branches, plus failures and preservation of the last valid solution. For example, from the project directory: `g++ -std=c++17 src/kinematics.cpp test/native_kinematics/test_kinematics.cpp -o /tmp/test-kinematics && /tmp/test-kinematics`.

## Interactive servo calibration (ESP32 web interface)

Open **Calibrate servo** in the main web menu. Entering this menu suspends the automatic sequence irrespective of the physical switch; the other servos hold their current PWM positions. Select a physical servo (GPIO 25/26/27), then adjust its **actual 16-bit LEDC ticks** using +/- and selectable steps of 1, 10 or 50 ticks. One tick is approximately 0.305 microseconds at 50 Hz. Each press is acknowledged before another command is accepted.

The targets are **-30, 0, 45, 90 degrees**, in that order. Zero is the radial horizontal outward position; positive angles raise the palonnier. The animated 2D drawing is a target indication, not measured feedback. Align the real palonnier, then press **Done**. The PWM points must be strictly increasing or strictly decreasing; mixed directions and identical consecutive points are rejected.

After the fourth point, the ESP32 automatically verifies **90 -> 45 -> 0 -> -30 degrees**, with a one-second hold at each position and 600 ms transitions. This runs without blocking IMU acquisition or HTTP handling. Replay first returns gradually to 90 degrees over 1.2 seconds. If satisfied, select **q1/q2/q3** and press **Valider et enregistrer**. The four points and mapping are stored together in ESP32 NVS flash via Preferences and reloaded after restart. Failed writes leave the saved calibration in RAM unchanged; the UI stays at validation so the operator can retry.

The saved curves are piecewise linear on [-30,0], [0,45], [45,90]. The two directions of PWM/angle relation are supported. IDs remain unique: an uncalibrated servo must choose a free q ID; reassigning an already calibrated servo to an occupied q ID swaps the two IDs in the same NVS record. The UI labels this exchange. Cancel/back discards the draft and preserves the saved data. After saving, the UI returns to servo selection; the demo stays suspended until **Retour au menu principal**.

The adjustment envelope is **500-2500 microseconds**, bounded to ticks 1639-8191. These are configurable electrical bounds in `src/servo_calibration.h`, not verified mechanical limits. Legacy microsecond commands retain their narrower bounds; calibration and the automatic wave use direct ticks derived from their angle curves. Tune the calibration envelope to the actual servo/linkage if needed.

A session/revision check rejects stale or repeated POST commands. Session polling acts as a heartbeat; after five seconds without it, the controller stops the verification and holds the latest PWM. The calibration menu remains active, so an OFF switch cannot unexpectedly resume the demo on a network failure. Reloading the page can resume the same session; a paused calibration requires starting selection again or explicitly leaving the menu. No draft is automatically written to flash.

Angle-command entry points:

```cpp
// Uses the physical servo's saved curve; refuses missing calibration/out-of-range angle.
ServoControl::setServoAngle(1, 22.5f);
// Uses all three stored q mappings; validates all angles before writing any PWM.
ServoControl::setKinematicAngles(q1, q2, q3);
```

Both angle APIs reject requests while the interactive calibration menu owns the servos. They always use the saved calibration, with no guessed fallback. The automatic demo now uses the calibrated angle curves; the IMU/kinematics-to-servo loop is still not enabled by this change.

Implementation: `servo_calibration.*` (state machine, curves, NVS) and `servo_calibration_web.*` (web menu and POST routes). Hardware-independent regression tests are in `test/native_servo`.

### Edit servo IDs independently

The servo selection menu has a q1/q2/q3 selector and **Enregistrer l’identifiant** on each card. This changes only the persistent mapping, without moving the servo or altering its PWM calibration points. It is also possible to assign a free ID before calibrating a servo; angle commands still refuse an uncalibrated curve. Selecting an occupied ID swaps the mappings if the current servo already has an ID; otherwise select a free ID. Flash failures preserve the previous data.

Dropdowns and servo cards are no longer recreated by each polling refresh. They rebuild only when the saved records/session change; a pending operator selection remains intact between updates.

### Automatic platform-pose wave

Switch OFF now generates a wave in platform roll/pitch, not directly in motor angles:

```text
z = WAVE_Z_MM = 0 mm above h0
roll  = 5 * sin(phase) degrees
pitch = 5 * cos(phase) degrees
phase = 2*pi*time/4500 ms
```

The IK solver computes lateral compensation, parasitic yaw and q1/q2/q3 for each pose. These q angles are converted through the saved per-servo calibration and joint mapping into actual PWM ticks. The period is 4.5 seconds with updates every 20 ms and a 1.5-second PWM easing transition from the previous positions at startup/resume. The target pose displayed during this initial easing is not measured feedback.

The +/-5 degree trajectory at z=0 was checked over a full cycle against the current measured dimensions (h0=60 mm); +/-10 degrees at that height is not valid for the full cycle. Tune WAVE_Z_MM, WAVE_ROLL_AMPLITUDE_DEG, WAVE_PITCH_AMPLITUDE_DEG and WAVE_PERIOD_MS in servo_control.h. Out-of-reach poses or missing/out-of-range calibration targets hold all three PWM outputs; the web interface and serial diagnostics report the problem. IMU mode and the interactive calibration menu still suspend the automatic movement. This is an open-loop IK test, not feedback control of the measured platform orientation.

### Initial measured calibrations

At startup, valid saved NVS records always take priority. If NVS is absent or corrupt, use these measured fallback points (-30, 0, 45, 90 degrees):

| Physical servo | GPIO | Joint | PWM ticks |
| --- | --- | --- | --- |
| 1 | 25 | q2 | 2337 / 3037 / 4137 / 5237 |
| 2 | 26 | q1 | 1872 / 2522 / 3672 / 4822 |
| 3 | 27 | q3 | 2208 / 2918 / 3968 / 5068 |

These defaults are read into RAM without rewriting flash at boot. Draft adjustment and verification never overwrite the saved records; only final validation saves a new curve. Cancel, timeout and write failure preserve previous values. Explicit ID-only changes save the mapping without changing calibration points. Uploading this firmware therefore preserves newer valid calibrations, while a fresh device starts from the photo's values.

## IMU-driven platform control

With the switch ON, `platform_control.cpp` sends the calibrated, zero-referenced MPU roll/pitch into `Kinematics::compute_kinematics(IMU_Z_MM, roll, pitch)`, then sends q1/q2/q3 through the saved servo mapping and PWM calibration. Height remains h0 + IMU_Z_MM (default z=0 mm). Control updates at most every 20 ms, only for a new IMU sample.

The interactive servo calibration menu owns PWM and overrides both automatic and IMU modes. During gyro/zero calibration, MPU disconnection, absent data, age >=250 ms, non-finite angles, unreachable IK poses or missing/out-of-range servo calibration, no new platform PWM is issued: the servos hold the previous commanded positions. Unreachable angles are not silently clamped. Turning the switch OFF resumes the automatic platform wave.

`src/platform_control.h` contains IMU_Z_MM, IMU_ROLL_SIGN and IMU_PITCH_SIGN. Both signs default to +1 so the measured angles are transmitted directly. The cube's CSS/view sign adjustments remain display-only. The web page shows the control status, requested pose and any IK error. This is open-loop position control from the handheld IMU; there is no measurement of the actual platform orientation used for feedback.

After uploading, reload the web page, turn ON and keep the IMU still until calibration finishes. Begin with small tilts: valid workspace depends on the chosen height and dimensions. At current z=0/h0=60 mm, some larger inclinations exceed the mechanism's limits and will be rejected.

## Height control and lightweight web display

The web main menu now contains the 3D cube, the IMU Calibrate button and a height input. IMU diagnostic text, raw-vector tables, angle readouts and motor/IK status text have been removed from the interface; serial diagnostics remain available. Height is **z in mm above h0**, so physical center height is h0+z. Click Appliquer to submit a new desired height. It is shared by automatic and IMU modes and resets to the default on reboot (not saved to flash).

A height update is checked against the current requested roll/pitch and all calibrated servo ranges, before acceptance. Invalid/non-finite/negative heights or an active servo-calibration menu reject the request without changing the stored height or PWM. A height acceptable at one inclination does not guarantee that every subsequent inclination is reachable. Every runtime pose is still checked; IK rejection or an angle outside the calibrated range issues no PWM writes for any leg and holds the last commanded positions.

The MPU data is acquired locally at 100 Hz and used for servo commands at up to 50 Hz; browser telemetry is not in the control path. HTTP is handled on a separate FreeRTOS task pinned to core 0, while the ESP32 Arduino control loop runs on core 1. A recursive mutex serializes shared state and calibration/height changes; HTTP socket sending occurs outside this lock. Page polling is 4 Hz normally and 5 Hz while calibrating servos for its heartbeat. JSON omits the large IMU diagnostics. This reduces visualization traffic and prevents a slow client/socket send from holding up the control loop; flash writes and sensor reconnection still have their inherent latency.

### Noise-tolerant MPU calibration

Calibration still averages 200 accepted frames for the gyro bias and the gravity reference, but no longer requires every raw frame to be perfect. Changes greater than 0.20 g or 8 degrees/s relative to the last accepted frame are discarded, without erasing previous accepted samples. Invalid gravity vectors and raw gyro values above 20 degrees/s are also discarded. Twenty consecutive rejects (~200 ms) or more than 30 rejected frames per window restart the window.

At 200 accepted frames, verify per-axis gyro standard deviation <=1.5 degrees/s and accelerometer standard deviation <=0.04 g. This accepts modest stationary gyro noise which previously failed the 0.5 degrees/s threshold, while checking the full-window gravity stability so actual platform tilt is not treated as gyro bias. Thresholds are grouped at the top of imu_data.cpp. The initial filter attitude uses the averaged gravity reference rather than a potentially noisy last frame.

The Calibrate button shows the progress percentage; no large diagnostic payload is reintroduced. Typical duration is about 2 seconds, slightly longer with discarded frames. Invalid I2C reads remain errors, and calibration never succeeds solely because a timeout expired. Servo motion remains suspended until calibration has actually passed. Keep the IMU genuinely still: very slow constant rotation can still be indistinguishable from gyro bias.

### Strictly manual MPU calibration

Calibration now starts **only** on an explicit Calibrate button POST. Boot, switch ON/OFF and MPU reconnection do not start calibration. Before the first successful manual calibration after boot, sensor data may animate the cube, but platform PWM is held in IMU mode. A successful bias/zero reference is retained in RAM across switch changes and reconnection.

A failed attempt stops and waits for another click, instead of automatically restarting calibration windows. Turning OFF or an I2C disconnect also cancels a running attempt. If a previous accepted calibration exists, its bias/zero are preserved and restored when the new attempt fails/cancels; the new values replace them only on successful completion. These MPU calibration values are not stored to flash, so a reboot requires an explicit calibration click again. Servo NVS calibration is unaffected.
