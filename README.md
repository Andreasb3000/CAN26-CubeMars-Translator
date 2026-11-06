# Husky Robotics — CAN26 ↔ CubeMars V3 Servo Translator

**Deliverable 2: bidirectional, software-only prototype**

This project extends Deliverable 1 with decoding of CubeMars Servo feedback and creation of CAN26 encoder-estimate packets. It is meant for VS Code on a Mac, before STM32/FDCAN integration. **Nothing in this folder sends a physical CAN frame.**

## Folder layout

Keep the CAN26 library alongside this project:

```
HuskyRobotics/
├── CAN26/                               # existing Husky Robotics clone
└── CubeMars_Bidirectional_Deliverable2/
    ├── CubeMarsServo.h/.c               # existing CAN26->motor speed conversion/encoding
    ├── CAN26CubeMars.h/.c               # existing CAN26 velocity decoder/adapter
    ├── CubeMarsFeedback.h/.c            # NEW: CubeMars 0x29 motor feedback decoder
    ├── CAN26CubeMarsFeedback.h/.c       # NEW: motor feedback to CAN26 encoder estimates
    ├── test_core.c / test_can26.c       # previous tests
    ├── test_feedback_core.c             # NEW: portable feedback tests
    ├── test_feedback_can26.c            # NEW: CAN26 integration and request/reply tests
    └── Makefile
```

**Already working on Deliverable 1?** To avoid overwriting your edits, you can copy only the four new `*Feedback.h/.c` files and two `test_feedback_*.c` tests into your original folder. You can also copy the `Makefile`. The tests still expect `../CAN26/` relative to whichever directory you run them from.

## Build and run on macOS

From this directory:

```sh
# Tests that DO NOT need CAN26 installed
make test

# Tests that DO need the real CAN26 repository alongside this folder
make test-can26

# Or run just the new tests:
make test_feedback_core && ./test_feedback_core
make test_feedback_can26 && ./test_feedback_can26
```

If you prefer direct compiler commands:

```sh
cc -std=c11 -Wall -Wextra CubeMarsFeedback.c test_feedback_core.c \
    -lm -o test_feedback_core
./test_feedback_core

cc -std=c11 -Wall -Wextra -I../CAN26 \
    CubeMarsFeedback.c CAN26CubeMarsFeedback.c test_feedback_can26.c \
    ../CAN26/CANPacket.c -lm -o test_feedback_can26
./test_feedback_can26
```

## What the new code does

1. `CubeMarsDecodeServoFeedback()` verifies the 29-bit extended identifier `0x2901` for driver ID 1 and exact 8-byte payload length. It reads position (0.1 degrees/count), speed (10 ERPM/count), IQ (0.01 A/count), signed driver temperature, and the motor's status/fault code.
2. `CubeMarsFeedbackToMechanical()` converts the motor's electrical RPM to rev/s. It converts the position from degrees to rev using **an explicit configuration assumption**. It rejects nonzero status/fault codes, including special `0x77` disable confirmation.
3. `CubeMarsToCAN26EncoderEstimates()` uses Husky Robotics' existing `CANMotorPacket_BLDC_EncoderEstimates()` constructor to prepare a CAN26 `0x0E` response addressed to a chosen receiver.
4. `CAN26ReplyToEncoderEstimateRequest()` demonstrates request/response behavior for CAN26 `GetEncoderEstimates` (`0x0D`, encoder index 0). It requires the caller to affirm the last CubeMars feedback is fresh; the module itself **does not** implement a clock, cache, or watchdog.

### Example feedback

Simulated CubeMars 29-bit extended CAN identifier: `0x2901`

```
03 84   00 64   00 7D   23   00
  |       |       |      |    |
 +90°  +1000ERPM 1.25A   35°C no fault
```

At the **motor shaft**, 1000 ERPM ÷ (14 pole pairs × 60) = approximately **1.190476 rev/s**. At the **39:1 gearbox output**, it is approximately **0.030525 rev/s**.

The CAN26 response constructed by the real CAN26 library holds position in revolutions and velocity in rev/s, encoded using its two BFloat24 fields. For the test case with **motor-shaft position assumption** and motor-shaft output, position is +0.25 rev (90°) and speed is approximately +1.190476 rev/s.

## Important review decisions before hardware use

- **Position reference is not established.** The AK V3 manual defines `0x29` position as 0.1 degrees/count, but does not clearly say whether that angle is measured at the motor shaft or gearbox output. The `position_is_motor_shaft` argument is an explicit test choice, not a confirmed fact. `config.input_is_joint_speed` selects the CAN26 mechanical reporting reference. Verify both against the real motor and team command conventions.
- **Position range / origin:** the standard 0x29 field spans roughly +/-3200 degrees, and may be insufficient for a continuous absolute position record. The V3 protocol offers optional 0x2A position feedback with a larger range. The origin and wrap behavior must be characterized before controlling the robot arm.
- **Faults:** decoding a frame is not confirmation that the motor is healthy. The CAN26 encoder-estimates constructor has no fault field. This adapter refuses to publish normal-looking encoder estimates for a nonzero CubeMars status, but you still need separate fault, E-Stop, and heartbeat handling.
- **Freshness:** only use the request/reply helper with a recent status frame, measured using the STM32's monotonic time. A stale cached frame must never be presented as current feedback. A `true` Boolean in a test is not a firmware freshness implementation.
- **Driver ID / velocity limits / gear ratio / bus rates:** test defaults are not production settings. Verify with hardware.
- **CAN26 support:** request/reply helper currently supports encoder index 0 only and does not send CAN acknowledgments or communicate with the CubeMars hardware itself.
- **No STM32 HAL yet:** FDCAN reception, transmission, low-level filters, timeout logic, and physical hardware tests are a later milestone.

## Primary references

- CubeMars *AK Series Motor Module V3 Driver Software User Manual*, §2.3.1 (Servo status message 0x29, scale factors, optional 0x2A) and §2.1.4 (speed command).
- Husky Robotics [CAN26 source](https://github.com/huskyroboticsteam/CAN26), `Packets/Motor.h`, `CANPacket.h`, `CANPacket.c`.
- Last year's uploaded `ODriveTranslation.h/.c` for architectural inspiration (not binary-compatible with CubeMars).
