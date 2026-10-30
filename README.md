# Husky Robotics — CAN26 to CubeMars V3 Servo Velocity 

This is a **software-only** proof of the velocity-WRITE path:

1. Validate and decode CAN26 `BLDC_INPUT_VELOCITY` (existing Husky Robotics CAN26 library).
2. Convert rev/s to ERPM using configurable motor-shaft vs output-shaft semantics.
3. Encode an extended 29-bit CAN frame in CubeMars V3 Servo **speed mode 3**.

Nothing is sent to a motor. This is **not** production safety firmware.


## 1. Portable encoding/conversion tests

In a VS Code terminal in this folder:

```bash
cc -std=c11 -Wall -Wextra CubeMarsServo.c test_core.c -lm -o test_core
./test_core
```

## 2. CAN26 integrated test

If CAN26 is cloned in the sibling folder shown above:

```bash
cc -std=c11 -Wall -Wextra -I../CAN26 \
  CubeMarsServo.c CAN26CubeMars.c test_can26.c ../CAN26/CANPacket.c \
  -lm -o test_can26
./test_can26
```

Expected output for the first CAN26 translation:

```
Incoming CAN26 payload: 09 01 00 00 80 3F 00 00
Outgoing CubeMars ID: 0x301; data: 00 00 03 48
PASS: CAN26 -> CubeMars Servo software integration. No CAN transmission occurred.
```

This test uses motor-shaft semantics: 1 rev/s * 60 sec/min * 14 pole pairs = **840 ERPM** = `0x00000348`.
If your CAN26 rev/s instead means output-shaft speed, set `input_is_joint_speed=true` and the same request yields **32760 ERPM** (`00 00 7F F8`) due to the 39:1 gearbox.

