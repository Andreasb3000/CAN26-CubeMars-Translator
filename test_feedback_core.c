/* Test the CubeMars 0x29 decoder and unit conversion WITHOUT CAN26. */
#include "CubeMarsFeedback.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static CubeMarsFrame example_feedback(void) {
    /* Position +90 deg (900 counts), speed +1000 ERPM (100 counts),
       Iq +1.25 A (125 counts), temperature +35 C, normal operation. */
    CubeMarsFrame frame = {
        .id = 0x2901u,
        .data = {0x03,0x84, 0x00,0x64, 0x00,0x7D, 0x23,0x00},
        .length = 8u,
        .extended = true
    };
    return frame;
}

static bool near(float x, float y, float tolerance) {
    return fabsf(x-y) < tolerance;
}

int main(void) {
    CubeMarsFrame frame = example_feedback();
    CubeMarsServoFeedback decoded = {0};
    assert(CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    assert(near(decoded.position_degrees, 90.0f, 0.001f));
    assert(near(decoded.speed_erpm, 1000.0f, 0.001f));
    assert(near(decoded.iq_amps, 1.25f, 0.001f));
    assert(decoded.temperature_c == 35);
    assert(decoded.status_code == 0);

    CubeMarsConfig config = {
        .controller_id = 1u,
        .pole_pairs = AK60_39_POLE_PAIRS,
        .gear_ratio = AK60_39_GEAR_RATIO,
        .input_is_joint_speed = false,
        .max_abs_erpm = 40000
    };
    float pos = 0.0f, vel = 0.0f;
    assert(CubeMarsFeedbackToMechanical(&decoded, &config, true, &pos, &vel));
    assert(near(pos, 0.25f, 0.00001f));
    assert(near(vel, 1000.0f / (14.0f * 60.0f), 0.00001f));
    printf("Positive status: position=%.4f rev, velocity=%.6f motor rev/s, Iq=%.2f A, temp=%d C\n",
           pos, vel, decoded.iq_amps, decoded.temperature_c);

    config.input_is_joint_speed = true;
    assert(CubeMarsFeedbackToMechanical(&decoded, &config, true, &pos, &vel));
    assert(near(pos, 0.25f / 39.0f, 0.00001f));
    assert(near(vel, 1000.0f / (14.0f * 60.0f * 39.0f), 0.00001f));
    printf("Joint-reference TEST: position=%.6f rev, velocity=%.6f rev/s\n", pos, vel);

    /* The motor controller's position reference is NOT settled.
       If its angle is already measured at the output, do not divide position. */
    assert(CubeMarsFeedbackToMechanical(&decoded, &config, false, &pos, &vel));
    assert(near(pos, 0.25f, 0.00001f));

    frame.data[0] = 0xFC; frame.data[1] = 0x7C; /* -900 counts = -90 degrees */
    frame.data[2] = 0xFF; frame.data[3] = 0x9C; /* -100 counts = -1000 ERPM */
    frame.data[4] = 0xFF; frame.data[5] = 0x83; /* -125 counts = -1.25 A */
    frame.data[6] = 0xF6; /* -10 C */
    assert(CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    assert(near(decoded.position_degrees, -90.0f, 0.001f));
    assert(near(decoded.speed_erpm, -1000.0f, 0.001f));
    assert(near(decoded.iq_amps, -1.25f, 0.001f));
    assert(decoded.temperature_c == -10);

    /* Malformed messages must not modify the decoded output. */
    decoded.status_code = 0x5Au;
    frame.extended = false;
    assert(!CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    assert(decoded.status_code == 0x5Au);
    frame.extended = true;
    frame.length = 4u;
    assert(!CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    frame.length = 8u;
    frame.id = 0x2902u;
    assert(!CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    frame.id = 0x2801u;
    assert(!CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    frame.id = 0x2901u;

    /* Nonzero driver status and special 0x77 disable confirmation
       can be decoded, but must NOT be treated as healthy encoder data. */
    frame.data[7] = 0x02u;
    assert(CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    assert(!CubeMarsFeedbackToMechanical(&decoded, &config, true, &pos, &vel));
    frame.data[7] = 0x77u;
    assert(CubeMarsDecodeServoFeedback(&frame, 1u, &decoded));
    assert(decoded.status_code == 0x77u);
    assert(!CubeMarsFeedbackToMechanical(&decoded, &config, true, &pos, &vel));

    puts("PASS: CubeMars feedback decoding and conversion. No CAN transmitted.");
    return 0;
}
