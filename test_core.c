#include "CubeMarsServo.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void print_frame(const CubeMarsFrame *frame) {
    printf("CAN ID: 0x%03X | Extended: %s | DLC: %u | Data:",
           (unsigned)frame->id, frame->extended ? "yes" : "no",
           (unsigned)frame->length);
    for (unsigned i = 0; i < frame->length; ++i) {
        printf(" %02X", (unsigned)frame->data[i]);
    }
    putchar('\n');
}

int main(void) {
    CubeMarsConfig config = {
        .controller_id = 1, /* example, not approved hardware ID */
        .pole_pairs = AK60_39_POLE_PAIRS,
        .gear_ratio = AK60_39_GEAR_RATIO,
        .input_is_joint_speed = false, /* start with motor-shaft interpretation */
        .max_abs_erpm = 40000 /* demo limit, NOT a joint safety limit */
    };
    CubeMarsFrame frame = {0};

    /* Motor-shaft speed 1 rev/s -> 60 mechanical RPM -> 840 ERPM. */
    assert(CubeMarsTranslateServoVelocity(1.0f, 0.0f, &config, &frame));
    assert(frame.id == 0x301 && frame.extended && frame.length == 4);
    assert(frame.data[0] == 0x00 && frame.data[1] == 0x00);
    assert(frame.data[2] == 0x03 && frame.data[3] == 0x48);
    puts("Motor-shaft 1.0 rev/s:");
    print_frame(&frame);

    /* If CAN26 represents joint-output speed, the gearbox ratio is included. */
    config.input_is_joint_speed = true;
    assert(CubeMarsTranslateServoVelocity(1.0f, 0.0f, &config, &frame));
    assert(frame.data[0] == 0x00 && frame.data[1] == 0x00);
    assert(frame.data[2] == 0x7F && frame.data[3] == 0xF8);
    puts("Joint-output 1.0 rev/s (TEST assumption):");
    print_frame(&frame);

    assert(CubeMarsTranslateServoVelocity(-1.0f, 0.0f, &config, &frame));
    assert(frame.data[0] == 0xFF && frame.data[1] == 0xFF);
    assert(frame.data[2] == 0x80 && frame.data[3] == 0x08);

    assert(CubeMarsTranslateServoVelocity(0.0f, 0.0f, &config, &frame));
    for (unsigned i = 0; i < 4u; ++i) assert(frame.data[i] == 0);

    /* No write to out_frame when input is invalid. */
    CubeMarsFrame unchanged = frame;
    assert(!CubeMarsTranslateServoVelocity(1.0f, 0.5f, &config, &frame));
    assert(frame.id == unchanged.id && frame.data[0] == unchanged.data[0]);
    assert(!CubeMarsTranslateServoVelocity(NAN, 0.0f, &config, &frame));
    assert(!CubeMarsTranslateServoVelocity(2.0f, 0.0f, &config, &frame));
    config.pole_pairs = 0;
    assert(!CubeMarsTranslateServoVelocity(1.0f, 0.0f, &config, &frame));

    /* Verify against CubeMars V3 manual: 5000 ERPM -> 00 00 13 88. */
    assert(CubeMarsMakeServoVelocityFrame(0x68, 5000, &frame));
    assert(frame.id == 0x368);
    assert(frame.data[0] == 0x00 && frame.data[1] == 0x00);
    assert(frame.data[2] == 0x13 && frame.data[3] == 0x88);
    puts("CubeMars V3 manual test vector (5000 ERPM):");
    print_frame(&frame);

    puts("PASS: all portable Servo-mode tests. No CAN transmission occurred.");
    return 0;
}
