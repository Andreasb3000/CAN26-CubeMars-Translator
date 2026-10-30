/* Software-only end-to-end test using the REAL CAN26 constructor and decoder. */
#include "CAN26CubeMars.h"
#include "CANDevices.h"
#include "Packets/Motor.h"

#include <assert.h>
#include <stdio.h>

static void show_frame(const CubeMarsFrame *frame) {
    printf("Outgoing CubeMars ID: 0x%03X; data:", (unsigned)frame->id);
    for (unsigned i = 0; i < frame->length; ++i) {
        printf(" %02X", (unsigned)frame->data[i]);
    }
    putchar('\n');
}

int main(void) {
    CANDevice_t jetson = {0};
    jetson.deviceUUID = CAN_UUID_JETSON;

    CANDevice_t elbow = {0};
    elbow.motorDomain = true;
    elbow.deviceUUID = CAN_UUID_BLDC_ELBOW;

    CubeMarsConfig config = {
        .controller_id = 1,          /* illustrative */
        .pole_pairs = 14,
        .gear_ratio = 39.0f,
        .input_is_joint_speed = false, /* actual semantics TBD */
        .max_abs_erpm = 40000        /* software-demo limit only */
    };

    CANPacket_t incoming =
        CANMotorPacket_BLDC_SetInputVelocity(jetson, elbow, 1.0f, 0.0f);

    printf("Incoming CAN26 payload:");
    const uint8_t *payload = CANGetDataConst(&incoming);
    for (unsigned i = 0; i < CANGetDlc(&incoming); ++i) {
        printf(" %02X", (unsigned)payload[i]);
    }
    putchar('\n');

    CubeMarsFrame outgoing = {0};
    assert(CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));
    assert(outgoing.id == 0x301 && outgoing.length == 4 && outgoing.extended);
    assert(outgoing.data[0] == 0x00 && outgoing.data[1] == 0x00);
    assert(outgoing.data[2] == 0x03 && outgoing.data[3] == 0x48);
    show_frame(&outgoing);

    /* Test negative velocity */

    incoming = CANMotorPacket_BLDC_SetInputVelocity(
        jetson, elbow, -1.0f, 0.0f
    );

    bool success = CAN26ToCubeMarsServoVelocity(
        &incoming, &config, &outgoing
    );

    printf("\nNegative velocity test:\n");
    printf("Translation successful: %s\n",
        success ? "yes" : "no");

    show_frame(&outgoing);

    assert(success);
    assert(outgoing.data[0] == 0xFF &&
        outgoing.data[1] == 0xFF);
    assert(outgoing.data[2] == 0xFC &&
        outgoing.data[3] == 0xB8);

    /* Reject CAN26 feedforward torque that Servo speed mode cannot encode. */
    incoming = CANMotorPacket_BLDC_SetInputVelocity(jetson, elbow, 1.0f, 1.0f);
    assert(!CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));
    incoming = CANMotorPacket_BLDC_SetInputVelocity(jetson, elbow, 1.0f, 0.0f);

    /* The CAN26 ACK-request bit may be set without changing command identity. */
    incoming.command |= 0x80u;
    assert(CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));

    /* Invalid packet checks. */
    incoming.contentsLength = 4;
    assert(!CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));
    incoming.contentsLength = 6;
    incoming.device.deviceUUID = CAN_UUID_JETSON;
    assert(!CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));
    incoming.device.deviceUUID = CAN_UUID_BLDC_ELBOW;
    incoming.command = CAN_COMMAND_ID__E_STOP;
    assert(!CAN26ToCubeMarsServoVelocity(&incoming, &config, &outgoing));

    puts("PASS: CAN26 -> CubeMars Servo software integration. No CAN transmission occurred.");
    return 0;
}
