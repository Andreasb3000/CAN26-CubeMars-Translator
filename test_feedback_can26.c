/* Software integration test: CubeMars 0x29 feedback -> CAN26 encoder estimates.
 * Uses the real CAN26 library from sibling directory ../CAN26 on your Mac. */
#include "CAN26CubeMarsFeedback.h"
#include "CANCommandIDs.h"
#include "CANDevices.h"
#include "Packets/Motor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static CubeMarsFrame make_motor_feedback(void) {
    CubeMarsFrame result = {
        .id = 0x2901u,
        .data = {0x03,0x84, 0x00,0x64, 0x00,0x7D, 0x23,0x00},
        .length = 8u,
        .extended = true
    };
    return result;
}

static bool near(float x, float y, float tolerance) {
    return fabsf(x - y) < tolerance;
}

static void print_can26(const CANPacket_t *packet) {
    printf("CAN26 ID 0x%03X (%u data bytes):",
           (unsigned)CANGetPacketHeader(packet), (unsigned)CANGetDlc(packet));
    for (uint8_t i = 0u; i < CANGetDlc(packet); ++i) {
        printf(" %02X", (unsigned)CANGetDataConst(packet)[i]);
    }
    putchar('\n');
}

int main(void) {
    CubeMarsConfig config = {
        .controller_id = 1u,
        .pole_pairs = AK60_39_POLE_PAIRS,
        .gear_ratio = AK60_39_GEAR_RATIO,
        .input_is_joint_speed = false,
        .max_abs_erpm = 40000
    };
    CANDevice_t jetson = {0};
    jetson.deviceUUID = CAN_UUID_JETSON;
    CANDevice_t elbow = {0};
    elbow.deviceUUID = CAN_UUID_BLDC_ELBOW;
    elbow.motorDomain = true;

    CubeMarsFrame feedback = make_motor_feedback();
    CANPacket_t reply = {0};
    assert(CubeMarsToCAN26EncoderEstimates(&feedback, &config, true,
                                           jetson, &reply));
    assert(reply.command == CAN_COMMAND_ID__BLDC_ENCODER_ESTIMATE);
    assert(reply.senderUUID == CAN_UUID_BLDC_ELBOW);
    assert(reply.device.deviceUUID == CAN_UUID_JETSON);
    assert(CANGetDlc(&reply) == 8u);
    assert(near(CANLoadBFloat24(reply.contents + 0), 0.25f, 0.0001f));
    assert(near(CANLoadBFloat24(reply.contents + 3),
                1000.0f/(14.0f*60.0f), 0.001f));
    print_can26(&reply);

    /* CAN26 GetEncoderEstimates asks the elbow; the sender receives the reply. */
    CANPacket_t request = CANMotorPacket_BLDC_GetEncoderEstimates(jetson, elbow, 0u);
    assert(CAN26ReplyToEncoderEstimateRequest(&request, &feedback, true,
                                               &config, true, &reply));
    assert(reply.senderUUID == CAN_UUID_BLDC_ELBOW);
    assert(reply.device.deviceUUID == CAN_UUID_JETSON);

    /* A stale cached frame must not produce a successful reply. */
    reply.command = 0xA5u;
    assert(!CAN26ReplyToEncoderEstimateRequest(&request, &feedback, false,
                                                &config, true, &reply));
    assert(reply.command == 0xA5u);
    request.contents[0] = 1u; /* Only one encoder (index 0) supported here */
    assert(!CAN26ReplyToEncoderEstimateRequest(&request, &feedback, true,
                                                &config, true, &reply));
    request.contents[0] = 0u;
    request.device.deviceUUID = CAN_UUID_BLDC_SHOULDER;
    assert(!CAN26ReplyToEncoderEstimateRequest(&request, &feedback, true,
                                                &config, true, &reply));

    /* A motor fault must not be represented as normal encoder feedback. */
    feedback.data[7] = 0x02u;
    assert(!CubeMarsToCAN26EncoderEstimates(&feedback, &config, true,
                                             jetson, &reply));
    feedback.data[7] = 0u;
    feedback.length = 7u;
    assert(!CubeMarsToCAN26EncoderEstimates(&feedback, &config, true,
                                             jetson, &reply));

    puts("PASS: CubeMars feedback -> CAN26 software integration. No CAN transmitted.");
    return 0;
}
