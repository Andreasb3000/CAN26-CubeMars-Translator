# Portable C tests for the CAN26 <-> CubeMars V3 Servo translation library.
# Clone https://github.com/huskyroboticsteam/CAN26 alongside this directory.
CC = cc
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -Wno-error=strict-prototypes
CAN26_DIR ?= ../CAN26
LDLIBS = -lm

.PHONY: test test-can26 all clean
all: test_core test_feedback_core

test_core: CubeMarsServo.c CubeMarsServo.h test_core.c
	$(CC) $(CFLAGS) CubeMarsServo.c test_core.c $(LDLIBS) -o $@

test_feedback_core: CubeMarsFeedback.c CubeMarsFeedback.h CubeMarsServo.h test_feedback_core.c
	$(CC) $(CFLAGS) CubeMarsFeedback.c test_feedback_core.c $(LDLIBS) -o $@

test_can26: CubeMarsServo.c CAN26CubeMars.c CAN26CubeMars.h CubeMarsServo.h test_can26.c
	$(CC) $(CFLAGS) -I$(CAN26_DIR) CubeMarsServo.c CAN26CubeMars.c test_can26.c $(CAN26_DIR)/CANPacket.c $(LDLIBS) -o $@

test_feedback_can26: CubeMarsFeedback.c CAN26CubeMarsFeedback.c CubeMarsFeedback.h CAN26CubeMarsFeedback.h test_feedback_can26.c
	$(CC) $(CFLAGS) -I$(CAN26_DIR) CubeMarsFeedback.c CAN26CubeMarsFeedback.c test_feedback_can26.c $(CAN26_DIR)/CANPacket.c $(LDLIBS) -o $@

test: test_core test_feedback_core
	./test_core
	./test_feedback_core

test-can26: test_can26 test_feedback_can26
	./test_can26
	./test_feedback_can26

clean:
	rm -f test_core test_feedback_core test_can26 test_feedback_can26 *.o
