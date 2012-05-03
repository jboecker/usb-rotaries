#ifndef __encoder_h_included__
#define encoder_h_included

typedef unsigned char uint8_t;

// elements of the ENCODER EVENT bitmask
#define ECEV_NONE 0x00

#define ECEV_BUTTON_DOWN (1<<0)
#define ECEV_BUTTON_UP (1<<1)
#define ECEV_LEFT (1<<2)
#define ECEV_RIGHT (1<<3)

// encoder state
#define ECST_STATEMASK_BUTTONSTATE  0b00000001
#define ECST_STATEMASK_ENCODERSTATE 0b00000110

// encoder states are arbitrarily named NORTH, EAST, SOUTH and WEST
// signal "left rotation": states NORTH -> WEST -> SOUTH -> EAST -> NORTH
// signal "right rotation":states NORTH -> EAST -> SOUTH -> WEST -> NORTH
#define ECST_NORTH 0b00000000
#define ECST_EAST  0b00000010
#define ECST_SOUTH 0b00000110
#define ECST_WEST  0b00000100

#define ECST_DEFAULT_STATE ECST_NORTH


uint8_t encoder_events(uint8_t oldstate, uint8_t newstate);


#endif
