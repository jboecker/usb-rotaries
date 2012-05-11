#include "encoder.h"

uint8_t encoder_events(uint8_t oldstate, uint8_t newstate) {
   	uint8_t retevent = 0x00;
	
	// remember: 0 -> button pressed (tied to GND), 1 -> button not pressed
	if (((oldstate & ECST_STATEMASK_BUTTONSTATE) > 0) && ((newstate & ECST_STATEMASK_BUTTONSTATE) == 0))
		retevent |= ECEV_BUTTON_DOWN;
	if (((oldstate & ECST_STATEMASK_BUTTONSTATE) == 0) && ((newstate & ECST_STATEMASK_BUTTONSTATE) > 0))
		retevent |= ECEV_BUTTON_UP;

	if (((oldstate & ECST_STATEMASK_ENCODERSTATE) == ECST_EAST) &&
		((newstate & ECST_STATEMASK_ENCODERSTATE) == ECST_SOUTH))
		retevent |= ECEV_LEFT;
	
	if (((oldstate & ECST_STATEMASK_ENCODERSTATE) == ECST_WEST) &&
		((newstate & ECST_STATEMASK_ENCODERSTATE) == ECST_SOUTH))
		retevent |= ECEV_RIGHT;

	return retevent;
}
