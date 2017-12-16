#include "states.h"

FSM_INITIAL_STATE(AccessSystem, ReaderInitialize)

uint16_t WaitForCard::attempts = 0;
