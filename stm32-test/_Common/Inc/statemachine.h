#pragma once
#include <stdint.h>

typedef uint16_t State;

typedef State (*StateFunction)(StateMachine stm);

typedef struct
{
	State current;
	State prev;
	State prev_diff;
	uint8_t stateNum;
	StateFunction* functions;
}
StateMachine;

void InitStateMachine(StateMachine* stm, uint8_t stateNum, StateFunction* states, uint8_t initialState);
void RunStateMAchine(StateMachine* stm);
