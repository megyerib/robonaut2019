#include "statemachine.h"

void InitStateMachine(
	StateMachine* stm,
	uint8_t stateNum,
	StateFunction* states,
	uint8_t initialState)
{
	stm->stateNum = stateNum;
	stm->functions = states;
	stm->current = initialState;
	stm->prev = initialState;
	stm->prev_diff = initialState;
}

void RunStateMachine(StateMachine* stm)
{
	uint8_t current = stm->current;
}
