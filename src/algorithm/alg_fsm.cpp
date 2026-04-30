#include "wheeled_dog/algorithm/alg_fsm.hpp"
#include <iostream>

Class_FSM::Class_FSM() : currentState(DogState::PRONE) 
{
    // 初始化时默认为趴下状态
}

void Class_FSM::handleCommand(DogCommand cmd) 
{
    switch (currentState) 
    {
        case DogState::PRONE:
            if (cmd == DogCommand::CMD_STAND_UP) 
            {
                currentState = DogState::STAND_LOCKED;
            }
            break;

        case DogState::STAND_LOCKED:
            if (cmd == DogCommand::CMD_MOVE) 
            {
                currentState = DogState::MOVING;
            } 
            else if (cmd == DogCommand::CMD_LAY_DOWN) 
            {
                currentState = DogState::PRONE;
            }
            break;

        case DogState::MOVING:
            if (cmd == DogCommand::CMD_STOP) 
            {
                currentState = DogState::STAND_LOCKED;
            }
            break;
    }
}


DogState Class_FSM::getCurrentState() const 
{
    return currentState;
}

std::string Class_FSM::getStateString() const 
{
    switch (currentState) 
    {
        case DogState::PRONE: return "PRONE";
        case DogState::STAND_LOCKED: return "STAND_LOCKED";
        case DogState::MOVING: return "MOVING";
        default: return "UNKNOWN";
    }
}