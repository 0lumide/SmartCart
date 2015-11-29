#include "IR.h"

IR::IR(uint8_t pin, int distanceLimit, long milliDelay){
    _pin = pin;
    pinMode(_pin, INPUT);
    _limit = distanceLimit;
    _delay = milliDelay;
}
bool IR::isBlocked(){
    if(readIR() >= _limit){
        _lastDetectedTime = millis();
        return 1;
    }else if((millis() - _lastDetectedTime) <= _delay)
        return 1;
    return 0;
}

int IR::readIR(){
    return analogRead(_pin);
}
