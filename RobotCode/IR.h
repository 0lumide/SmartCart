#ifndef IR_H
#define IR_H

#include <Arduino.h>

class IR
{
    public:
        IR(uint8_t pin, int distanceLimit, long milliDelay);
        bool isBlocked();
    private:
        long _delay;
        uint8_t _pin;
        long _lastDetectedTime;
        int _limit;
        int readIR();
};
#endif
