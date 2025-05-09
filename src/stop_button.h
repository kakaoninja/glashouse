#ifndef STOP_BUTTON_H
#define STOP_BUTTON_H

#include <Arduino.h>

class Stop {
public:
    static void initialize();
    static bool isStopped_forward();
    static bool isStopped_backward();
    static void resetStop();

private:
    static volatile bool stopped_forward;
    static volatile bool stopped_backward;
    static void stopISR1();
    static void stopISR2();
};

#endif