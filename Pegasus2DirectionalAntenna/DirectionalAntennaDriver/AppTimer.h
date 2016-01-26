// AppTimer.h

#ifndef _APPTIMER_h
#define _APPTIMER_h

#include <inttypes.h>
#include "Arduino.h"
#include "Event.h"

#define MAX_NUMBER_OF_EVENTS (4)

#define TIMER_NOT_AN_EVENT (-2)
#define NO_TIMER_AVAILABLE (-1)

class AppTimer
{

public:
    AppTimer(void);

    int8_t every(unsigned long period, void(*callback)(void));
    int8_t every(unsigned long period, void(*callback)(void), int repeatCount);
    int8_t after(unsigned long duration, void(*callback)(void));

    void stop(int8_t id);
    void update(void);
    void update(unsigned long now);

protected:
    Event _events[MAX_NUMBER_OF_EVENTS];
    int8_t findFreeEventIndex(void);

};

#endif

