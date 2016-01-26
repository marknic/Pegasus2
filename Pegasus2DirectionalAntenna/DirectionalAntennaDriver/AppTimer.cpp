#include "AppTimer.h"

AppTimer::AppTimer(void)
{
}

int8_t AppTimer::every(unsigned long period, void(*callback)(), int repeatCount)
{
    int8_t i = findFreeEventIndex();
    if (i == -1) return -1;

    _events[i].eventType = EVENT_EVERY;
    _events[i].period = period;
    _events[i].repeatCount = repeatCount;
    _events[i].callback = callback;
    _events[i].lastEventTime = millis();
    _events[i].count = 0;
    return i;
}

int8_t AppTimer::every(unsigned long period, void(*callback)())
{
    return every(period, callback, -1); // - means forever
}

int8_t AppTimer::after(unsigned long period, void(*callback)())
{
    return every(period, callback, 1);
}


void AppTimer::stop(int8_t id)
{
    if (id >= 0 && id < MAX_NUMBER_OF_EVENTS) {
        _events[id].eventType = EVENT_NONE;
    }
}

void AppTimer::update(void)
{
    unsigned long now = millis();
    update(now);
}

void AppTimer::update(unsigned long now)
{
    for (int8_t i = 0; i < MAX_NUMBER_OF_EVENTS; i++)
    {
        if (_events[i].eventType != EVENT_NONE)
        {
            _events[i].update(now);
        }
    }
}

int8_t AppTimer::findFreeEventIndex(void)
{
    for (int8_t i = 0; i < MAX_NUMBER_OF_EVENTS; i++)
    {
        if (_events[i].eventType == EVENT_NONE)
        {
            return i;
        }
    }
    return NO_TIMER_AVAILABLE;
}
