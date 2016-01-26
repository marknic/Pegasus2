
#include "Event.h"
#include <stddef.h>
#include <sys/time.h>


Event::Event(void)
{
    eventType = EVENT_NONE;
}

long long Event::get_milliseconds() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time

    long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // caculate milliseconds
    return milliseconds;
}

void Event::update(void)
{
    long long now = get_milliseconds();

    update(now);
}

void Event::update(long long now)
{
    if (now - lastEventTime >= period)
    {
        switch (eventType)
        {
            case EVENT_EVERY:
                (*callback)();
                break;

            //case EVENT_OSCILLATE:
            //    pinState = ! pinState;
            //    digitalWrite(pin, pinState);
            //    break;
        }
        lastEventTime = now;
        count++;
    }
    if (repeatCount > -1 && count >= repeatCount)
    {
        eventType = EVENT_NONE;
    }
}
