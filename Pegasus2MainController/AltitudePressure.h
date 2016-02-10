#pragma once

#define ALT_PRESS_ARRAY_COUNT  8721

class AltitudePressure
{
private:
    int _altitude[ALT_PRESS_ARRAY_COUNT];
    double _pressure[ALT_PRESS_ARRAY_COUNT];
    int _totalCount;
    int _currentPointer;
    int _currentAltitude;
    
public:
    AltitudePressure();
    ~AltitudePressure();
    
    void addValues(int altitude, double pressure);
    double getPressure(int altitude);
};

