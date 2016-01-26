#include "AltitudePressureValues.h"


AltitudePressureValues::AltitudePressureValues(double pressure, int height, double temperature, double lapseRate)
{
    Pressure = pressure;
    Height = height;
    Temperature = temperature;
    LapseRate = lapseRate;
}


AltitudePressureValues::~AltitudePressureValues()
{
}
