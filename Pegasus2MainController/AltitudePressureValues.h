#pragma once
class AltitudePressureValues
{
public:
    double Pressure;
    int Height;
    double Temperature;
    double LapseRate;

    AltitudePressureValues(double pressure, int height, double temperature, double lapseRate);
    ~AltitudePressureValues();
};

