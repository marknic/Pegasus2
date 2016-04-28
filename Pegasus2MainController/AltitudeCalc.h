#ifndef _ALTITUDE_CALC_H

#define _ALTITUDE_CALC_H
#include "AltitudePressureValues.h"


#define ALTITUDE_CALCE_IS_INITIALIZED   0xBEAD 

class AltitudeCalc
{
private:
    double UniversalGasConstant;
    double GravitationalAccelerationConstant;
    double MolarMass;
    double AccelerationByMolarMass;  // Grav x Molar Mass
    double StaticPressure; // [N/m^2] = [Pa]
    double StaticHeight; // [ft]
    double StaticTemp; // [K]
    double CgRGas; //(GravitationalAccelerationConstant * convertFtToMeters) / crGasSi;
    double ConvertFtToMeters;
    double StandardLapseRate;
    double ConvertToKelvinTemp;
    double StandardMslTemp;
    double StandardMslPressure;

    AltitudePressureValues* StaticValues[7];

    double _tempMsl;
    double _presMsl;
    
    int is_initialized;

public:
    AltitudeCalc();
    ~AltitudeCalc();

    double CalcAltitudeWithLapse(double pressure, int index);

    double CalcAltitudeWithoutLapse(double pressure);

    int CalculateAltitude(double pressure);

    void InitializeValues();
    
    void Initialize(double stationTemperature, double stationAltitude, double stationPressure);
    
    double CalculatePressureAtMsl(double stationPressure, double stationAltitude, double temperatureAtMsl);

    double CalculateTempAtMsl(double stationTemperature, double stationAltitude);

};


#endif

