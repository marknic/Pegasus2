#include "AltitudeCalc.h"


AltitudeCalc::AltitudeCalc()
{
    Initialize();
}


AltitudeCalc::~AltitudeCalc()
{
}

void AltitudeCalc::Initialize()
{
    UniversalGasConstant = 8.31432;
    GravitationalAccelerationConstant = 9.80665;
    MolarMass = 0.0289644;
    AccelerationByMolarMass = 0.28404373326;  // Grav x Molar Mass
    StaticPressure = 22632.22842714; // [N/m^2] = [Pa]
    StaticHeight = 36089.2388451444; // [ft]
    StaticTemp = 216.65; // [K]
    CgRGas = 0.01041294436915830874437821586954; //(GravitationalAccelerationConstant * convertFtToMeters) / crGasSi;
    ConvertFtToMeters = 0.3048;


    StaticValues[0] = new AltitudePressureValues(101325.00, 0, 288.15, -0.0065);
    StaticValues[1] = new AltitudePressureValues(22632.10, 11000, 288.15, 0.0);
    StaticValues[2] = new AltitudePressureValues(5474.89, 20000, 216.65, 0.001);
    StaticValues[3] = new AltitudePressureValues(868.02, 32000, 228.65, 0.0028);
    StaticValues[4] = new AltitudePressureValues(110.91, 47000, 270.65, 0.0);
    StaticValues[5] = new AltitudePressureValues(66.94, 51000, 270.65, -0.0028);
    StaticValues[6] = new AltitudePressureValues(3.96, 71000, 214.65, -0.002);

    is_initialized = ALTITUDE_CALCE_IS_INITIALIZED;
}


double AltitudeCalc::CalcAltitudeWithLapse(double pressure, int index)
{
    if (is_initialized != ALTITUDE_CALCE_IS_INITIALIZED) {
        Initialize();
    }

    double staticTempLapse = StaticValues[index]->Temperature / StaticValues[index]->LapseRate;

    double ratioOfStatic = (pressure / StaticValues[index]->Pressure);

    double gasByLapseRate = -UniversalGasConstant * StaticValues[index]->LapseRate;

    double ratioRaisedToPower = pow(ratioOfStatic,
        (gasByLapseRate / AccelerationByMolarMass));

    double h = StaticValues[index]->Height +
        (staticTempLapse) *
        (ratioRaisedToPower - 1);

    return h;
}


double AltitudeCalc::CalcAltitudeWithoutLapse(double pressure)
{

    double h = StaticHeight - log(pressure / StaticPressure) * StaticTemp / CgRGas;

    h = h * ConvertFtToMeters;

    return h;
}

int AltitudeCalc::CalculateAltitude(double pressure)
{
    int index;
    int altitude;

    pressure = pressure * 100.0;

    if (pressure > 22632.1) index = 0;
    else if (pressure > 5474.89) index = 1;
    else if (pressure > 868.02) index = 2;
    else if (pressure > 110.91) index = 3;
    else if (pressure > 66.94) index = 4;
    else if (pressure > 3.96) index = 5;
    else index = 6;

    if ((index == 1) || (index == 4))
    {
        altitude = (int) CalcAltitudeWithoutLapse(pressure);
    }
    else
    {
        altitude = (int) CalcAltitudeWithLapse(pressure, index);
    }

    return altitude;
}

