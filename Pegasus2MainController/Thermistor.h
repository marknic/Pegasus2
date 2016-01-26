//
#ifndef THERMISTOR_H

#include "math.h"

#define THERMISTOR_H

// Degrees Kelvin to Celcius Converter Value:  Tc = Tk - 273.15  
#define DEGREES_KELVIN_CONVERTER            273.15
#define ANALOG_VALUE_MAX                    1023.0
#define RESISTANCE_10K                     10000.0
#define STANDARD_VOLTAGE                       5.0

// Coefficients in the Steinhart-Hart equation
#define SH_THERMISTER_COEF1           1.129240e-03
#define SH_THERMISTER_COEF2           2.341080e-04
#define SH_THERMISTER_COEF3           8.775500e-08


class Thermistor {

public:
    Thermistor(float fixedResistorValue);
    Thermistor(float fixedResistorValue, float referenceVoltage);

    float getResistance(float voltageLevel);
    float getTemperature(float voltageLevel);
    float getAdjustedTemperature(float voltageLevel);

    void setFixedResistance(float fixedResistorValue);
    void setVoltage(float voltage);

private:
    float _fixedResistorValue;
    float _actualVoltage;
    float _voltageRatio;

};

#endif