#include "Thermistor.h"

// ctor
Thermistor::Thermistor(float fixedResistorValue) {

    _fixedResistorValue = fixedResistorValue; // Fixed resistance

    _actualVoltage = STANDARD_VOLTAGE;

    _voltageRatio = 1.0;

}

Thermistor::Thermistor(float fixedResistorValue, float referenceVoltage) {

    _fixedResistorValue = fixedResistorValue; // Fixed resistance

    _actualVoltage = referenceVoltage;

    _voltageRatio = _actualVoltage / STANDARD_VOLTAGE;

}


// Calculate Thermistor Resistance
float Thermistor::getResistance(float voltageLevel) {

    float thermistorResistance =
        _fixedResistorValue * ((ANALOG_VALUE_MAX / voltageLevel) - 1.0);

    return thermistorResistance;
}// Calculate Thermistor Temperature Given Temperature
float Thermistor::getTemperature(float voltageLevel) {

    float thermistorResistance = getResistance(voltageLevel);

    float logOfRt = log(thermistorResistance);

    float temp = (1.0 / (SH_THERMISTER_COEF1 + (SH_THERMISTER_COEF2 * logOfRt) +
        (SH_THERMISTER_COEF3 * pow(logOfRt, 3)))) - DEGREES_KELVIN_CONVERTER;

    return temp;
}// Calculate Thermistor Temperature Given Temperature
float Thermistor::getAdjustedTemperature(float voltageLevel) {

    float adjustedVoltageLevel = voltageLevel * _voltageRatio;

    float thermistorResistance = getResistance(adjustedVoltageLevel);

    float logOfRt = log(thermistorResistance);

    float temp = (1.0 / (SH_THERMISTER_COEF1 + (SH_THERMISTER_COEF2 * logOfRt) +
        (SH_THERMISTER_COEF3 * pow(logOfRt, 3)))) - DEGREES_KELVIN_CONVERTER;

    return temp;
}// Set the fixed resistance property
void Thermistor::setFixedResistance(float fixedResistance) {
    _fixedResistorValue = fixedResistance;
}

// Set the fixed resistance property
void Thermistor::setVoltage(float voltage) {
    _actualVoltage = voltage;

    _voltageRatio = _actualVoltage / STANDARD_VOLTAGE;
}



