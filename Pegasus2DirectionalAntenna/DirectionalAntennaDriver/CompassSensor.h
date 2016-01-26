// CompassSensor.h

#ifndef _COMPASSSENSOR_h
#define _COMPASSSENSOR_h

#include <Wire.h>
#include "arduino.h"


class CompassSensor
{
 private:
     float _heading;
     float _pitch;
     float _roll;

     float Accx;
     float Accy;
     float Accz;

     float Magx;
     float Magy;
     float Magz;


     // Calibration Numbers:
     float Mag_minx = -643;
     float Mag_miny = -737;
     float Mag_minz = -868;
     float Mag_maxx = 713;
     float Mag_maxy = 711;
     float Mag_maxz = 531;


 public:
    CompassSensor();

    float get_TiltHeading(void);
    void setMinMax(float minx, float miny, float minz, float maxx, float maxy, float maxz);
    void get_Magnetometer(void);
    void get_Accelerometer(void);
    void init_Compass(void);
    byte ReadMagRegister(byte regaddress);
    void WriteMagRegister(byte data, byte regaddress);
    byte ReadAccRegister(byte regaddress);
    void WriteAccRegister(byte data, byte regaddress);
    float get_heading();
    float get_Pitch();
};


#endif

