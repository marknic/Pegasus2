#include "CompassSensor.h"

CompassSensor::CompassSensor()
{
}


void CompassSensor::init_Compass(void)
{
    WriteAccRegister(0x67, 0x20);  // Enable accelerometer, 200Hz data output

    WriteMagRegister(0x9c, 0x00);  // Enable temperature sensor, 220Hz data output
    WriteMagRegister(0x20, 0x01);  // set gain to +/-1.3Gauss
    WriteMagRegister(0x00, 0x02);  // Enable magnetometer constant conversions
}

/*
Send register address and the byte value you want to write the accelerometer and
loads the destination register with the value you send
*/
void CompassSensor::WriteAccRegister(byte data, byte regaddress)
{
    Wire.beginTransmission(0x19);   // Use accelerometer address for regs >=0x20
    Wire.write(regaddress);
    Wire.write(data);
    Wire.endTransmission();
}

/*
Send register address to this function and it returns byte value
for the accelerometer register's contents
*/
byte CompassSensor::ReadAccRegister(byte regaddress)
{
    byte data;
    Wire.beginTransmission(0x19);   // Use accelerometer address for regs >=0x20  
    Wire.write(regaddress);
    Wire.endTransmission();

    delayMicroseconds(100);

    Wire.requestFrom(0x19, 1);   // Use accelerometer address for regs >=0x20
    data = Wire.read();
    Wire.endTransmission();

    delayMicroseconds(100);

    return data;
}

/*
Send register address and the byte value you want to write the magnetometer and
loads the destination register with the value you send
*/
void CompassSensor::WriteMagRegister(byte data, byte regaddress)
{
    Wire.beginTransmission(0x1E);   // Else use magnetometer address
    Wire.write(regaddress);
    Wire.write(data);
    Wire.endTransmission();

    delayMicroseconds(100);
}

/*
Send register address to this function and it returns byte value
for the magnetometer register's contents
*/
byte CompassSensor::ReadMagRegister(byte regaddress)
{
    byte data;
    Wire.beginTransmission(0x1E);   // Else use magnetometer address  
    Wire.write(regaddress);
    Wire.endTransmission();

    delayMicroseconds(100);

    Wire.requestFrom(0x1E, 1);   // Else use magnetometer address
    data = Wire.read();
    Wire.endTransmission();

    delayMicroseconds(100);

    return data;
}


/*
Readsthe X,Y,Z axis values from the accelerometer and sends the values to the
serial monitor.
*/
void CompassSensor::get_Accelerometer(void)
{
    // accelerometer values
    byte xh = ReadAccRegister(0x29);
    byte xl = ReadAccRegister(0x28);
    byte yh = ReadAccRegister(0x2B);
    byte yl = ReadAccRegister(0x2A);
    byte zh = ReadAccRegister(0x2D);
    byte zl = ReadAccRegister(0x2C);

    // need to convert the register contents into a righ-justified 16 bit value
    Accx = (xh << 8 | xl);
    Accy = (yh << 8 | yl);
    Accz = (zh << 8 | zl);
}

/*
Reads the X,Y,Z axis values from the magnetometer sends the values to the
serial monitor.
*/
void CompassSensor::get_Magnetometer(void)
{
    // magnetometer values
    byte xh = ReadMagRegister(0x03);
    byte xl = ReadMagRegister(0x04);
    byte yh = ReadMagRegister(0x07);
    byte yl = ReadMagRegister(0x08);
    byte zh = ReadMagRegister(0x05);
    byte zl = ReadMagRegister(0x06);

    // convert registers to ints
    Magx = (xh << 8 | xl);
    Magy = (yh << 8 | yl);
    Magz = (zh << 8 | zl);
}


void CompassSensor::setMinMax(float minx, float miny, float minz, float maxx, float maxy, float maxz) {

    Mag_minx = minx;
    Mag_miny = miny;
    Mag_minz = minz;
    Mag_maxx = maxx;
    Mag_maxy = maxy;
    Mag_maxz = maxz;
}


/*
Converts values to a tilt compensated heading in degrees (0 to 360)
*/
float CompassSensor::get_TiltHeading(void)
{
    // use calibration values to shift and scale magnetometer measurements
    Magx = (Magx - Mag_minx) / (Mag_maxx - Mag_minx) * 2 - 1;
    Magy = (Magy - Mag_miny) / (Mag_maxy - Mag_miny) * 2 - 1;
    Magz = (Magz - Mag_minz) / (Mag_maxz - Mag_minz) * 2 - 1;

    // Normalize acceleration measurements so they range from 0 to 1
    float accxnorm = Accx / sqrt(Accx*Accx + Accy*Accy + Accz*Accz);
    float accynorm = Accy / sqrt(Accx*Accx + Accy*Accy + Accz*Accz);

    // calculate _pitch and roll
    _pitch = asin(-accxnorm);
    _roll = asin(accynorm / cos(_pitch));

    // tilt compensated magnetic sensor measurements
    float magxcomp = Magx*cos(_pitch) + Magz*sin(_pitch);
    float magycomp = Magx*sin(_roll) * sin(_pitch) + 
        Magy*cos(_roll) - Magz*sin(_roll) * cos(_pitch);

    // arctangent of y/x converted to degrees
    _heading = 180 * atan2(magycomp, magxcomp) / PI;

    if (_heading < 0)
        _heading += 360;

    return _heading;
}


float CompassSensor::get_heading() {
    
    get_Accelerometer();
    get_Magnetometer();

    float tilt_heading = get_TiltHeading();

    return tilt_heading;
}

float CompassSensor::get_Pitch()
{
    const float pitch_alpha = 0.5;

    //Low Pass Filter
    float fXg = Accx * pitch_alpha + (fXg * (1.0 - pitch_alpha));
    float fYg = Accy * pitch_alpha + (fYg * (1.0 - pitch_alpha));
    float fZg = Accz * pitch_alpha + (fZg * (1.0 - pitch_alpha));

    return (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0) / M_PI;
}

