#include <Arduino.h>
#include <EEPROM.h>
#include "AzimuthElevation.h"


#ifndef TRUE
#define TRUE    (1==1)
#endif

#ifndef FALSE
#define FALSE   (1==2)
#endif

#define LAT_LON_EEPROM_ADDRESS                              0
#define GBANG_DO_OFFSET_ADDRESS                            12
#define GBANG_OFFSET_DIRECTION_ADDRESS                     13

#define GPS_OFFSET_N                                        0
#define GPS_OFFSET_NE                                       1
#define GPS_OFFSET_E                                        2
#define GPS_OFFSET_SE                                       3
#define GPS_OFFSET_S                                        4
#define GPS_OFFSET_SW                                       5
#define GPS_OFFSET_W                                        6
#define GPS_OFFSET_NW                                       7


latLonAlt _launchStationPosition;

uint8_t _doOffset;
uint8_t _launchOffsetDirection;

void setup()
{
    Serial.begin(115200);

    _doOffset = TRUE;

    _launchOffsetDirection = GPS_OFFSET_SE  ;
    
    _launchStationPosition.lat = 0.0;
    _launchStationPosition.lon = 0.0;
    _launchStationPosition.alt = 0.0;

    EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
    EEPROM.put(GBANG_DO_OFFSET_ADDRESS, _doOffset);
    EEPROM.put(GBANG_OFFSET_DIRECTION_ADDRESS, 0xff);

    EEPROM.get(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
    EEPROM.get(GBANG_DO_OFFSET_ADDRESS, _doOffset);
    EEPROM.get(GBANG_OFFSET_DIRECTION_ADDRESS, _launchOffsetDirection);

    Serial.print("_launchStationPosition: "); 
    Serial.print(_launchStationPosition.lat); Serial.print(", ");
    Serial.print(_launchStationPosition.lon); Serial.print(" @ ");
    Serial.print(_launchStationPosition.alt); Serial.print(" meters");
    Serial.println();

    Serial.print("_doOffset: ");
    Serial.print(_doOffset);
    Serial.println();

    Serial.print("_launchOffsetDirection: ");
    Serial.print(_launchOffsetDirection);
    Serial.println();

}

void loop()
{

  /* add main program code here */

}
