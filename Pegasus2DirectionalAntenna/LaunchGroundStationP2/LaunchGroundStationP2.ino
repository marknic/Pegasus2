//#include <Wire.h>
#include <PegasusCommandProcessor/PegasusCommandProcessor.h>
#include "AzimuthElevation.h"
#include "MessageValidation.h"
#include <Timer.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <Arduino.h>


#ifndef TRUE
#define TRUE    (1==1)
#endif

#ifndef FALSE
#define FALSE   (1==2)
#endif

#define RADIO_SERIAL_BAUD_RATE                          38400
#define DIRECTIONAL_ANTENNA_SERIAL_BAUD_RATE            38400
#define FIELD_GATEWAY_SERIAL_BAUD_RATE                  38400
#define DEBUG_CONSOLE_SERIAL_BAUD_RATE                  38400


#define RADIO_MSG_MAX_SIZE                                256
#define RADIO_MSG_MAX_OFFSET         (RADIO_MSG_MAX_SIZE - 1)
#define RADIO_MSG_OFFSET_INIT                              -1

#define AZ_EL_MAX_SIZE                                     16
#define AZ_EL_MAX_OFFSET                 (AZ_EL_MAX_SIZE - 1)
#define AZ_EL_INDICATOR_CHAR                              '|'
#define AZ_EL_OFFSET_INIT                                  -1

#define POS_LAT                                            23
#define POS_LON                                            24
#define POS_ALT                                             2

#define TELEMETRY_INDICATOR_CHAR                          '$'
#define COMMAND_INDICATOR_CHAR                            '{'
#define NOTE_INDICATOR_CHAR                               '}'

#define DATA_ARRAY_STR_LEN                                 24
#define DATA_ARRAY_LEN                                     39
#define COMMAND_ARRAY_LEN                                   3

#define TMP_36_PIN                                         A0
#define GPS_RESET_PIN                                      36
#define REFERENCE_VOLTAGE                                 5.0
#define ADC_COUNT                                      1024.0
#define VOLTAGE_RATIO          (REFERENCE_VOLTAGE / ADC_COUNT)

#define PIN_SIGNAL_STRENGTH                                 6

#define CLOCK_ADDRESS                                    0x68  // This is the I2C address

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


#define GPS_OFFSET_SHIFT_AMOUNT                        0.0002

#define ANTENNA_CONTROL_INDICATOR_CHAR                            '|'
#define ANTENNA_MSG_OFFSET_INIT                                    -1
#define ANTENNA_MSG_MAX_SIZE                                       12
#define ANTENNA_MSG_MAX_OFFSET             (ANTENNA_MSG_MAX_SIZE - 1)    
#define ANTENNA_MSG_EXPECTED_LEN                                    8

latLonAlt _launchStationPosition;  
latLonAlt _balloonPosition;
azDistVals _calculatedValuesBalloon;

uint8_t _doOffset = TRUE;
uint8_t _launchOffsetDirection;

char _radioMessageIn[RADIO_MSG_MAX_SIZE];
char _radioMessageOut[RADIO_MSG_MAX_SIZE];
uint16_t _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
uint16_t _radioMsgOutCount = RADIO_MSG_OFFSET_INIT;

char _lastCommand[RADIO_MSG_MAX_SIZE];

MessageValidation _messageValidator;

char _commandTemp[RADIO_MSG_MAX_SIZE];

uint8_t second;
uint8_t minute; 
uint8_t hour;
uint8_t day;
uint8_t month;
uint8_t year;

char _gpsDate[12];
char _gpsTimestamp[12];
char _gpsData[64];

Timer _timer;
char _video_data[16];

int _signalStrength = 0;

char _telemetry[RADIO_MSG_MAX_SIZE];


char _antennaControllerMsg[ANTENNA_MSG_MAX_SIZE];
uint8_t _antennaMsgCount = 0;


void send_data_to_video_antenna() {

    char az[6];
    char el[6];

    Serial.print("calc'd az: "); Serial.print(_calculatedValuesBalloon.azimuth);
    Serial.print("  el: "); Serial.println(_calculatedValuesBalloon.elevation);

    if ((_calculatedValuesBalloon.azimuth >= 0.0) && (_calculatedValuesBalloon.azimuth <= 360.0)) {
        if ((_calculatedValuesBalloon.azimuth >= -20.0) && (_calculatedValuesBalloon.elevation <= 90.0)) {
            
            if ((_calculatedValuesBalloon.azimuth != 0.0) && (_calculatedValuesBalloon.elevation != 0.0))
            {
                dtostrf(_calculatedValuesBalloon.azimuth, 5, 1, az);
                dtostrf(_calculatedValuesBalloon.elevation, 5, 1, el);

                sprintf(_video_data, "|%s,%s\n", az, el);

                // Send to onboard directional antenna controller
                Serial3.print(_video_data);
            }
        }
    }
}


void watchdogSetup()
{
    cli();  // Disables the interrupts so no other interrupts get called while we are setting up
    wdt_reset();

    /*
    WDP  WDP  WDP  WDP  Time - out
    3    2    1    0     (ms)
    0    0    0    0      16
    0    0    0    1      32
    0    0    1    0      64
    0    0    1    1     125
    0    1    0    0     250
    0    1    0    1     500
    0    1    1    0    1000
    0    1    1    1    2000
    1    0    0    0    4000
    1    0    0    1    8000
    */

    /*
    WDTCSR configuration:
    WDIE = 1: Interrupt Enable
    WDE = 1 :Reset Enable
    WDP3 = 0 :For 2000ms Time-out
    WDP2 = 1 :For 2000ms Time-out
    WDP1 = 1 :For 2000ms Time-out
    WDP0 = 1 :For 2000ms Time-out
    */

    // Enter WatchDog configuration mode:
    // (1 << 5) generated a byte with all zeros and one 1 at the 5th (counting from zero) bit from the right.
    // hence, for example, (1<<WDCE) generates "00010000", since WDCE=4 (see datasheet 10.9.2)
    WDTCSR |= (1 << WDCE) | (1 << WDE);

    // Set WatchDog Settings:
    //WDTCSR = (1<<WDE) | (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);

    sei(); // enable interrupts
}


void watchdog_reset() {
    wdt_reset();
}


void generateTelemetry() {

    char scratchRegister1[16];
    char scratchRegister2[16];
    char scratchRegister3[16];
    char scratchRegister4[16];
    char scratchRegister5[16];

    //getDateTime();

    sprintf(_gpsDate, "20%02d-%02d-%02dT", year, month, day);
    sprintf(_gpsTimestamp, "%02d:%02d:%02dZ", hour, minute, second);

    sprintf(_gpsData, "%s,%s,%s,0.0,0.0,0,0",
        dtostrf(_launchStationPosition.lat, 3, 4, scratchRegister1),
        dtostrf(_launchStationPosition.lon, 3, 4, scratchRegister2),
        dtostrf(_launchStationPosition.alt, 2, 1, scratchRegister3)
        );


    //                     DT  IT OT H  AP GP SS AX AY AZ HE BL BR

    //getting the voltage reading from the temperature sensor
    int tmpReading = analogRead(TMP_36_PIN);

    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage_value = tmpReading * VOLTAGE_RATIO;

    // now print out the temperature
    float temperatureC = (voltage_value - 0.5) * 100;  //converting from 10 mv per degree wit 500 mV offset
    //to degrees ((tmp_voltage - 500mV) times 100)

    sprintf(_telemetry, "#:%s%s,%s,%s,%s,%s,%d,%d,%d,%s,%s,0.0",
        _gpsDate,
        _gpsTimestamp,
        dtostrf(temperatureC, 2, 1, scratchRegister1),
        _gpsData,
        dtostrf(_calculatedValuesBalloon.azimuth, 2, 1, scratchRegister2),
        dtostrf(_calculatedValuesBalloon.elevation, 2, 1, scratchRegister3),
        _signalStrength,
        _messageValidator.getReceptionErrorCount(),
        0,
        dtostrf(_calculatedValuesBalloon.distanceGround, 2, 1, scratchRegister4),
        dtostrf(_calculatedValuesBalloon.distanceTotal, 2, 1, scratchRegister5)
        );

    _messageValidator.appendCheckSum(_telemetry);

    //Serial.println(_telemetry);

    Serial1.print(_telemetry);
    Serial1.print("\n");

    Serial.print(_telemetry);
    Serial.print("\n");
}


void get_eeprom_data()
{
    _calculatedValuesBalloon.azimuth = -1.0;
    _calculatedValuesBalloon.elevation = -1.0;
    _calculatedValuesBalloon.distanceGround = 0.0;
    _calculatedValuesBalloon.distanceTotal = 0.0;

    _balloonPosition.alt = 0.0;
    _balloonPosition.lat = 0.0;
    _balloonPosition.lon = 0.0;

    EEPROM.get(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
    EEPROM.get(GBANG_DO_OFFSET_ADDRESS, _doOffset);
    EEPROM.get(GBANG_OFFSET_DIRECTION_ADDRESS, _launchOffsetDirection);

}


void reset_eeprom_values(int direction)
{
    Serial.println("Resetting the location and EEPROM values");

    _doOffset = TRUE;

    _launchOffsetDirection = direction;

    _launchStationPosition.lat = 0.0;
    _launchStationPosition.lon = 0.0;
    _launchStationPosition.alt = 0.0;

    EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
    EEPROM.put(GBANG_DO_OFFSET_ADDRESS, _doOffset);

    _launchOffsetDirection = direction;
    EEPROM.put(GBANG_OFFSET_DIRECTION_ADDRESS, _launchOffsetDirection);

}


char* get_offset_direction(int dir)
{
    switch (dir) {
        case GPS_OFFSET_S:
            return "South";

        case GPS_OFFSET_SW:
            return "SouthWest";

        case GPS_OFFSET_W:
            return "West";

        case GPS_OFFSET_NW:
            return "NorthWest";

        case GPS_OFFSET_N:
            return "North";

        case GPS_OFFSET_NE:
            return "NorthEase";

        case GPS_OFFSET_E:
            return "East";

        case GPS_OFFSET_SE:
            return "SouthEast";
    }

    return "Invalid Offset Direction!!!";

}

void setup()
{
    // Debug Console & external directional antenna controller
    Serial.begin(DEBUG_CONSOLE_SERIAL_BAUD_RATE);

    // Field Gateway
    Serial1.begin(FIELD_GATEWAY_SERIAL_BAUD_RATE);

    Serial.println("Starting the radio receiver...");

    // Radio
    Serial2.begin(RADIO_SERIAL_BAUD_RATE);

    Serial.println("Starting the directional antenna...");

    // Video Directional Antenna
    Serial3.begin(DIRECTIONAL_ANTENNA_SERIAL_BAUD_RATE);

    //pinMode(GPS_RESET_PIN, INPUT);

    //  Uncomment the next line and run to reset the eeprom (lat/lon) values
    //  Directional value is the direction the antenna is POINTING when centered
    reset_eeprom_values(GPS_OFFSET_NE);

    get_eeprom_data();

    Serial.print("Starting P2 Launch Ground Station...\n\n");
    Serial.print("_launchStationPosition: ");
    Serial.print(_launchStationPosition.lat); Serial.print(", ");
    Serial.print(_launchStationPosition.lon); Serial.print(" @ ");
    Serial.print(_launchStationPosition.alt); Serial.print(" meters");
    Serial.println();

    Serial.print("_doOffset: ");
    Serial.print(_doOffset);
    Serial.println();

    Serial.print("_launchOffsetDirection: ");
    Serial.print(get_offset_direction(_launchOffsetDirection));
    Serial.println();

    if (_launchStationPosition.lat == 0.0) {
        Serial.println("Waiting for antenna calibration...");
        delay(12000);
    }

    watchdogSetup();

    _timer.every(1000, watchdog_reset);
    _timer.every(2000, generateTelemetry);

}


// Process the commands coming from the Field Gateway
int8_t processCommand(int cmdDataCount) {
    int commandDataValue;

    int8_t commandValid = -1;

    Serial.println("Command Received: "); Serial.println(_radioMessageOut);

    if (_radioMessageOut[0] == COMMAND_INDICATOR_CHAR) {

        if ((_radioMessageOut[1] == 'G') || (_radioMessageOut[1] == 'g')) {

            if (_radioMessageOut[3] == '!') {

                _launchStationPosition.lat = _balloonPosition.lat;
                _launchStationPosition.lon = _balloonPosition.lon;
                _launchStationPosition.alt = _balloonPosition.alt;

                Serial.print("Starting: Lat: "); Serial.print(_launchStationPosition.lat);
                Serial.print("  Lon: "); Serial.println(_launchStationPosition.lat);

                if (_doOffset) {
                    switch (_launchOffsetDirection) {
                    case GPS_OFFSET_S:
                        _launchStationPosition.lat += GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_SW:
                        _launchStationPosition.lat += GPS_OFFSET_SHIFT_AMOUNT;
                        _launchStationPosition.lon += GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_W:
                        _launchStationPosition.lon += GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_NW:
                        _launchStationPosition.lat -= GPS_OFFSET_SHIFT_AMOUNT;
                        _launchStationPosition.lon += GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_N:
                        _launchStationPosition.lat -= GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_NE:
                        _launchStationPosition.lat -= GPS_OFFSET_SHIFT_AMOUNT;
                        _launchStationPosition.lon -= GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_E:
                        _launchStationPosition.lon -= GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    case GPS_OFFSET_SE:
                        _launchStationPosition.lat += GPS_OFFSET_SHIFT_AMOUNT;
                        _launchStationPosition.lon -= GPS_OFFSET_SHIFT_AMOUNT;
                        break;

                    }
                }

                Serial.println("Writing to EEPROM!");

                EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);

                Serial.print("EE Lat: "); Serial.print(_launchStationPosition.lat);
                Serial.print("EE Lon: "); Serial.print(_launchStationPosition.lon);
                Serial.print("EE Alt: "); Serial.print(_launchStationPosition.alt);
                Serial.println(" meters.");


                if (!((_launchStationPosition.lat == 0.0) || (_balloonPosition.lat == 0.0) || (_launchStationPosition.lon == 0.0) || (_balloonPosition.lon == 0.0))) {
                    Calculate(_launchStationPosition, _balloonPosition, &_calculatedValuesBalloon);

                    if (_calculatedValuesBalloon.elevation < -0.0) {
                        _calculatedValuesBalloon.elevation = 0.0;
                    }

                    Serial.println(" > send_data_to_video_antenna");
                    // Send data to the other antenna and move it
                    send_data_to_video_antenna();
                }

                commandValid = 0;
            }
        }
        else if ((_radioMessageOut[1] == 'R') || (_radioMessageOut[1] == 'r')) {
            if (_radioMessageOut[3] == '!') {

                _doOffset = TRUE;

                _launchStationPosition.lat = 0.0;
                _launchStationPosition.lon = 0.0;
                _launchStationPosition.alt = 0.0;

                EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
                EEPROM.put(GBANG_DO_OFFSET_ADDRESS, _doOffset);
            }
        }
        else if (cmdDataCount == 3) {
            _launchStationPosition.lat = atof(_commandDataReceived[0]);
            _launchStationPosition.lon = atof(_commandDataReceived[1]);
            _launchStationPosition.alt = atof(_commandDataReceived[2]);

            Serial.print(" Lat: "); Serial.print(_launchStationPosition.lat);
            Serial.print(" Lon: "); Serial.print(_launchStationPosition.lon);
            Serial.print(" Alt: "); Serial.print(_launchStationPosition.alt);
            Serial.println(" meters.");
            commandValid = 0;
        }
    }

    return commandValid;
}




void loop()
{
    _timer.update();


    // Field Gateway
    //  FFFFFF   GGGG
    //  FF      GG  
    //  FFFFF   GG GGG
    //  FF      GG  GG
    //  FF       GGGG
    while (Serial1.available() > 0) {
        char incomingByte = (char) Serial1.read();

        if (incomingByte == COMMAND_INDICATOR_CHAR) {
            _radioMsgOutCount = RADIO_MSG_OFFSET_INIT;
        }

        _radioMsgOutCount++;

        if (_radioMsgOutCount >= RADIO_MSG_MAX_OFFSET) {
            _radioMsgOutCount = 0;
        }

        _radioMessageOut[_radioMsgOutCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\') || (incomingByte == '|')) {

            _radioMessageOut[_radioMsgOutCount] = 0;
            _radioMsgOutCount = RADIO_MSG_OFFSET_INIT;

            strcpy(_commandTemp, _radioMessageOut);

            int cmdCount = splitCommandData(_commandTemp);

            // Process command
            if (processCommand(cmdCount) == -1) {
                Serial.print("Sending Command: "); Serial.println(_radioMessageOut);

                Serial2.print(_radioMessageOut);
                Serial2.print("\n");
            }

        }
    }


    //Radio
    //  RRRRR     AAAA   DDDDD   IIIIII   OOOO
    //  RR  RR   AA  AA  DD  DD    II    OO  OO
    //  RRRRR    AAAAAA  DD  DD    II    OO  OO
    //  RR RR    AA  AA  DD  DD    II    OO  OO
    //  RR  RR   AA  AA  DDDDD   IIIIII   OOOO 
    while (Serial2.available() > 0) {
        int validatorPos;

        char incomingByte = (char) Serial2.read();

        // Craft Telemetry
        if (incomingByte == TELEMETRY_INDICATOR_CHAR) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
        }
        // Craft messages
        if (incomingByte == NOTE_INDICATOR_CHAR) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
        }

        _radioMsgInCount++;

        if (_radioMsgInCount >= RADIO_MSG_MAX_OFFSET) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;

            Serial.print("#Data 2 Error: msg > "); Serial.print(RADIO_MSG_MAX_SIZE); Serial.println(" bytes");
        }

        _radioMessageIn[_radioMsgInCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\') || (incomingByte == '|')) {

            _radioMessageIn[_radioMsgInCount] = 0;

            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;

            if ((validatorPos = _messageValidator.validateMessage(_radioMessageIn)) != -1) {

               //Serial.print("validatorPos: "); Serial.println(validatorPos);

                

                if (_radioMessageIn[0] == TELEMETRY_INDICATOR_CHAR) {
                    second = (uint8_t) ((_radioMessageIn[19] - 48) * 10 + (_radioMessageIn[20] - 48));
                    minute = (uint8_t) ((_radioMessageIn[16] - 48) * 10 + (_radioMessageIn[17] - 48));
                    hour = (uint8_t) ((_radioMessageIn[13] - 48) * 10 + (_radioMessageIn[14] - 48));
                    day = (uint8_t) ((_radioMessageIn[10] - 48) * 10 + (_radioMessageIn[11] - 48));
                    month = (uint8_t) ((_radioMessageIn[7] - 48) * 10 + (_radioMessageIn[8] - 48));
                    year = (uint8_t) ((_radioMessageIn[4] - 48) * 10 + (_radioMessageIn[5] - 48));

                }
                
                   
                // Received telemetry so pull out lat/lon/alt for positioning
                if (_radioMessageIn[0] == TELEMETRY_INDICATOR_CHAR){

                    strcpy(_commandTemp, _radioMessageIn);

                    _commandTemp[validatorPos] = 0;

                    //Serial.print("_commandTemp: "); Serial.println(_commandTemp);

                    if (splitCommandData(_commandTemp) == DATA_ARRAY_LEN) {
                        
                        _balloonPosition.lat = atof(_commandDataReceived[POS_LAT]);
                        _balloonPosition.lon = atof(_commandDataReceived[POS_LON]);
                        _balloonPosition.alt = atof(_commandDataReceived[POS_ALT]);

                        Serial.print("Launch/Balloon:  ("); Serial.print(_launchStationPosition.lat, 5);
                        Serial.print(", "); Serial.print(_launchStationPosition.lon, 5);
                        Serial.print(", "); Serial.print(_launchStationPosition.alt, 5);
                        Serial.print(")  ("); Serial.print(_balloonPosition.lat, 5);
                        Serial.print(", "); Serial.print(_balloonPosition.lon, 5);
                        Serial.print(", "); Serial.print(_balloonPosition.alt, 5);
                        Serial.print(") ");

                        if ((_launchStationPosition.lat != 0.0) && (_balloonPosition.lat != 0.0) && (_launchStationPosition.lon != 0.0) || (_balloonPosition.lon != 0.0)) {

                            Calculate(_launchStationPosition, _balloonPosition, &_calculatedValuesBalloon);

                            Serial.print("AZ/EL: ("); Serial.print(_calculatedValuesBalloon.azimuth, 1);
                            Serial.print(", "); Serial.print(_calculatedValuesBalloon.elevation, 1);
                            Serial.print(")");

                            Serial.print("Dist Grd/Tot: ("); Serial.print(_calculatedValuesBalloon.distanceGround, 1);
                            Serial.print(", "); Serial.print(_calculatedValuesBalloon.distanceTotal, 1);
                            Serial.print(")");

                            if (_calculatedValuesBalloon.elevation < -0.0) {
                                _calculatedValuesBalloon.elevation = 0.0;
                            }

                            // Send data to the other antenna and move it
                            send_data_to_video_antenna();
                        }
                        Serial.println();
                    }
                }

                // Send out to field gateway
                Serial1.println(_radioMessageIn);
                
                // Debugging...
                Serial.println(_radioMessageIn);
            } else
            {
                Serial.print(">>>Recept Err: '"); Serial.print(_radioMessageIn); Serial.println("'");
            }
        }
    }

    char _az_el_msg[AZ_EL_MAX_SIZE];

    // Antenna
    //  AAAA   NN  NN  TTTTTT  EEEEEE  NN  NN  NN  NN   AAAA 
    // AA  AA  NNN NN    TT    EE      NNN NN  NNN NN  AA  AA
    // AAAAAA  NN NNN    TT    EEEE    NN NNN  NN NNN  AAAAAA
    // AA  AA  NN  NN    TT    EE      NN  NN  NN  NN  AA  AA
    // AA  AA  NN  NN    TT    EEEEEE  NN  NN  NN  NN  AA  AA
    while (Serial3.available() > 0) {

        char incomingByte = (char)Serial3.read();

        if (incomingByte == AZ_EL_INDICATOR_CHAR) {
            _antennaMsgCount = AZ_EL_OFFSET_INIT;
        }
       
        _antennaMsgCount++;

        if (_antennaMsgCount >= AZ_EL_MAX_OFFSET) {
            _antennaMsgCount = AZ_EL_OFFSET_INIT;
        }

        _antennaControllerMsg[_antennaMsgCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\')) {

            _antennaControllerMsg[_antennaMsgCount] = 0;

            Serial.println(_antennaControllerMsg);

            _antennaMsgCount = AZ_EL_OFFSET_INIT;
            
            if (_az_el_msg[0] == AZ_EL_INDICATOR_CHAR) {

                if ((_az_el_msg[4] == '.') && (_az_el_msg[6] == ',') && (_az_el_msg[10] == '.') && (strlen(_az_el_msg) == 12)) {

                    Serial.print("Actual Az/El: "); Serial.println(&_az_el_msg[1]);
                }
            }
        }
    }



    // DEBUGGING
    while (Serial.available() > 0) {

        char incomingByte = (char) Serial.read();

        // Is Craft Telemetry arriving...?
        if (incomingByte == TELEMETRY_INDICATOR_CHAR) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
        }

        if (incomingByte == COMMAND_INDICATOR_CHAR) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
            //Serial.println("Caught a command coming in!!!");
        }

        if (incomingByte == '|') {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
            //Serial.println("Message to Antenna Driver!");
        }

        _radioMsgInCount++;

        if (_radioMsgInCount >= RADIO_MSG_MAX_OFFSET) {
            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;

            //Serial.print("#Data 2 Error: msg > "); Serial.print(RADIO_MSG_MAX_SIZE); Serial.println(" bytes");
        }

        _radioMessageIn[_radioMsgInCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\') || (incomingByte == '|')) {

            _radioMessageIn[_radioMsgInCount] = 0;

            //Serial.print("_radioMessageIn: ");
            //Serial.println(_radioMessageIn);

            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;

            if (_messageValidator.validateMessage(_radioMessageIn) != -1) {

                // Received telemetry so pull out lat/lon/alt for positioning
                if (_radioMessageIn[0] == TELEMETRY_INDICATOR_CHAR){

                    strcpy(_commandTemp, _radioMessageIn);

                    int count = splitCommandData(_commandTemp);
                    Serial.print("Data Array Count: "); Serial.println(count);

                    if (count == DATA_ARRAY_LEN) {
                        
                        _balloonPosition.lat = atof(_commandDataReceived[POS_LAT]);
                        _balloonPosition.lon = atof(_commandDataReceived[POS_LON]);
                        _balloonPosition.alt = atof(_commandDataReceived[POS_ALT]);


                        Serial.print(" -LLat: "); Serial.print(_launchStationPosition.lat, 4);
                        Serial.print(" -LLon: "); Serial.print(_launchStationPosition.lon, 4);

                        Serial.print(" -BLat: "); Serial.print(_balloonPosition.lat, 4);
                        Serial.print(" -BLon: "); Serial.println(_balloonPosition.lon, 4);

                        if (!((_launchStationPosition.lat == 0.0) || (_balloonPosition.lat == 0.0) || (_launchStationPosition.lon == 0.0) || (_balloonPosition.lon == 0.0))) {
                            Calculate(_launchStationPosition, _balloonPosition, &_calculatedValuesBalloon);

                            if (_calculatedValuesBalloon.elevation < -0.0) {
                                _calculatedValuesBalloon.elevation = 0.0;
                            }
                         
                            // Send data to the other antenna and move it
                            Serial.println("send_data_to_video_antenna");
                            send_data_to_video_antenna();

                        }

                        Serial.print(" LLat: "); Serial.print(_launchStationPosition.lat, 4);
                        Serial.print(" LLon: "); Serial.print(_launchStationPosition.lon, 4);

                        Serial.print(" BLat: "); Serial.print(_balloonPosition.lat, 4);
                        Serial.print(" BLon: "); Serial.print(_balloonPosition.lon, 4);
                        Serial.print(" BAlt: "); Serial.print(_balloonPosition.alt);
                        Serial.print(" meters.");

                        Serial.print(" Az: "); Serial.print(_calculatedValuesBalloon.azimuth);
                        Serial.print(" El: "); Serial.print(_calculatedValuesBalloon.elevation);

                        Serial.print(" Ground: "); Serial.print(_calculatedValuesBalloon.distanceGround);
                        Serial.print(" Total: "); Serial.print(_calculatedValuesBalloon.distanceTotal);
                        Serial.println();

                        //Serial.print(" Lat: "); Serial.print(_launchStationPosition.lat);
                        //Serial.print(" Lon: "); Serial.print(_launchStationPosition.lon);
                        //Serial.print(" Alt: "); Serial.print(_launchStationPosition.alt);
                        //Serial.print(" meters.");
                        //Serial.println();

                    }
                }
                else if (_radioMessageIn[0] == COMMAND_INDICATOR_CHAR) {

                    int cmdLen = strlen(_radioMessageIn);
                    if (cmdLen > 4) {
                        
                        _radioMessageIn[cmdLen - 4] = '\0';

                        strcpy(_radioMessageOut, _radioMessageIn);

                        int cmdCount = splitCommandData(_commandTemp);

                        // Process command
                        if (processCommand(cmdCount) == -1) {
                            Serial.print("Didn't process the command: "); Serial.println(_radioMessageIn);

                        }
                    }
                }

                //// Send out to field gateway
                //Serial1.println(_radioMessageIn);

                //// Debugging...
                //Serial.println(_radioMessageIn);
            }
            else
            {
                Serial.println(_radioMessageIn);

                if (!strcmp(_radioMessageIn, "H!"))
                {
                    Serial3.println("|H!");
                }
            }
        }
    }

    delay(10);
}
