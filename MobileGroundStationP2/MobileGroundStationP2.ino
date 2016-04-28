
#include <SD/SD.h>
#include <Wire.h>

#include <SoftwareSerial.h>
#include <SPI.h>

#include <math.h>

#include <Timer.h>
#include <avr/wdt.h>

#include <Adafruit_GPS.h>

#include <EEPROM.h>

#include <PegasusCommandProcessor.h>
#include <AzimuthElevation.h>
#include <MessageValidation.h>

#ifndef TRUE
#define TRUE    (1==1)
#endif

#ifndef FALSE
#define FALSE   (1==2)
#endif

#define LAT_LON_EEPROM_ADDRESS                              0

#define DEBUG_DO_DISPLAY                                    0

#define RADIO_MSG_MAX_SIZE                                256
#define RADIO_MSG_MAX_OFFSET         (RADIO_MSG_MAX_SIZE - 1)
#define RADIO_MSG_OFFSET_INIT                              -1


#define RADIO_SERIAL_BAUD_RATE                          38400
#define FIELD_GATEWAY_SERIAL_BAUD_RATE                  38400


#define STATUS_GO                                           1
#define STATUS_NOGO                                         0

#define GPSECHO                                         false

#define DO_LOG_DATA_TO_SD                                TRUE

#define CLOCK_ADDRESS                                    0x68  // This is the I2C address

#define MPH_PER_KNOT                        1.150779448023543
#define KPH_PER_KNOT                                    1.852

#define DO_SPEED_IN_MPH                                  TRUE          
#define DO_SPEED_IN_KPH                                 FALSE   

#define TELEMETRY_INDICATOR_CHAR                          '$'
#define COMMAND_INDICATOR_CHAR                            '{'
#define NOTE_INDICATOR_CHAR                               '}'


#define POS_LAT                                            23
#define POS_LON                                            24
#define POS_ALT                                             2

#define TMP_36_PIN                                         A0
#define REFERENCE_VOLTAGE                                 5.0
#define ADC_COUNT                                      1024.0
#define VOLTAGE_RATIO          (REFERENCE_VOLTAGE / ADC_COUNT)

#define DATA_ARRAY_LEN                                     39


Timer _timer;

bool _realTimeClockSet = false;

MessageValidation _messageValidator;

uint8_t _balloonReleased = 0;

Adafruit_GPS _gpsSensor(&Serial1);
uint8_t _usingInterrupt = false;

void useInterrupt(boolean v);

char _telemetryTemp[RADIO_MSG_MAX_SIZE];

char _commandTemp[64];

char _gpsDate[12];
char _gpsTimestamp[12];
char _gpsData[64];
float _latitude = 0.0;
float _longitude = 0.0;
int _signalStrength = 0;


char _radioMessageIn[RADIO_MSG_MAX_SIZE];
char _radioMessageOut[RADIO_MSG_MAX_SIZE];
uint16_t _radioMsgInCount = RADIO_MSG_OFFSET_INIT;
uint16_t _radioMsgOutCount = RADIO_MSG_OFFSET_INIT;


uint16_t _prevAltitude;


uint8_t second;
uint8_t minute;
uint8_t hour;
uint8_t dayOfWeek;
uint8_t day;
uint8_t month;
uint8_t year;

latLonAlt _launchStationPosition;
latLonAlt _mobileStationPosition;
latLonAlt _balloonPosition;
azDistVals _calculatedValuesBalloon;
azDistVals _calculatedValuesStations;

uint8_t _logCounter = 0;


void generateTelemetry();


//// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
    char c = _gpsSensor.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
    if (v) {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
    }
    else {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
    }
    _usingInterrupt = v;
}


#define CHIP_SELECT_PIN 10
uint32_t _eventCounter = 0;
uint8_t _timerExpiredCheckDone = 0;
char _filename[24];
File _logfile;


void setupSDLogging() {

    if (DO_LOG_DATA_TO_SD) {

        // make sure that the default chip select pin is set to
        // output, even if you don't use it:
        pinMode(CHIP_SELECT_PIN, OUTPUT);

        // see if the card is present and can be initialized:
        if (!SD.begin(CHIP_SELECT_PIN, 11, 12, 13)) {
            Serial.println("Card init. failed!");
        }
        Serial.println("card initialized.");

        strcpy(_filename, "PEGLOG00.TXT");

        for (uint8_t i = 0; i < 100; i++) {
            _filename[6] = '0' + i / 10;
            _filename[7] = '0' + i % 10;
            // create if does not exist, do not open existing, write, sync after write
            if (!SD.exists(_filename)) {
                break;
            }
        }

        _logfile = SD.open(_filename, FILE_WRITE);
        
        if (_logfile == NULL) {
            Serial.print("Couldn't create ");
            Serial.println(_filename);
        }
        else {
            Serial.print("Writing to ");
            Serial.println(_filename);
        }
    }
}


void writeToLogFile(char* dataToWrite) {

    if (DO_LOG_DATA_TO_SD) {

        _logfile.write(dataToWrite);

        _logfile.write("\n");

        _logCounter++;

        if (_logCounter >= 5)
        {
            _logfile.flush();

            _logCounter = 0;
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


void reset_eeprom_values()
{
    Serial.println("Resetting the location and EEPROM values");

    _launchStationPosition.lat = 0.0;
    _launchStationPosition.lon = 0.0;
    _launchStationPosition.alt = 0.0;

    EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);
}


void get_eeprom_data()
{

    EEPROM.get(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);

}


// Convert binary coded decimal to normal decimal numbers
uint8_t bcdToDec(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(uint8_t val)
{
    return ((val / 10 * 16) + (val % 10));
}


void setDateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    uint8_t dayOfWeek = (uint8_t) 0;
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write(uint8_t(0x00));
    Wire.write(decToBcd(second));  // 0 to bit 7 starts the clock
    Wire.write(decToBcd(minute));
    Wire.write(decToBcd(hour));    // If you want 12 hour am/pm you need to set
    // bit 6 (also need to change readDateDs1307)
    Wire.write(decToBcd(dayOfWeek));
    Wire.write(decToBcd(day));
    Wire.write(decToBcd(month));
    Wire.write(decToBcd(year));
    Wire.endTransmission();
}

void getDateTime() {
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write(uint8_t(0x00));
    Wire.endTransmission();

    Wire.requestFrom(CLOCK_ADDRESS, 7);

    // A few of these need masks because certain bits are control bits
    second = bcdToDec(Wire.read() & 0x7f);
    minute = bcdToDec(Wire.read());

    // Need to change this if 12 hour am/pm
    hour = bcdToDec(Wire.read() & 0x3f);
    dayOfWeek = bcdToDec(Wire.read());
    day = bcdToDec(Wire.read());
    month = bcdToDec(Wire.read());
    year = bcdToDec(Wire.read());
}

void setup()
{

    Serial.begin(FIELD_GATEWAY_SERIAL_BAUD_RATE);   // Field Gateway
    Serial2.begin(RADIO_SERIAL_BAUD_RATE);  // 'radio
    
    Wire.begin();

    //  Uncomment the next line and run to reset the eeprom (lat/lon) values
    // reset_eeprom_values();

    get_eeprom_data();

#if(DEBUG_DO_DISPLAY)
    delay(5000);
    Serial.print("_launchStationPosition.lat: "); Serial.println(_launchStationPosition.lat);
    Serial.print("_launchStationPosition.lon: "); Serial.println(_launchStationPosition.lon);
    Serial.print("_launchStationPosition.alt: "); Serial.println(_launchStationPosition.alt);
#endif

#if(DEBUG_DO_DISPLAY)
    Serial.println("Starting.");
#endif

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    _gpsSensor.begin(9600);
    _gpsSensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    _gpsSensor.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    _gpsSensor.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
    _gpsSensor.sendCommand(PGCMD_ANTENNA);

    useInterrupt(true);

#if(DEBUG_DO_DISPLAY)
    Serial.println("GPS SETUP DONE.");
#endif

    _calculatedValuesBalloon.azimuth = -1.0;
    _calculatedValuesBalloon.elevation = -1.0;
    _calculatedValuesBalloon.distanceGround = 0.0;
    _calculatedValuesBalloon.distanceTotal = 0.0;

    _balloonPosition.alt = 0.0;
    _balloonPosition.lat = 0.0;
    _balloonPosition.lon = 0.0;

    _gpsDate[0] = 0;

    setupSDLogging();

    watchdogSetup();

    _timer.every(1000, watchdog_reset);
    _timer.every(2000, generateTelemetry);
}


/**
* \fn convert_knots_to_mph
* \brief convert the current speed from knots per hour
* \return speed in miles per hour
*/
double convert_knots_to_mph(double speedKnots) {
    return speedKnots * MPH_PER_KNOT;
}


/**
* \fn convert_knots_to_kph
* \brief convert the current speed from knots per hour
* \return speed in kilometers per hour
*/
double convert_knots_to_kph(double speedKnots) {
    return speedKnots * KPH_PER_KNOT;
}

/**
* \fn do_speed_conversion
* \brief determines the desired speed format and does the conversion
* \return speed in desired measure per hour
*/
double do_speed_conversion(double speedKnots) {

    if (DO_SPEED_IN_MPH) {
        return convert_knots_to_mph(speedKnots);
    }

    if (DO_SPEED_IN_KPH) {
        return convert_knots_to_kph(speedKnots);
    }

    return speedKnots;
}


void generateTelemetry() {

    char scratchRegister1[16];
    char scratchRegister2[16];
    char scratchRegister3[16];
    char scratchRegister4[16];
    char scratchRegister5[16];
    char scratchRegister6[16];

    getDateTime();

    sprintf(_gpsDate, "20%02d-%02d-%02dT", year, month, day);
    sprintf(_gpsTimestamp, "%02d:%02d:%02dZ", hour, minute, second);
    
    if (_gpsSensor.newNMEAreceived()) {

        if (_gpsSensor.parse(_gpsSensor.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false

            if (_gpsSensor.fix) {
                
                _prevAltitude = _gpsSensor.altitude;

                sprintf(_gpsData, "%s,%s,%s,%s,%s,%d,%d",
                    dtostrf(_gpsSensor.latitudeDegrees, 3, 4, scratchRegister1),
                    dtostrf(_gpsSensor.longitudeDegrees, 3, 4, scratchRegister2),
                    dtostrf(_gpsSensor.altitude, 2, 1, scratchRegister3),
                    dtostrf(do_speed_conversion(_gpsSensor.speed), 2, 1, scratchRegister4),
                    dtostrf(_gpsSensor.angle, 2, 2, scratchRegister5),
                    _gpsSensor.fix,
                    _gpsSensor.satellites
                    );
            }
            else {
                sprintf(_gpsData, "0.0,0.0,%s,0.0,0.0,0,0",
                    dtostrf(_prevAltitude, 2, 1, scratchRegister3)
                    );

            }
        }
    }
    else {
        sprintf(_gpsData, "0.0,0.0,%s,0.0,0.0,0,0",
            dtostrf(_prevAltitude, 2, 1, scratchRegister3)
            );
    }


    char telemetry[256];
    //                     DT  IT OT H  AP GP SS AX AY AZ HE BL BR


    //getting the voltage reading from the temperature sensor
    int tmpReading = analogRead(TMP_36_PIN);

    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage_value = tmpReading * VOLTAGE_RATIO;

    // now print out the temperature
    float temperatureC = (voltage_value - 0.5) * 100;  //converting from 10 mv per degree wit 500 mV offset
    //to degrees ((tmp_voltage - 500mV) times 100)

    sprintf(telemetry, "#:%s%s,%s,%s,%s,%s,%d,%d,%d,%s,%s,%s",
        _gpsDate,
        _gpsTimestamp,
        dtostrf(temperatureC, 2, 1, scratchRegister6),
        _gpsData,
        dtostrf(_calculatedValuesBalloon.azimuth, 2, 1, scratchRegister1),
        dtostrf(_calculatedValuesBalloon.elevation, 2, 1, scratchRegister2),
        _signalStrength,
        _messageValidator.getReceptionErrorCount(),
        0,
        dtostrf(_calculatedValuesBalloon.distanceGround, 2, 1, scratchRegister3),
        dtostrf(_calculatedValuesBalloon.distanceTotal, 2, 1, scratchRegister4),
        dtostrf(_calculatedValuesStations.distanceGround, 2, 1, scratchRegister5)
        );

    _messageValidator.appendCheckSum(telemetry);

    //Serial.print(" ---> Telemetry out> ");
    Serial.println(telemetry);

    writeToLogFile(telemetry);

    // $: timestamp, tempInside, tempOutside, humidity, atPressure, latitude, longitude, altitude, groundSpeed, direction, signalStrength, accelerometerX, accelerometerY, accelerometerZ, heading, batteryLevel, balloonRelease, gpsFix, satellites
    // "$:2014-12-01T13:15:30Z,48.1,-18.2,14.6,6.7523,41.095938,-87.909996,23568.5,47.9,134.3,25.5,5.847,2.39,9.4,33.2,7.1,0,1,6";


}


// {G:41.816577,-87.890244,192
int8_t processCommand(int cmdDataCount) {
    int commandDataValue;

    int8_t commandValid = -1;

    if ((_radioMessageOut[0] == COMMAND_INDICATOR_CHAR) &&
        ((_radioMessageOut[1] == 'G') || (_radioMessageOut[1] == 'g'))) {

        if (_radioMessageOut[3] == '!') {

            _launchStationPosition.lat = _balloonPosition.lat;
            _launchStationPosition.lon = _balloonPosition.lon;
            _launchStationPosition.alt = _balloonPosition.alt;

            EEPROM.put(LAT_LON_EEPROM_ADDRESS, _launchStationPosition);

#if(DEBUG_DO_DISPLAY)
            Serial.print(" Lat: "); Serial.print(_launchStationPosition.lat);
            Serial.print(" Lon: "); Serial.print(_launchStationPosition.lon);
            Serial.print(" Alt: "); Serial.print(_launchStationPosition.alt);
            Serial.println(" meters.");
#endif

            commandValid = 0;

        }
        else {
            //Serial.println("in Else");
            if (cmdDataCount != 3) {
                Serial.println(" Not enough data elements (need 3): ");
            }
            else {
                _launchStationPosition.lat = atof(_commandDataReceived[0]);
                _launchStationPosition.lon = atof(_commandDataReceived[1]);
                _launchStationPosition.alt = atof(_commandDataReceived[2]);

                //Serial.print(" Lat: "); Serial.print(_launchStationPosition.lat);
                //Serial.print(" Lon: "); Serial.print(_launchStationPosition.lon);
                //Serial.print(" Alt: "); Serial.print(_launchStationPosition.alt);
                //Serial.println(" meters.");
                commandValid = 0;
            }
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
    while (Serial.available() > 0) {
        char incomingByte = (char)Serial.read();

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
                //Serial.print("Sending Command: "); Serial.println(_radioMessageOut);

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
        char incomingByte = (char)Serial2.read();

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

            //Serial.print("#Data 1 Error: msg > "); Serial.print(RADIO_MSG_MAX_SIZE); Serial.println(" bytes.");
        }

        _radioMessageIn[_radioMsgInCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\') || (incomingByte == '|')) {

            _radioMessageIn[_radioMsgInCount] = 0;

            _radioMsgInCount = RADIO_MSG_OFFSET_INIT;

            writeToLogFile(_radioMessageIn);

            if (_messageValidator.validateMessage(_radioMessageIn) != -1) {

                //if (!_realTimeClockSet) {
                if (_radioMessageIn[0] == TELEMETRY_INDICATOR_CHAR) {
                    second = (uint8_t)((_radioMessageIn[19] - 48) * 10 + (_radioMessageIn[20] - 48));
                    minute = (uint8_t)((_radioMessageIn[16] - 48) * 10 + (_radioMessageIn[17] - 48));
                    hour = (uint8_t)((_radioMessageIn[13] - 48) * 10 + (_radioMessageIn[14] - 48));
                    //dayOfWeek = (uint8_t) 0;
                    day = (uint8_t)((_radioMessageIn[10] - 48) * 10 + (_radioMessageIn[11] - 48));
                    month = (uint8_t)((_radioMessageIn[7] - 48) * 10 + (_radioMessageIn[8] - 48));
                    year = (uint8_t)((_radioMessageIn[4] - 48) * 10 + (_radioMessageIn[5] - 48));

                    //setDateTime(year, month, day, hour, minute, second);

                    //_realTimeClockSet = true;
                }
                //}


                // Received telemetry so pull out lat/lon/alt for positioning
                if (_radioMessageIn[0] == TELEMETRY_INDICATOR_CHAR) {

                    strcpy(_telemetryTemp, _radioMessageIn);

                    if (splitCommandData(_telemetryTemp) == DATA_ARRAY_LEN) {
                        _balloonPosition.lat = atof(_commandDataReceived[POS_LAT]);
                        _balloonPosition.lon = atof(_commandDataReceived[POS_LON]);
                        _balloonPosition.alt = atof(_commandDataReceived[POS_ALT]);

                        //if (!((_launchStationPosition.lat == 0.0) || (_balloonPosition.lat == 0.0))) {
                        if ((_launchStationPosition.lat != 0.0) && (_balloonPosition.lat != 0.0) && (_launchStationPosition.lon != 0.0) || (_balloonPosition.lon != 0.0)) {

                            Calculate(_launchStationPosition, _balloonPosition, &_calculatedValuesBalloon);

                            if (_calculatedValuesBalloon.elevation < -0.0) {
                                _calculatedValuesBalloon.elevation = -0.0;
                            }


                            if ((_mobileStationPosition.lat != 0.0) && (_mobileStationPosition.lon != 0.0)) {
                                Calculate(_launchStationPosition, _mobileStationPosition, &_calculatedValuesStations);

                                if (_calculatedValuesStations.elevation < -0.0) {
                                    _calculatedValuesStations.elevation = -0.0;
                                }
                            }
                        }

                        //Serial.print(" Az: "); Serial.print(_calculatedValuesBalloon.azimuth);
                        //Serial.print(" El: "); Serial.print(_calculatedValuesBalloon.elevation);

                        //Serial.print("\n>>> LLat: "); Serial.print(_launchStationPosition.lat);
                        //Serial.print(" LLon: "); Serial.print(_launchStationPosition.lon);
                        //Serial.print(" LAlt: "); Serial.print(_launchStationPosition.alt);
                        //Serial.print(" meters.");

                        //Serial.print("\n>>> Lat: "); Serial.print(_balloonPosition.lat);
                        //Serial.print(" Lon: "); Serial.print(_balloonPosition.lon);
                        //Serial.print(" Alt: "); Serial.print(_balloonPosition.alt);
                        //Serial.print(" meters.");

                        //Serial.print(" Ground: "); Serial.print(_calculatedValuesBalloon.distanceGround);
                        //Serial.print(" Total: "); Serial.print(_calculatedValuesBalloon.distanceTotal);

                        //Serial.print(" Peer: "); Serial.print(_calculatedValuesStations.distanceGround);

                        //Serial.println();
                    }
                }

                // Send out to field gateway
                Serial.println(_radioMessageIn);
            }
        }

    }

    delay(10);
}
