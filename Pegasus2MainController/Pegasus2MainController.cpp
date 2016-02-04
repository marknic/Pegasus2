#include <string.h>                     // strlen / strcpy
#include <stdlib.h>                     // atoi / exit
#include <stdio.h>                      // printf / fprintf
#include <time.h>                       // time functions
#include "wiringPi.h"                               // <<<<<<<<<<<<<<<<<
//#include <math.h>

#include "Timer.h"
#include "I2CTransfer.h"                            // <<<<<<<<<<<<<<<<<
#include "Pegasus2MainController.h"
#include "UartStream.h"                             // <<<<<<<<<<<<<<<<<
#include "CommandProcessor.h"
#include "MessageValidation.h"
//#include "GpioCtrl.h"                               // <<<<<<<<<<<<<<<<<
#include "AltitudeCalc.h"

#include <ctype.h>
#include <math.h>
#include <sys/stat.h> /* struct stat, fchmod (), stat (), S_ISREG, S_ISDIR */
#include <sys/time.h>
//#include <stdio.h>


#ifndef uint8_t

#define uint8_t __uint8_t

#endif // !uint_8


// Prototypes
void process_gps_data1(char* dataLine);
void process_gps_data2(char* dataLine);
void process_radio_data(char* dataLine);
int processCommand(char *msg);
void switch_leds(bool onOff);
void process_proc2_data(char* dataLine);
int send_craft_message(int index, int value);
char* format_timestamp(char* timestampDest);
char* format_timestamp_with_milliseconds(char* timestampDest);
void write_to_log(const char* source, char* data);
char* integrate_real_telemetry(char* dataIn);
int release_balloon_now();
int deploy_parachute_now();
int test_uart_streams(char uartNumber);


// Variables
int _telemetry_sent_count = 0;     //  <<<<<<<<<<<<<<<<<<<<<

Timer _timer;

I2CTransfer _subProc1(ADDRESS_SUBPROC_1);
I2CTransfer _subProc3(ADDRESS_SUBPROC_3);

bool _ledsOnOff = FALSE;     //  <<<<<<<<<<<<<<<<<<<<<

char _timestamp[24];     //  <<<<<<<<<<<<<<<<<<<<<

bool _warmupPeriodOver = FALSE;     //  <<<<<<<<<<<<<<<<<<<<<

Timer _telemetry_timer;


UartStream* _serialStream_Gps1;              // ***
UartStream* _serialStream_subProc2;

UartStream* _serialStream_Gps2;
UartStream* _serialStream_Radio;


//UartStream _serialStream_Gps1('3', process_gps_data1, FALSE, "GPS 1");              // ***
//UartStream _serialStream_subProc2('0', process_proc2_data, FALSE, "MicroController 2");
//
//UartStream _serialStream_Gps2('1', process_gps_data2, FALSE, "GPS 2");
//UartStream _serialStream_Radio('2', process_radio_data, FALSE, "Radio Modem");


GpsData* _gps1_data = NULL;
GpsData* _gps2_data = NULL; 

AltitudeCalc _altitude_calculation;

int _pictureCount = 0;     
int _balloonSwitchValue = 0;     
int _parachuteSwitchValue = 0;  
int _safetySwitchValue = 0;  
int _safety_switch_was;

double _gpsLat1 = 0.0;
double _gpsLat2 = 0.0;

bool _gps_data1_attached = FALSE;
bool _gps_data2_attached = FALSE;

char _gpsDataString1[GPS_DATA_BUFFER_LEN];    
char _gpsDataString2[GPS_DATA_BUFFER_LEN];     
char _lastGpsDataString[GPS_DATA_BUFFER_LEN];
char _craftMessage[TELEMETRY_DATA_LEN];
char _radioMessage[TELEMETRY_DATA_LEN];   

char _proc2_dataArray[PROC2_DATA_COUNT][DATA_ARRAY_STR_LEN];
char _proc1_dataArray[PROC1_DATA_COUNT][DATA_ARRAY_STR_LEN];

const int _smoothingReadingCount = 5;
double _smoothingReadings[_smoothingReadingCount];      // the readings from the analog input
int _smoothingReadIndex = 0;                            // the index of the current reading
double _smoothingTotal = 0;                             // the running total
double _smoothingAverage = 0;                           // the average

bool _videoCameraUp = FALSE;

uint8_t messagesDisplayed[5] = { 0, 0, 0, 0, 0 };

uint8_t _craft_notes_flags[CRAFT_NOTE_COUNT] = { 
    CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, 
    CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, 
    CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, CRAFT_MESSAGE_NOT_SENT, 
    CRAFT_MESSAGE_MULTISEND, CRAFT_MESSAGE_MULTISEND, CRAFT_MESSAGE_MULTISEND, CRAFT_MESSAGE_MULTISEND, 
    CRAFT_MESSAGE_NOT_SENT };

char _craft_notes[CRAFT_NOTE_COUNT][CRAFT_NOTE_TEXT_LENGTH] = {
    "}N:PII - Liftoff",
    "}N:PII - Safe Mode",
    "}N:PII - Above Air Traffic",
    "}N:PII - Curvature Seen",
    "}N:PII - Goal Altitude Reached!",
    "}N:PII - Balloon Released-Falling!",
    "}N:PII - Parachute Deployed",
    "}N:PII - I can see my house!",
    "}N:PII - Reached Stratosphere!",
    "}N:PII - Diving over 200MPH!",
    "}N:PII - Plummeting over 250MPH!",
    "}N:PII - Screaming over 300MPH!",
    "}N:PII - Minutes to Autorelease: %d",
    "}N:PII - Autodeploy Altitude: %d",
    "}N:PII - UFO Lights On",
    "}N:PII - UFO Lights Off",
    "}N:PII - Craft Armed"
};



double _initialAltitude = 0.0;     //  <<<<<<<<<<<<<<<<<<<<<
double _current_altitude = 0.0;
double _vertical_speed = 0.0;

int _minutes_till_release = (DEFAULT_SECONDS_TILL_RELEASE / 60);
int _seconds_till_release = DEFAULT_SECONDS_TILL_RELEASE;
int _altitude_for_deploy = DEFAULT_ALTITUDE_FOR_DEPLOY;
//bool _craft_armed = FALSE;

time_t _initial_time;
time_t _current_time;

bool _balloon_released = FALSE;

int _telemetry_count = 0;

MessageValidation _messageValidator;


char _proc1SensorBuffer[GENERAL_BUFFER_LEN];

char _telemetry_data_string[TELEMETRY_DATA_LEN];
char _tmp_log_data[GENERAL_BUFFER_LEN];

typedef struct {
    double lat;
    double lon;
    double alt;
} latLonAlt;



#if (TEST_TELEMETRY)


typedef struct {
    double altitude_meters;
    double speed_meters_per_second;
} acceleration_entry;

int _altitude_table_size = 0;

acceleration_entry _altitude_table[350];

latLonAlt _balloon_starting_position;
latLonAlt _balloon_ending_position;
latLonAlt _balloon_current_position;

double _balloon_change_lat;
double _balloon_change_lon;

enum TestState
{
    BALLOON_PREP,
    BALLOON_RISING,
    BALLOON_RELEASED,
    BALLOON_FALLING,
    PARACHUTE_DEPLOYED
};

TestState _balloon_state;

int _altitude_index;
double _speed_increment;

uint8_t _balloon_falling_counter = 0;

double _ground_altitude = 0;


FILE * fp;

char* _test_buff;
size_t len = 0;

char _tmp_telemetry_value_list[39][DATA_ARRAY_STR_LEN];

size_t lineLength;
char* line;

FILE* _altitude_data_fp;

int _testStepCounter = 0;
int _testRiseCount;

int get_altitude_index(double altitude);
void TestStep();

#endif


void debug_print(char* msg) {

#if (DEBUG_PRINT) 
    printf("%s\n", msg);
#endif
}



void display_switch_values()
{
    printf("Safety Pin: %s  --  ", _safetySwitchValue ? "REMOVED" : "Inserted");
    printf("Balloon Pin: %s  --  ", _balloonSwitchValue ? "REMOVED" : "Inserted");
    printf("Parachute Pin: %s\n", _parachuteSwitchValue ? "REMOVED" : "Inserted");
}



void safety_switch_check() {

    _safetySwitchValue = digitalRead(GPIO_PIN_SAFETY);
    int parachute_and_balloon = _balloonSwitchValue & _parachuteSwitchValue;

    if (_safetySwitchValue != _safety_switch_was) {
        sprintf(_tmp_log_data, "SafetySwitch: %d", _safetySwitchValue);
        write_to_log("SSW", _tmp_log_data);
    }

    display_switch_values();
    
    // balloon and parachute pins removed and safety pin newly inserted -> shutdown
    //                                   1                                                     1                                             0
    if ((parachute_and_balloon == PIN_REMOVED_INDICATED) && (_safety_switch_was == PIN_REMOVED_INDICATED) && (_safetySwitchValue == PIN_INSERTED_INDICATED)) {
        printf("Initiate Shutdown!\n");

        system("sudo shutdown -h now");
    }
    else {
        // balloon or para inserted and safety pin newly inserted
        //                                       0                                                 0                                               1
        if ((parachute_and_balloon == PIN_INSERTED_INDICATED) && (_safetySwitchValue == PIN_INSERTED_INDICATED) && (_safety_switch_was == PIN_REMOVED_INDICATED)) {
            printf("Turn LED's Off!\n");

            switch_leds(FALSE);
        }
        else { //                                    0                                                1                                                0
            if ((parachute_and_balloon == PIN_INSERTED_INDICATED) && (_safetySwitchValue == PIN_REMOVED_INDICATED) && (_safety_switch_was == PIN_INSERTED_INDICATED)) {
                printf("Turn LED's On!\n");

                switch_leds(TRUE);
            }
        }
    }

    _safety_switch_was = _safetySwitchValue;
}



/**
* \fn evaluate_data
* \brief evaluate current telemetry to determine if an action
*   should be taken such as sending a craft message,
*   releasing the balloon, or deploying the main parachute 
*/
void evaluate_data() {

    static uint8_t fall_counter = 0;
    static uint8_t parachute_deploy_attempts = 0;

    if (_warmupPeriodOver) {

        if ((_initialAltitude == 0.0) && (_current_altitude != 0.0)) {
            _initialAltitude = _current_altitude;
        }

        if (_craft_notes_flags[LIFTOFF_POS] == CRAFT_MESSAGE_NOT_SENT) {
            double altDif = _current_altitude - _initialAltitude;

            // Start the timer
            _initial_time = _current_time;

            if (altDif > 20.0) {
                send_craft_message(LIFTOFF_POS, MESSAGE_NO_VALUE);
                _subProc3.send_command(PROC3_COMMAND_GOING_UP);
            }
        }

        //if (_craft_notes_flags[SAFE_MODE_POS] == CRAFT_MESSAGE_NOT_SENT) {
        //    double altDif = _current_altitude - _initialAltitude;

        //    if (altDif > 200) {
        //        send_craft_message(SAFE_MODE_POS, MESSAGE_NO_VALUE);
        //    }
        //}

        if ((_craft_notes_flags[ABOVE_TRAFFIC_POS] == CRAFT_MESSAGE_NOT_SENT) && (_current_altitude > ALTITUDE_ABOVE_TRAFFIC)) {
            send_craft_message(ABOVE_TRAFFIC_POS, MESSAGE_NO_VALUE);
            _subProc3.send_command(PROC3_COMMAND_ABOVE_TRAFFIC);
        }

        if ((_craft_notes_flags[STRATOSPHERE_POS] == CRAFT_MESSAGE_NOT_SENT) && (_current_altitude > ALTITUDE_STRATOSPHERE)) {
            send_craft_message(STRATOSPHERE_POS, MESSAGE_NO_VALUE);
            _subProc3.send_command(PROC3_COMMAND_STRATOSPHERE);
        }

        if ((_craft_notes_flags[SEE_MY_HOUSE_POS] == CRAFT_MESSAGE_NOT_SENT) && (_current_altitude > ALTITUDE_I_CAN_SEE)) {
            send_craft_message(SEE_MY_HOUSE_POS, MESSAGE_NO_VALUE);
        }

        if ((_craft_notes_flags[CURVATURE_POS] == CRAFT_MESSAGE_NOT_SENT) && (_current_altitude > ALTITUDE_CURVATURE)) {
            send_craft_message(CURVATURE_POS, MESSAGE_NO_VALUE);
        }

        if ((_craft_notes_flags[GOAL_ALTITUDE_POS] == CRAFT_MESSAGE_NOT_SENT) && (_current_altitude >= ALTITUDE_GOAL)) {
            send_craft_message(GOAL_ALTITUDE_POS, MESSAGE_NO_VALUE);
            _subProc3.send_command(PROC3_COMMAND_REACHED_GOAL);
        }

        if ((messagesDisplayed[0] == 0) && (_current_altitude > 20000)) {
            debug_print("evaluate_data: altitude above 20000 sending cmd 3");
            messagesDisplayed[0] = 1;
            _subProc3.send_command(PROC3_COMMAND_THANK_MSR);
        }

        if ((messagesDisplayed[1] == 0) && (_current_altitude > 22000)) {
            debug_print("evaluate_data: altitude above 22000 sending cmd 4");
            messagesDisplayed[1] = 1;
            _subProc3.send_command(PROC3_COMMAND_EARTH_PEOPLE);
        }

        if ((messagesDisplayed[2] == 0) && (_current_altitude > 24000)) {
            debug_print("evaluate_data: altitude above 24000 sending cmd 5");
            messagesDisplayed[2] = 1;
            _subProc3.send_command(PROC3_COMMAND_FLIGHT_LEADS);
        }

        if ((messagesDisplayed[3] == 0) && (_current_altitude > 26000)) {
            debug_print("evaluate_data: altitude above 26000 sending cmd 6");

            messagesDisplayed[3] = 1;
            _subProc3.send_command(PROC3_COMMAND_HIMON);
        }

        //if ((messagesDisplayed[4] == 0) && (_current_altitude > 28000)) {
        //    messagesDisplayed[4] = 1;
        //    _subProc3.send_command(7);
        //}


        //if ((_craft_armed == FALSE) && (_vertical_speed > 0) && (_current_altitude > (_initialAltitude + DEFAULT_ALTITUDE_FOR_ARMING))) {
        //    _craft_armed = TRUE;
        //    send_craft_message(CRAFT_ARMED_POS, MESSAGE_NO_VALUE);
        //}

        if ((_vertical_speed <= FALL_LIMIT_SPEED) && (_current_altitude <= (_initialAltitude + _altitude_for_deploy))) {

            if (fall_counter++ >= 3) {
                if ((_balloon_released == FALSE) || (_balloonSwitchValue != PIN_REMOVED_INDICATED))
                {
                    debug_print("evaluate_data: falling, release balloon now!");
                    release_balloon_now();
                }

                if (parachute_deploy_attempts++ < 3) {
                    debug_print("evaluate_data: falling, deploy parachute now!");
                    deploy_parachute_now();
                }
            }
        }

        if ((_current_time - _initial_time) >= _seconds_till_release) {
            if (_balloon_released == FALSE) {
                debug_print("evaluate_data: time limit relase balloon now!");
                release_balloon_now();
            }
        }
    }
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


bool starts_with(char* stringToCheck, char* startsWith)
{
    if ((stringToCheck == NULL) || (startsWith == NULL)) return FALSE;
    
    int stringToCheckLen = strlen(stringToCheck);
    int startsWithLen = strlen(startsWith);
    
    if ((stringToCheckLen  == 0) || (startsWithLen == 0)) return FALSE;
    if (stringToCheckLen < startsWithLen) return FALSE;
    
    for (int i = 0; i < startsWithLen; i++)
    {
        if (stringToCheck[i] != startsWith[i]) return FALSE;
    }
    
    return TRUE;
}


/**
* \fn process_gps_data1
* \brief convert the incoming raw GPS data to a usable format
*/
void process_gps_data1(char* dataLine) {

    if ((dataLine == NULL) || (!starts_with(dataLine, "$GPRMC") && !starts_with(dataLine, "$GPGGA"))) return;
    
    write_to_log("GP1", dataLine);
    
    _gps_data1_attached = FALSE;
    
    _gps1_data = _serialStream_Gps1->parseGps(dataLine);

    if (_gps1_data) {

        _gps_data1_attached = TRUE;

        double speed = _gps1_data->getSpeed();

        speed = do_speed_conversion(speed);

        if (speed < 1.0) {
            speed = 0.1;
        }

        _gpsLat1 = _gps1_data->getLatitude();

        sprintf(_gpsDataString1,
            "%7.5f,%7.5f,%3.1f,%3.1f,%3.1f,%d,%d",
            _gpsLat1,
            _gps1_data->getLongitude(),
            _gps1_data->getAltitude(),
            speed,
            _gps1_data->getDirection(),
            _gps1_data->getFix(),
            _gps1_data->getSatellites());
    }
}


/**
* \fn process_gps_data2
* \brief convert the incoming raw GPS data to a usable format
*/
void process_gps_data2(char* dataLine) {

    if ((dataLine == NULL) || (!starts_with(dataLine, "$GPRMC") && !starts_with(dataLine, "$GPGGA"))) return;
    
    write_to_log("GP2", dataLine);

    _gps2_data = _serialStream_Gps2->parseGps(dataLine);
    
    if (_gps2_data) {
        _gps_data2_attached = TRUE;

        double speed = _gps2_data->getSpeed();

        speed = do_speed_conversion(speed);

        if (speed < 1.0) {
            speed = 0.2;
        }

        _gpsLat2 = _gps2_data->getLatitude();

        sprintf(_gpsDataString2, "%7.5f,%7.5f,%3.1f,%3.1f,%3.1f,%d,%d",
            _gpsLat2,
            _gps2_data->getLongitude(),
            _gps2_data->getAltitude(),
            speed,
            _gps2_data->getDirection(),
            _gps2_data->getFix(),
            _gps2_data->getSatellites());
    }
}



/**
* \fn process_radio_data
* \brief Process incoming radio data (commands)
*/
void process_radio_data(char* dataLine) {

    write_to_log("RD1", dataLine);

    int len = strlen(dataLine);

    if (dataLine[len - 1] == '\n') {
        dataLine[len - 1] = 0;
    }

    int checksumPos = _messageValidator.validateMessage(dataLine);

    if ((checksumPos != -1) && (checksumPos < 256)) {
        // Valid Message
        strcpy(_radioMessage, dataLine);

        _radioMessage[checksumPos] = 0;

        processCommand(_radioMessage);
    }
}



/**
* \fn smooth
* \brief Performs a averaging of data to smooth the results over time
* \return smoothed data
*/
double smooth(double dataValue){
    // subtract the last reading:
    _smoothingTotal = _smoothingTotal - _smoothingReadings[_smoothingReadIndex];
    // read from the sensor:
    _smoothingReadings[_smoothingReadIndex] = dataValue;
    // add the reading to the total:
    _smoothingTotal = _smoothingTotal + _smoothingReadings[_smoothingReadIndex];
    // advance to the next position in the array:
    _smoothingReadIndex = _smoothingReadIndex + 1;

    // if we're at the end of the array...
    if (_smoothingReadIndex >= _smoothingReadingCount) {
        // ...wrap around to the beginning:
        _smoothingReadIndex = 0;
    }

    // calculate the average:
    _smoothingAverage = _smoothingTotal / _smoothingReadingCount;

    return _smoothingAverage;
}



/**
* \fn trim
* \brief removes white space at the beginning and end of a character string
* \return pointer to the trimmed string
*/
char *trim(char *s)
{
    if (s == NULL) { return NULL; }

    int len = strlen(s);

    if (len == 0) { return NULL; }

    while (*s && isspace(*s)) {
        s++;
    }

    len = strlen(s);

    if (len == 0) { return NULL; }

    char* back = s + (len - 1);

    if (isspace(*back)) {
        while (*back && isspace(*--back));

        *(back + 1) = '\0';
    }

    return s;
}


/**
* \fn findChar
* \brief Finds a character in a larger string
* \return Position of the character in the string
*/
int findChar(char *dataIn, char charToFind, int startChar) {

    if (dataIn == NULL) return -1;
    if (startChar < 0) return -1;

    int currentChar = startChar;

    do {
        if ((dataIn[currentChar] == charToFind) || (dataIn[currentChar] == 0)) {
            return currentChar;
        }

        currentChar++;

    } while (dataIn[currentChar] != 0);

    return -1;
}





int strsplit(const char* str, const char* delim, char dataArray[][DATA_ARRAY_STR_LEN]) {
    // copy the original string so that we don't overwrite parts of it
    // (don't do this if you don't need to keep the old line,
    // as this is less efficient)
    char *s = strdup(str);

        // these three variables are part of a very common idiom to
        // implement a dynamically-growing array
    size_t tokens_alloc = 1;
    size_t tokens_used = 0;
    
    char **tokens = (char**) calloc(tokens_alloc, sizeof(char*));

    char *token, *strtok_ctx;
    for (token = strtok_r(s, delim, &strtok_ctx); token != NULL; token = strtok_r(NULL, delim, &strtok_ctx)) 
    {
        // check if we need to allocate more space for tokens
        if (tokens_used == tokens_alloc) {
            tokens_alloc *= 2;
            tokens = (char**) realloc(tokens, tokens_alloc * sizeof(char*));
        }
        
        tokens[tokens_used++] = strdup(token);
    }

        // cleanup
    if (tokens_used == 0) {
        free(tokens);
        tokens = NULL;
    
    }
    else {
        tokens = (char**) realloc(tokens, tokens_used * sizeof(char*));
    }
    
    free(s);
    
    for (size_t i = 0; i < tokens_used; i++) 
    {
        strcpy(dataArray[i], tokens[i]);
        
        //printf("    token: \"%s\"\n", tokens[i]);
        free(tokens[i]);
    }
    
    if (tokens != NULL)
    {
        free(tokens);
    }

    return tokens_used;
}


///**
//* \fn splitDataIntoArray
//* \brief Splits a character into a array based on a comma delimiter
//* \return Number of data components in the resulting array
//*/
//int splitDataIntoArray(char *dataIn, char dataArray [][DATA_ARRAY_STR_LEN], int arraySize) {
//
    //int pos = -1;
    //int currentPos = 0;
    ////char tmp[256];
//
    ////strcpy(tmp, dataIn);
//
    //for (int i = 0; i < arraySize; i++) {
        //pos = findChar(dataIn, ',', currentPos);
//
        //if (pos > 0) {
            //dataIn[pos] = 0;
//
            //int len = strlen(&dataIn[currentPos]);
//
            //if (len > 0) {
                //strcpy(dataArray[i], trim(&dataIn[currentPos]));
            //}
            //else {
                //dataArray[i][0] = 0;
            //}
//
            //currentPos = pos + 1;
        //}
//
        //if (pos == -1) {
//
            //int len = strlen(&dataIn[currentPos]);
//
            //if (len > 0) {
                //strcpy(dataArray[i], trim(&dataIn[currentPos]));
            //}
//
            //return(i + 1);
            //break;
        //}
    //}
//
    //return arraySize + 1;
//}

/*

sprintf(sensorData, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
     0   dtostrf(pressure_abs, 3, 1, scratch01),
     1   dtostrf(pressure_temperature_c, 3, 1, scratch02),  
     2   dtostrf(humidity, 3, 1, scratch03),
     3   dtostrf(thermocouple_temp_c, 3, 1, scratch04),
     4   dtostrf(lsm.accelData.x, 3, 1, scratch05),
     5   dtostrf(lsm.accelData.y, 3, 1, scratch06),
     6   dtostrf(lsm.accelData.z, 3, 1, scratch07),
     7   dtostrf(lsm.gyroData.x, 3, 1, scratch08),
     8   dtostrf(lsm.gyroData.y, 3, 1, scratch09),
     9   dtostrf(lsm.gyroData.z, 3, 1, scratch10),
    10   dtostrf(lsm.magData.x, 3, 1, scratch11),
    11   dtostrf(lsm.magData.y, 3, 1, scratch12),
    12   dtostrf(lsm.magData.z, 3, 1, scratch13),
    13   dtostrf(temperatureC, 3, 1, scratch14)

*/

/**
* \fn process_proc2_data
* \brief Process incoming MicroController #2 data (sensor telemetry)
*/
void process_proc2_data(char* dataLine) {

    char dataToLog[256];
    
    strcpy(dataToLog, dataLine);
    
    int splitCount = strsplit(dataLine, ",", _proc2_dataArray);
    //int splitCount = splitDataIntoArray(dataLine, _proc2_dataArray, PROC2_DATA_COUNT);

    if (splitCount == PROC2_DATA_COUNT) {

        double _air_pressure = atof(_proc2_dataArray[PROC2_DATA_POS_AIR_PRESSURE]);

        _altitude_calculation.CalculateAltitude(_air_pressure);
    }
 
    write_to_log("MC2", dataToLog);

}


/**
* \fn get_subproc1_sensor_data
* \brief Gets the sensor data from the #1 MicroController using I2C
* \return Pointer to the character string with the comman delimited sensor data
*/
char* get_subproc1_sensor_data() {

    static uint8_t counter = 0;

    uint8_t packetInfoBuffer[4];

    counter++;

    if (counter >= 5) {
        // Do this just as a safety - will reset the data retrieval in proc 3
        _subProc1.send_command(PROC1_COMMAND_RESET_DATA);
        counter = 0;
    }
    
    memset(packetInfoBuffer, 0, 4);
    

    _subProc1.get_i2c_data_packet(packetInfoBuffer, 3);

    int i;
    for (i = 0; i < packetInfoBuffer[1]; i++) {
        _subProc1.get_i2c_string(&_proc1SensorBuffer[i * PACKET_LENGTH], PACKET_LENGTH);
    }

    if (packetInfoBuffer[2]) {
        _subProc1.get_i2c_string(&_proc1SensorBuffer[packetInfoBuffer[1] * PACKET_LENGTH], packetInfoBuffer[2]);
    }

    _proc1SensorBuffer[packetInfoBuffer[0]] = '\n';
    _proc1SensorBuffer[packetInfoBuffer[0]+1] = '\0';

    write_to_log("MC1", _proc1SensorBuffer);
    
    _proc1SensorBuffer[packetInfoBuffer[0]] = '\0';

    return _proc1SensorBuffer;
}


int file_exists(char* filename)
{
    if (filename == NULL) return -1;
    
    struct stat sts;
        
    if (stat(filename, &sts) == -1)
    {
        return 0;
    }
    
    return 1;
}

void initialize_log(char* filename)
{
    FILE* fd;
    
    if (filename == NULL) {
    
        printf("\n>>>>> Tried to initialize the log file but the filename was null\n\n");
        
        return;
    }
    
    if (!file_exists(filename))
    {
        ssize_t nrd;
        
        fd = fopen(filename, "w");

        if (fd != NULL)
        {
            fprintf(fd, "INIT,Logfile Initialized,INIT\n");
    
            fflush(fd);
            
            fclose(fd);
        }
    }
    else
    {
        fd = fopen(TELEMETRY_LOG_FILE, "a");
    
        if (fd == NULL) {
            printf("Couldn't open logfile for appending.\n");
        
            return;
        }
        
        fprintf(fd, "INIT,Logfile Initialized,INIT\n");

    
        fflush(fd);
    
        /* close the file */
        fclose(fd);
    }
    
}

void write_to_log(const char* source, char* data) {

    char timestamp[36];
    char telemetry_string[256];

    FILE* fp;
    
    if ((source == NULL) || (data == NULL)) return;
    
    /* open the file for append */
    fp = fopen(TELEMETRY_LOG_FILE, "a");
    
    if (fp == NULL) {
        printf("I couldn't open results.dat for appending.\n");
        
        return;
    }

    format_timestamp_with_milliseconds(timestamp);

    fprintf(fp, "%s,%s,%s", source, timestamp, data);

    fflush(fp);
    
    /* close the file */
    fclose(fp);
}


/**
* \fn switch_leds
* \brief Turn LED Signal lights on or off
*/
void get_m1_data() {

    get_subproc1_sensor_data();

    //int splitCount = splitDataIntoArray(_proc1SensorBuffer, _proc1_dataArray, PROC2_DATA_COUNT);
    int splitCount = strsplit(_proc1SensorBuffer, ",", _proc1_dataArray);
}


/**
* \fn switch_leds
* \brief Turn LED Signal lights on or off
*/
void switch_leds(bool onOff) {
    _ledsOnOff = onOff;

    if (onOff) {
        _subProc3.send_command(PROC3_COMMAND_LEDS_ON);

        send_craft_message(LIGHTS_ON_POS, MESSAGE_NO_VALUE);
    }
    else {
        _subProc3.send_command(PROC3_COMMAND_LEDS_OFF);

        send_craft_message(LIGHTS_OFF_POS, MESSAGE_NO_VALUE);
    }
}


//double myfabs(double value)
//{
    //if (value < 0.0) return (value * -1.0);
//
    //return value;
//}
//
/**
* \fn calculate_vertical_speed
* \brief Calculate vertical speed (ascending:+ or descending:-) 
*
* Conversion:
*  44.7 == 100 mph
*  89.4 == 200 mph  Diving
* 111.8 == 250 mph  Plummeting
* 134.1 == 300 mph  Screaming
* \return Speed in meters per second
*/
double calculate_vertical_speed(double altitude, double seconds) {
    
    static double previousAltitude = 0.0;

    double mps = (altitude - previousAltitude) / seconds;
    
    //double abs_mps = myfabs(mps);

    //printf("VS: alt: %.1f  prev: %.1f  sec: %.1f  mps: %.1f\n", altitude, previousAltitude, seconds, abs_mps);

    previousAltitude = altitude;

    if (_warmupPeriodOver) {
        if ((_craft_notes_flags[DIVING_POS] == 0) && (mps <= -89.4)) {
            send_craft_message(DIVING_POS, MESSAGE_NO_VALUE);
        }
        else {
            if ((_craft_notes_flags[PLUMMETING_POS] == 0) && (mps <= -111.8)) {
                _subProc3.send_command(PROC3_COMMAND_SPEED_200);
                send_craft_message(PLUMMETING_POS, MESSAGE_NO_VALUE);
            }
            else {
                if ((_craft_notes_flags[SCREAMING_POS] == 0) && (mps <= -134.1)) {
                    _subProc3.send_command(PROC3_COMMAND_SPEED_300);
                    send_craft_message(SCREAMING_POS, MESSAGE_NO_VALUE);
                }

            }
        }
    }

    return mps;
}



/**
* \fn send_telemetry
* \brief send telemetry to ground stations
*/
void send_telemetry() {

    char* gpsDataPtr;
    char geiger[12];

    double pressure = atof(_proc2_dataArray[PROC2_DATA_POS_AIR_PRESSURE]);

#if (!TEST_TELEMETRY)

    if (_gpsLat1 == 0.0) {
        if (_gpsLat2 == 0.0) {
            gpsDataPtr = _lastGpsDataString;
        }
        else {
            gpsDataPtr = _gpsDataString2;
        }
    }
    else {
        gpsDataPtr = _gpsDataString1;
    }
#else 
    
    TestStep();
    
#endif

    _balloonSwitchValue = !digitalRead(GPIO_PIN_BALLOON);
    _parachuteSwitchValue = !digitalRead(GPIO_PIN_PARACHUTE);
    
#if (TEST_TELEMETRY)

    switch (_balloon_state) {

        case BALLOON_PREP:

            if (_telemetry_sent_count > 10) {
                _balloon_state = BALLOON_RISING;
            }
            break;
        
        case BALLOON_RISING:
        _current_altitude += ((DEFAULT_ASCENT_RATE * 2) * TEST_ASCENT_RATE_MULTIPLIER );

            _balloon_current_position.lat += _balloon_change_lat;
            _balloon_current_position.lon += _balloon_change_lon;
            break;

        case BALLOON_RELEASED:
            
            _balloon_falling_counter++;
            _current_altitude = _current_altitude - (_speed_increment * _balloon_falling_counter);

            if (_balloon_falling_counter == 5) {
                _balloon_state = BALLOON_FALLING;

                _altitude_index = get_altitude_index(_current_altitude) + 1;
            }

            _balloon_current_position.lat += _balloon_change_lat;
            _balloon_current_position.lon += _balloon_change_lon;
            break;

        case BALLOON_FALLING:

             if ((_altitude_index < _altitude_table_size) && (_current_altitude > _balloon_ending_position.alt)) {
                _current_altitude = _altitude_table[_altitude_index++].altitude_meters;

                if (_current_altitude < _ground_altitude) {
                    _current_altitude = _ground_altitude;

                }
                else {
                    _balloon_current_position.lat += _balloon_change_lat;
                    _balloon_current_position.lon += _balloon_change_lon;
                }
            }

            break;

        case PARACHUTE_DEPLOYED:

            _current_altitude -= DEFAULT_PARACHUTE_DECENT_RATE;

            if (_current_altitude < _ground_altitude) {
                _current_altitude = _ground_altitude;

            }
            else {
                _balloon_current_position.lat += _balloon_change_lat;
                _balloon_current_position.lon += _balloon_change_lon;
            }

            break;
    }


    sprintf(_gpsDataString1, "%7.5f,%7.5f,%3.1f,%3.1f,%3.1f,%d,%d",
        _balloon_current_position.lat,
        _balloon_current_position.lon,
        _current_altitude,
        8.2,
        92.6,
        TRUE,
        6);

    gpsDataPtr = _gpsDataString1;
    
#else

    if (pressure > 0.0) {
        _current_altitude = _altitude_calculation.CalculateAltitude(pressure);
    }
    else {

        if (_gps_data1_attached) {
            _current_altitude = _gps1_data->getAltitude();

            if (_current_altitude <= 0.0) {
                if (_gps_data2_attached) {
                    _current_altitude = _gps2_data->getAltitude();
                }
            }
        }
        else {
            if (_gps_data2_attached) {
                _current_altitude = _gps2_data->getAltitude();
            }
        }
    }

#endif


    //_smoothed_altitude = smooth(_current_altitude);
    //_smoothed_altitude = _current_altitude;

    _telemetry_sent_count++;

    _vertical_speed = calculate_vertical_speed(_current_altitude, 2.0);

    double voltageMain = atof(_proc1_dataArray[PROC1_DATA_POS_BATTERY_MAIN]) / 10.0;
    double voltageAux = atof(_proc1_dataArray[PROC1_DATA_POS_BATTERY_AUX]) / 10.0;
    // tempIn = atof(_proc1_dataArray[PROC1_DATA_POS_TEMP_IN]) / 10.0;
    //double tempOut = atof(_proc1_dataArray[PROC1_DATA_POS_TEMP_OUT]) / 10.0;

    if (strlen(_proc1_dataArray[PROC1_DATA_POS_GEIGER_CPS]) == 0) {
        strcpy(geiger, "0");
    }
    else {
        strcpy(geiger, _proc1_dataArray[PROC1_DATA_POS_GEIGER_CPS]);
    }
        
    double pressureTemp = atof(_proc2_dataArray[PROC2_DATA_POS_PRESSURE_TEMP]);
    double humidity = atof(_proc2_dataArray[PROC2_DATA_POS_HUMIDITY]);

    double accelx = atof(_proc2_dataArray[PROC2_DATA_POS_ACCEL_X]);
    double accely = atof(_proc2_dataArray[PROC2_DATA_POS_ACCEL_Y]);
    double accelz = atof(_proc2_dataArray[PROC2_DATA_POS_ACCEL_Z]);
    double gyrox = atof(_proc2_dataArray[PROC2_DATA_POS_GYRO_X]);
    double gyroy = atof(_proc2_dataArray[PROC2_DATA_POS_GYRO_Y]);
    double gyroz = atof(_proc2_dataArray[PROC2_DATA_POS_GYRO_Z]);
    double magx = atof(_proc2_dataArray[PROC2_DATA_POS_MAG_X]);
    double magy = atof(_proc2_dataArray[PROC2_DATA_POS_MAG_Y]);
    double magz = atof(_proc2_dataArray[PROC2_DATA_POS_MAG_Z]);
    double tmp36 = atof(_proc2_dataArray[PROC2_DATA_TMP36]);
    double thermoCouple = atof(_proc2_dataArray[PROC2_DATA_POS_THERMOCOUPLE_TEMP]);
    

    double uv = atof(_proc1_dataArray[PROC1_DATA_POS_UV_LEVEL]);
    int balloonRelay = atoi(_proc1_dataArray[PROC1_DATA_POS_BALLOON_RELAY]);
    int auxRelay = atoi(_proc1_dataArray[PROC1_DATA_POS_AUX_RELAY]);

/*

sprintf(sensorData, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
     0   dtostrf(pressure_abs, 3, 1, scratch01),
     1   dtostrf(pressure_temperature_c, 3, 1, scratch02),  
     2   dtostrf(humidity, 3, 1, scratch03),
     3   dtostrf(thermocouple_temp_c, 3, 1, scratch04),
     4   dtostrf(lsm.accelData.x, 3, 1, scratch05),
     5   dtostrf(lsm.accelData.y, 3, 1, scratch06),
     6   dtostrf(lsm.accelData.z, 3, 1, scratch07),
     7   dtostrf(lsm.gyroData.x, 3, 1, scratch08),
     8   dtostrf(lsm.gyroData.y, 3, 1, scratch09),
     9   dtostrf(lsm.gyroData.z, 3, 1, scratch10),
    10   dtostrf(lsm.magData.x, 3, 1, scratch11),
    11   dtostrf(lsm.magData.y, 3, 1, scratch12),
    12   dtostrf(lsm.magData.z, 3, 1, scratch13),
    13   dtostrf(temperatureC, 3, 1, scratch14)

*/
    //                                                        |                      |                  |                        |            |  |                  |
    sprintf(_telemetry_data_string, "$:%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%s,%d,%.1f,%d,%.1f,%d,%d,%d,%d,%d",
        format_timestamp(_timestamp),                                               //  1 - timestamp
        pressure,                                                                   //  2 - air pressure
        _current_altitude,                                                          //  3 - altitude
        pressureTemp,                                                               //  4 - pressure temp
        humidity,                                                                   //  5 - humidity

        tmp36,                                                                      //  6 - temp in
        thermoCouple,                                                               //  7 - temp out
        voltageMain,                                                                //  8 - main battery voltage
        voltageAux,                                                                 //  9 - aux battery voltage
        _balloonSwitchValue,                                                        // 10 - balloon switch

        _parachuteSwitchValue,                                                      // 11 - parachute switch
        geiger,                                                                     // 12 - geiger counter cps
        _videoCameraUp,                                                             // 13 - camera up position
        accelx,                                                                     // 14 - accelerometer X
        accely,                                                                     // 15 - accelerometer Y

        accelz,                                                                     // 16 - accelerometer Z
        gyrox,                                                                      // 17 - gyroscope X
        gyroy,                                                                      // 18 - gyroscope Y
        gyroz,                                                                      // 19 - gyroscope Z
        magx,                                                                       // 20 - magnetometer X

        magy,                                                                       // 21 - magnetometer Y
        magz,                                                                       // 22 - magnetometer Z
        "0.0",                                                                      // 23 - radio strength  ****
        
        gpsDataPtr,                                                                 // 24-30 - GPS Data
                                                                                     // 24: latitude
                                                                                        // 25: longitude
                                                                                        // 26: altitude
                                                                                        // 27: speed
                                                                                        // 28: direction
                                                                                        // 29: fix
                                                                                        // 30: satelites
        _messageValidator.getReceptionErrorCount(),                                 // 31 - reception errors  ****
        _vertical_speed,                                                            // 32 - vertical speed
        _pictureCount,                                                              // 33 - picture count
        uv,                                                                         // 34 - UV rays
        _ledsOnOff,                                                                 // 35 - leds activated
        
        balloonRelay,                                                               // 36 - balloon/parachute relay on/off
        auxRelay,                                                                   // 37 - aux relay (camera) on/off
        _altitude_for_deploy,                                                       // 38 - altitude to auto-deploy the parachute (when falling)
        _minutes_till_release                                                       // 39 - minutes until auto-release (after launch)
        );

    
    
    //if (_gps_data1_attached) {
        //printf("GPS 1 Attached - ");
    //}
//
    //if (_gps_data2_attached) {
        //printf("GPS 2 Attached - ");
    //}

    evaluate_data();

    safety_switch_check();

    _messageValidator.appendCheckSum(_telemetry_data_string);

    strcat(_telemetry_data_string, "\n");

    //printf("Telem: %s\n", _telemetry_data_string);
    printf("TelemetryCount: %d\n", ++_telemetry_count);

    printf("air: %f | alt: %f | PresTmp: %f | Humidity: %f | Tmp36: %f | Thermo: %f | ReceptErrs: %d\n", 
        pressure,
        _current_altitude,
        pressureTemp,
        humidity,
        tmp36,
        thermoCouple,
        _messageValidator.getReceptionErrorCount());
    printf("VMain: %f | VAux: %f\n", voltageMain, voltageAux);
    printf("Geiger: %s | UV: %f | PicCount: %d | LedsOn: %d | AltDeploy: %d | MinToRelease: %d\n", geiger, uv, _pictureCount, _ledsOnOff, _altitude_for_deploy, _minutes_till_release);
    
    char gps1Attached[2];
    if (_gps_data1_attached) { strcpy(gps1Attached, "Y"); } else { strcpy(gps1Attached, "N"); }
    char gps2Attached[2];
    if (_gps_data2_attached) { strcpy(gps2Attached, "Y"); } else { strcpy(gps2Attached, "N"); }
    
    printf("GPS: %s | VertSpeed: %f | GPS1Attach: %s | GPS2Attach: %s \n\n", gpsDataPtr, _vertical_speed, gps1Attached, gps2Attached);
    
    write_to_log("TEL", _telemetry_data_string);

    _serialStream_Radio->uart_transmit(_telemetry_data_string);
}



/**
* \fn format_timestamp
* \brief Gets the current date/time and formats it into a standard
*    2015-01-28T23:09:28Z
* \return Pointer to the character string with the date/time 
*/
char* format_timestamp(char* timestampDest) {

    _current_time = time(NULL);
    struct tm tm = *gmtime(&_current_time);

    sprintf(timestampDest, 
        "%04d-%02d-%02dT%02d:%02d:%02dZ", 
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, 
        tm.tm_hour, tm.tm_min, tm.tm_sec);

    return timestampDest;
}



char* format_timestamp_with_milliseconds(char* timestampDest)
{
    if (timestampDest == NULL) return NULL;
    
    struct timeval timeNow;

    int mtime;    

    gettimeofday(&timeNow, NULL);

    mtime = timeNow.tv_usec / 1000.0 + 0.5;
    
    _current_time = time(NULL);
    struct tm tm = *gmtime(&_current_time);

    sprintf(timestampDest, 
        "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", 
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday, 
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        mtime);

    return timestampDest;
}


// ========================================================
// COMMANDS // COMMANDS // COMMANDS // COMMANDS // COMMANDS 
// COMMANDS // COMMANDS // COMMANDS // COMMANDS // COMMANDS 
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV

// Craft-payload
/**
* \fn send_craft_message
* \brief Sends a given message from the craft to the ground station
* \return COMMAND_SUCCESS or COMMAND_ERROR
*/
int send_craft_message(int index, int value) {

    if (_craft_notes_flags[index] != CRAFT_MESSAGE_SENT) {

        if (value != MESSAGE_NO_VALUE) {
            sprintf(_craftMessage, _craft_notes[index], value);
        }
        else {
            strcpy(_craftMessage, _craft_notes[index]);
        }

        if (_craft_notes_flags[index] == CRAFT_MESSAGE_NOT_SENT) {
            _craft_notes_flags[index] = CRAFT_MESSAGE_SENT;
        }

        if (_messageValidator.appendCheckSum(_craftMessage)) {

            write_to_log("CRM", _craftMessage);
            strcat(_craftMessage, "\n");

            _serialStream_Radio->uart_transmit(_craftMessage);

#if (DEBUG_PRINT)
            char debugMsg[128];

            sprintf(debugMsg, "Sending Craft Message: %s\n", _craftMessage);

            debug_print(debugMsg);
#endif
        }

        return COMMAND_SUCCESS;
    }

    return COMMAND_ERROR;
}


// Craft-payload
/**
* \fn release_balloon_now
* \brief Sends a command to the MicroController to release the balloon
*   It also sends a message to the ground station
* \return COMMAND_SUCCESS
*/
int release_balloon_now() {

    static bool proc3SendMsg = TRUE;
    
    _subProc1.send_command(PROC1_COMMAND_RELEASE_BALLOON);

    _balloon_released = TRUE;

    printf("Releasing Balloon NOW!\n");

    send_craft_message(BALLOON_RELEASED_POS, MESSAGE_NO_VALUE);
    
    if (proc3SendMsg)
    {
        _subProc3.send_command(PROC3_COMMAND_GOING_DOWN);
        proc3SendMsg = FALSE;
    }
    
#if (TEST_TELEMETRY)
    
    if (_balloon_state == BALLOON_RISING) {

        _balloon_state = BALLOON_RELEASED;

        _altitude_index = get_altitude_index(_current_altitude);

        _current_altitude = _altitude_table[_altitude_index].altitude_meters;

        _speed_increment = _altitude_table[_altitude_index + 1].speed_meters_per_second / 5;
    }

#endif

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "Release Balloon: %d", counter);

    write_to_log("BAP", _tmp_log_data);

    return COMMAND_SUCCESS;
}


// Craft-payload
/**
* \fn release_balloon_time
* \brief Accepts a time in minutes till the balloon should be released
* \return COMMAND_SUCCESS or COMMAND_ERROR
*/
int release_balloon_time(int minutesTillRelease) {

    printf("Release Balloon Time Called\n");
    
    if ((minutesTillRelease > 0) && (minutesTillRelease <= MINUTES_ALOFT_MAX)) {

        _minutes_till_release = minutesTillRelease;

        _seconds_till_release = minutesTillRelease * 60;

        printf("Releasing Balloon in %d minutes\n", minutesTillRelease);

        send_craft_message(MINUTES_TO_AUTORELEASE_POS, minutesTillRelease);

        return COMMAND_SUCCESS;
    }

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "Set Balloon Release Time: %d  -  %d", minutesTillRelease, counter);

    write_to_log("BAP", _tmp_log_data);

    return COMMAND_ERROR;
}


// Craft-payload
/**
* \fn deploy_parachute_now
* \brief Sends a command to the MicroController to deploy the main chute
*   It also sends a message to the ground station
* \return COMMAND_SUCCESS
*/
int deploy_parachute_now() {

    _subProc1.send_command(PROC1_COMMAND_DEPLOY_PARACHUTE);
    usleep(100000);
    _subProc1.send_command(PROC1_COMMAND_DEPLOY_PARACHUTE);
    usleep(100000);
    
    printf("Deploying Parachute NOW!\n");

    send_craft_message(PARACHUTE_DEPLOYED_POS, MESSAGE_NO_VALUE);

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "Deploy Parachute: %d", counter);

    write_to_log("BAP", _tmp_log_data);

#if (TEST_TELEMETRY)
    
    _balloon_state = PARACHUTE_DEPLOYED;

#endif

    return COMMAND_SUCCESS;
}


// Craft-payload
/**
* \fn deploy_parachute_height
* \brief Accepts a height in meters (above launch altitude)
*   in which the craft will automatically deploy the main chute
* \return COMMAND_SUCCESS
*/
int deploy_parachute_height(int metersAtDeploy) {

    _altitude_for_deploy = metersAtDeploy;

    printf("Deploying Parachute at %d meters.\n", metersAtDeploy);

    send_craft_message(AUTODEPLOY_HEIGHT_POS, metersAtDeploy);

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "Set Minumum Parachute Deploy Height: %d  -  %d", metersAtDeploy, counter);

    write_to_log("BAP", _tmp_log_data);

    return COMMAND_SUCCESS;
}


// Craft-payload
/**
* \fn rotate_video_camera
* \brief Rotates the live video camera "up" or "out"
* \return COMMAND_SUCCESS
*/
int rotate_video_camera(char position) {

    static int counter = 0;

    counter++;

    if ((position == 'u') || (position == 'U')) {

        _videoCameraUp = TRUE;

        _subProc1.send_command(PROC1_COMMAND_POSITION_CAMERA_UP);
        usleep(100000);
        _subProc1.send_command(PROC1_COMMAND_POSITION_CAMERA_UP);
        usleep(100000);
        
        printf("Positioning Video Camera Up!\n");

        sprintf(_tmp_log_data, "Positioning Video Camera Up!: %d", counter);

        write_to_log("BAP", _tmp_log_data);

        return COMMAND_SUCCESS;
    }

    _videoCameraUp = FALSE;

    printf("Positioning Video Camera Down!\n");

    _subProc1.send_command(PROC1_COMMAND_POSITION_CAMERA_OUT);
    usleep(100000);
    _subProc1.send_command(PROC1_COMMAND_POSITION_CAMERA_OUT);
    usleep(100000);
      
    sprintf(_tmp_log_data, "Positioning Video Camera Down!: %d", counter);

    write_to_log("VID", _tmp_log_data);

    return COMMAND_SUCCESS;

}


// Craft-payload
/**
* \fn display_user_message
* \brief Sends text to the on-board display
* \return COMMAND_SUCCESS
*/
int display_user_message(char* msg) {
    
    if (msg == NULL) return -1;
    
    int msgLen = strlen(msg);
    
    if (msgLen == 0) return -1;
    
    _pictureCount++;
    
    for (int i = 0; i < msgLen; i++)
    {
        if ((msg[i] == '\n') || (msg[i] == '\r'))
        {
            msg[i] = '\0';
        }
    }	

    printf("User message to display: %s\n", msg);
    
    _subProc3.send_user_text(msg);

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "User Message: %s  -  %d", msg, counter);

    write_to_log("MSG", _tmp_log_data);

    return COMMAND_SUCCESS;

}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// COMMANDS // COMMANDS // COMMANDS // COMMANDS // COMMANDS 
// COMMANDS // COMMANDS // COMMANDS // COMMANDS // COMMANDS
// ========================================================


// Craft, Mobile and Launch Station
/**
* \fn processCommand
* \brief Processes incoming commands
* \return COMMAND_SUCCESS
*/
int processCommand(char *msg) {

    char tmpCommand[64];
    int len;
    int meters;
    int mins;

    switch (msg[COMMAND_ID_POSITION]) {

        // Balloon message for use on craft (Not mobile or launch ground stations)
        case 'B':
        case 'b':
            if ((msg[COMMAND_PREFIX_SIZE] == '!') && (release_balloon_now() == COMMAND_SUCCESS)) {
                printf("release balloon immediate succeeded\n");

                break;
            }

            mins = atoi(&msg[COMMAND_PREFIX_SIZE]);

            if (mins > 0) {

                release_balloon_time(mins);
            }

            break;

            // Parachute message for use on craft (Not mobile or launch ground stations)
        case 'P':
        case 'p':
            if (msg[COMMAND_PREFIX_SIZE] == '!') {
                deploy_parachute_now();

                break;
            }

            meters = atoi(&msg[COMMAND_PREFIX_SIZE]);

            if (meters > 0) {

                deploy_parachute_height(meters);
            }

            break;

            // Video reposition message for use on craft (Not mobile or launch ground stations)
        case 'V':
        case 'v':
            if ((msg[COMMAND_PREFIX_SIZE] == 'U') || (msg[COMMAND_PREFIX_SIZE] == 'u') || (msg[COMMAND_PREFIX_SIZE] == 'O') || (msg[COMMAND_PREFIX_SIZE] == 'o')) {
                rotate_video_camera(msg[COMMAND_PREFIX_SIZE]);

                break;
            }

            break;

            // User message for use on craft (Not mobile or launch ground stations)
        case 'U':
        case 'u':
            len = strlen(&msg[COMMAND_PREFIX_SIZE]);

            if (len <= USER_MESSAGE_SIZE_MAX) {

                display_user_message(&msg[COMMAND_PREFIX_SIZE]);

                break;
            }

            break;
    }

    static int counter = 0;

    counter++;

    sprintf(_tmp_log_data, "Command Received: %s  -  %d", msg, counter);

    write_to_log("CMD", _tmp_log_data);

    return COMMAND_SUCCESS;
}


/**
* \fn wiring_pi_setup
* \brief Set up the access to GPIOs
*/
void wiring_pi_setup() {

    wiringPiSetup();

    pinMode(GPIO_PIN_BALLOON, INPUT);

    pinMode(GPIO_PIN_PARACHUTE, INPUT);

    pinMode(GPIO_PIN_SAFETY, INPUT);
}


void end_warmup_period() {
    
    printf("%s\n", "Warmup Period Over");
    
    _warmupPeriodOver = TRUE;
}



#if (TEST_TELEMETRY) 

// Called when the balloon is released or 
int get_altitude_index(double altitude) {

    for (int i = 0; i < _altitude_table_size; i++) {
        if (altitude >= _altitude_table[i].altitude_meters) {
            return i;
        }
    }

    return -1;
}

//double get_current_altitude(double altitude, bool falling) {
//
    //static bool is_falling = false;
//
    //if (!is_falling && falling) {  // just started falling
//
        ////int x = asdf;
        //is_falling = TRUE;
        //
        //
    //}
//
//}


void load_altitude_table() {

    const char* accelerationFile = "/home/pi/p2data/altitude_data.csv";

    _altitude_data_fp = fopen(accelerationFile, "r");

    if (_altitude_data_fp == NULL) {
        printf("Could not find data file: %s\n", accelerationFile);

        exit(EXIT_FAILURE);
    }
    else {

        // Read first line to get initial GPS values
        int read;
        int dataPos = 0;
        char* line = NULL;

        while ((read = getline(&line, &len, _altitude_data_fp)) != -1) {

            if (line) {

                int pos = findChar(line, ',', 0);
                dataPos = 0;

                if (pos != -1) {
                    line[pos] = '\0';
                    _altitude_table[_altitude_table_size].altitude_meters = atof(&line[dataPos]);

                    dataPos = pos + 1;
                    
                    _altitude_table[_altitude_table_size].speed_meters_per_second = atof(&line[dataPos]);
                }

                line = NULL;
                len = 0;
                _altitude_table_size++;

                free(line);
            }
        }

        fclose(_altitude_data_fp);
    }

}
 




void TestStep()
{
    _testStepCounter++;
    

    if (_testStepCounter < TEST_STEP_PREP_DELAY_COUNT)
    {
        printf("In Prep...\n");

        return;
    }
    
    if (_testStepCounter == TEST_STEP_PREP_DELAY_COUNT)
    {
        _warmupPeriodOver = TRUE;
        
        //int targetAltitude = TEST_TARGET_ALTITUDE_METERS;

        int totalRise = TEST_TARGET_ALTITUDE_METERS - _balloon_starting_position.alt;
        int remainder = totalRise % (int)(DEFAULT_ASCENT_RATE * TEST_ASCENT_RATE_MULTIPLIER * 2);

        _testRiseCount = totalRise / (DEFAULT_ASCENT_RATE * TEST_ASCENT_RATE_MULTIPLIER * 2);

        if (remainder)
        {
            _testRiseCount++;
        }
        
        _balloon_change_lat = (_balloon_starting_position.lat - _balloon_ending_position.lat) / (_testRiseCount + 500);
        _balloon_change_lon = (_balloon_starting_position.lon - _balloon_ending_position.lon) / (_testRiseCount + 500);
        
        _testRiseCount += TEST_STEP_PREP_DELAY_COUNT + 2;
        
        return;
    }
    
    if (_testStepCounter < _testRiseCount)  //3043)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount)  //3043)
    {
        release_balloon_now();

        return;
    }
    

    if (_testStepCounter < _testRiseCount + 100)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount + 100)
    {
        return;
    }

    if (_testStepCounter < _testRiseCount + 200)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount + 200)
    {
        return;
    }

    if (_testStepCounter < _testRiseCount + 300)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount + 300)
    {
        return;
    }

    if (_testStepCounter < _testRiseCount + 400)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount + 400)
    {
        return;
    }

    if (_testStepCounter < _testRiseCount + 500)
    {
        return;
    }
    
    if (_testStepCounter == _testRiseCount + 500)
    {
        return;
    }

}

#endif

        
void initialize_serial_devices()
{
    
    if (test_uart_streams('3') == IS_GPS)
    {
        _serialStream_Gps1 = new UartStream('3', process_gps_data1, FALSE, "GPS 1");
        _serialStream_Radio = new UartStream('2', process_radio_data, TRUE, "Radio Modem");
        write_to_log("InitDevices", "GPS1-3, Radio-2\n");
    }
    else
    {
        _serialStream_Gps1 = new UartStream('2', process_gps_data1, FALSE, "GPS 1");
        _serialStream_Radio = new UartStream('3', process_radio_data, TRUE, "Radio Modem");
        write_to_log("InitDevices", "GPS1-2, Radio-3\n");
    }

    if (test_uart_streams('1') == IS_GPS)
    {
        _serialStream_subProc2 = new UartStream('0', process_proc2_data, FALSE, "MicroController 2");
        _serialStream_Gps2 = new UartStream('1', process_gps_data2, FALSE, "GPS 2");
        write_to_log("InitDevices", "GPS2-1, Proc2-0\n");
    }
    else
    {
        _serialStream_subProc2 = new UartStream('1', process_proc2_data, FALSE, "MicroController 2");
        _serialStream_Gps2 = new UartStream('0', process_gps_data2, FALSE, "GPS 2");
        write_to_log("InitDevices", "GPS2-0, Proc2-1\n");
    }
}

        
int test_uart_streams(char uartNumber)
{
    UartStream* _serialStream_Test = new UartStream(uartNumber); //, process_radio_data, FALSE, "test");
    
    char tstBuffer[1024];
    
    
    usleep(1000000);
    
    for (int i = 0; i < 4; i++)
    {
        int bytes = _serialStream_Test->uart_receive(tstBuffer);
        
        if (bytes > 0)
        {
            _serialStream_Test->closeFileStream();
            
            if ((tstBuffer[0] == '$') && (tstBuffer[1] == 'G') && (tstBuffer[2] == 'P'))
            {
                return IS_GPS;
            }
            else
            {
                return IS_PROC2;				
            }
        }
    }
    
    _serialStream_Test->closeFileStream();
    
    return 0;
}
        

/**
* \fn main
* \brief Main function (setup and endless loop)
* \return 0 (never happens)
*/
int main(int argc, char *argv [])
{
    initialize_log(TELEMETRY_LOG_FILE);
   
    initialize_serial_devices();
    
    wiring_pi_setup();
    
    _initial_time = time(NULL);

    _pictureCount = 0;

    _safety_switch_was = digitalRead(GPIO_PIN_SAFETY);

    sprintf(_lastGpsDataString, "%7.5f,%7.5f,%3.1f,%3.1f,%3.1f,%d,%d",
        0.0, 0.0, 0.0, 0.0, 0.0, 0, 0);

    switch_leds(FALSE);
    
    display_user_message("Testing...");
    
    _subProc1.send_command(PROC3_COMMAND_RESET);
    //rotate_video_camera('U');
    
    _subProc1.send_command(PROC1_COMMAND_RESET_SERVOS);
    
    
    //display_user_message("Hello, I am testing you.  Please work!");
    //
    //_subProc3.send_command(3);
    //_subProc3.send_command(4);
    //_subProc3.send_command(5);
    //_subProc3.send_command(6);
    //_subProc3.send_command(7);
    //_subProc3.send_command(8);
    //_subProc3.send_command(9);
    //
    //
    //deploy_parachute_now();
    //
    //release_balloon_now();
    //
    //rotate_video_camera('U');
    //
    //
    //_subProc1.send_command(PROC1_COMMAND_RESET_SERVOS);
    //
    //
    //
    //deploy_parachute_now();
    //
    //release_balloon_now();
//
    //_subProc1.send_command(PROC1_COMMAND_RESET_SERVOS);

#if (TEST_TELEMETRY)

    // Load Altitude table for testing - Contains data to simulate falling. 
    //  Rising is handled through simple addition of a constant rise rate.
    load_altitude_table();

    // Zero out starting and ending positions 
    _balloon_starting_position.lat = 0.0;
    _balloon_starting_position.lon = 0.0;
    _balloon_starting_position.alt = 0.0;

    _balloon_ending_position.lat = 0.0;
    _balloon_ending_position.lon = 0.0;
    _balloon_ending_position.alt = 0.0;

    // Open the projected start / end GPS coordinates
    FILE* gpsFileHandle_fp;
    const char* gpsFile = GPS_PROJECTION_DATA_FILE;

    gpsFileHandle_fp = fopen(gpsFile, "r");

    if (gpsFileHandle_fp == NULL) {

        printf("Could not find data file: %s\n", gpsFile);
    }
    else {
        // Read first line to get initial GPS values
        ssize_t read;

        line = NULL;
        lineLength = 0;

        if ((read = getline(&line, &lineLength, gpsFileHandle_fp)) != -1) {

            if (line) {

                //int count = splitDataIntoArray(line, _tmp_telemetry_value_list, 6);
                int count = strsplit(line, ",", _tmp_telemetry_value_list);

                if (count == 6) {

                    _ground_altitude = atof(_tmp_telemetry_value_list[5]);

                    // 0: latitude  // 1: longitude  // 2: altitude
                    _balloon_starting_position.lat = atof(_tmp_telemetry_value_list[0]);
                    _balloon_starting_position.lon = atof(_tmp_telemetry_value_list[1]);
                    _balloon_starting_position.alt = atof(_tmp_telemetry_value_list[2]);

                    _balloon_ending_position.lat = atof(_tmp_telemetry_value_list[3]);
                    _balloon_ending_position.lon = atof(_tmp_telemetry_value_list[4]);
                    _balloon_ending_position.alt = atof(_tmp_telemetry_value_list[5]);


                    _balloon_current_position.lat = _balloon_starting_position.lat;
                    _balloon_current_position.lon = _balloon_starting_position.lon;
                    _balloon_current_position.alt = _balloon_starting_position.alt;

                    _current_altitude = _balloon_current_position.alt;

                    //_balloon_change_lat = (_balloon_starting_position.lat - _balloon_ending_position.lat) / 3000;
                    //_balloon_change_lon = (_balloon_starting_position.lon - _balloon_ending_position.lon) / 3000;
                }

                free(line);
            }
        }

        fclose(gpsFileHandle_fp);
    }
    
    _balloon_state = BALLOON_PREP;

    //_telemetry_timer.every(2000, TestStep);

    //char* dataFile = "/home/pi/p2data/P2CraftTestTelemtry.txt";

    //fp = fopen(dataFile, "r");

    //if (fp == NULL) {
    //    printf("Could not find data file: %s\n", dataFile);

    //    exit(EXIT_FAILURE);
    //}
    //else {

    //    // Read first line to get initial GPS values
    //    int read;

    //    char* line = NULL;

    //    if ((read = getline(&line, &len, fp)) != -1) {

    //        if (line) {

    //            while ((line[read - 1] == '\r') || (line[read - 1] == '\n')) {
    //                line[read - 1] = '\0';
    //                read--;
    //            }

    //            int checksumPos = _messageValidator.validateMessage(line);

    //            if (checksumPos) {

    //                int len = strlen(line);
    //                line[checksumPos] = '\0';

    //                strcpy(_tmp_telemetry_string, line);

    //                free(line);

    //                int count = splitDataIntoArray(_tmp_telemetry_string, _tmp_telemetry_value_list, DATA_ARRAY_COUNT);

    //                if (count == DATA_ARRAY_COUNT) {

    //                    // 23: latitude // 24: longitude // 25: altitude
    //                    _balloon_starting_position_delta.lat = atof(_tmp_telemetry_value_list[23]) - _balloon_starting_position_delta.lat;
    //                    _balloon_starting_position_delta.lon = atof(_tmp_telemetry_value_list[24]) - _balloon_starting_position_delta.lon;
    //                    _balloon_starting_position_delta.alt = atof(_tmp_telemetry_value_list[25]) - _balloon_starting_position_delta.alt;
    //                }
    //            }
    //            else {
    //                printf("Couldn't read text file - Initialization Failed.");

    //                exit(EXIT_FAILURE);
    //            }
    //        }
    //    }
    //}


#else

    _telemetry_timer.after(10000, end_warmup_period);
    
#endif

    _telemetry_timer.every(1000, get_m1_data);
    
    _telemetry_timer.every(2000, send_telemetry);
    
    int received0;

    while (TRUE) {
        _telemetry_timer.update();
        
        received0 = _serialStream_Gps1->uart_receive();

        received0 = _serialStream_subProc2->uart_receive();
        
        received0 = _serialStream_Gps2->uart_receive();

        received0 = _serialStream_Radio->uart_receive();

        usleep(1000);
    }

    return 0;
}