//#include <Servo.h>
#include "Timer.h"
#include <Wire.h>
//#include <Thermistor.h>
//#include <string.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>

#include "QueueArray.h"
#include "RunningAverage.h"

// Data Output
// tempIn, tempOut, balloonParachuteRelayOn, vidRelayOn, bvmTransferValue, bvaTransferValue, refLevel, _uvIntensityString, _vidCameraUp, _geiger_values[1], _geiger_values[3], _geiger_values[5]


// SSC32 "Pins" (Servo Positions)
#define SERVO_BAL_PIN                   0   // Output: PWM pin for controlling the balloon release servo
#define SERVO_PARA_PIN                  1   // Output: PWM pin for controlling the parachute deployment servo
#define SERVO_VID_PIN                   2   // Output: PWM pin for controlling the video camera position servo 


// UV Sensor pin definitions
#define UV_UVOUT_PIN                   A0   // Input: Analog pin used to gather the UV sensor values
#define ARDUINO_REF_3V3                A1   // Input: Analog pin used to measure accuracy of the 3.3V power on the Arduino board

#define BATTERY_LEVEL_AUX_PIN          A2   // Input: Analog pin for measuring the auxilliary battery voltage
#define BATTERY_LEVEL_MAIN_PIN         A3   // Input: Analog pin for measuring the main battery voltage

#define TEMP_IN_THERMISTOR_PIN         A4   // Input: Analog pin for measuring the inside temperature thermistor
#define TEMP_OUT_THERMISTOR_PIN        A5   // Input: Analog pin for measuring the outside temperature thermistor

#define BATTERY_VOLTAGE_FULL_MAIN     8.4   // Voltage level of a full battery (used to calc current voltage)
#define BATTERY_VOLTAGE_FULL_AUX      8.4   // Voltage level of a full battery (used to calc current voltage)
#define ADC_HIGH_VALUE              902.0   // Adjusted, high value for the ADC pins

#define I2C_ADDRESS                  0x04

#define SERVO_LAUNCH_POSITION_BALLOON      2000   
#define SERVO_RETRACT_POSITION_BALLOON     1300

#define SERVO_LAUNCH_POSITION_PARACHUTE    1000  
#define SERVO_RETRACT_POSITION_PARACHUTE   1700  

#define SERVO_OUT_POSITION_VID             1000  
#define SERVO_UP_POSITION_VID              2000

#define SERVO_SPEED_VID                    1500
#define SERVO_SPEED_DEFAULT                1500

#define DATA_ARRAY_STR_LEN                   8


#define PACKET_LENGTH                        32
#define SIZES_PACKET_LENGTH                   3

#define REFERENCE_VOLTAGE                  4.96

#define RELAY_TIMER_UNDEFINED                -1
#define RELAY_TIMER_DELAY_MS               4000
#define SERVO_TIMER_DELAY_MS                200

#ifndef TRUE
#define TRUE    (1==1)
#endif
#ifndef FALSE
#define FALSE   (0==1)
#endif



void setupServos();
void receiveData(int byteCount);
void sendData();
void doSensorSamples();
void serialEvent();
void releaseBalloon();
void deployParachute();
void positionCamera(bool);

float calcBatteryLevel(int batteryLevel, float initialLevel);

int averageAnalogRead(int pinToRead);

void positionVidServoUp();
void positionVidServoOut();

void resetParachute();
void resetBalloon();

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

void setServoPosition(int pin, int positionValue);
void setServoPositionSpeed(int pin, int positionValue, int speed);

char _sensorData[64];

bool _ledOn = FALSE; 
bool volatile _vidCameraUp = FALSE;

char _geigerData[64];
char _geigerBuffer[64];
int _geigerCharIndex = 0;
char _geiger_values[7][DATA_ARRAY_STR_LEN];

char _uvIntensityString[6];

Timer _timer;
//Thermistor _thermistor_in(9950, REFERENCE_VOLTAGE);
//Thermistor _thermistor_out(9960, REFERENCE_VOLTAGE);


char _sendBuffer[64];
uint8_t _sizesBuffer[3];
bool _initialCall = TRUE;
int _fullPackets;
int _partialPacketSize;
int _fullPacketsSent = 0;
int _xferBufferPtr = 0;
uint8_t _transferBufferLen;

QueueArray<uint8_t> _commandQueue;

SoftwareSerial _servoSerial(10, 11);

RunningAverage _uvLevelAverage(10);
RunningAverage _refLevelAverage(10);


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
    WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);  // 2 Seconds

    sei(); // enable interrupts
}


void watchdog_reset() {
    wdt_reset();
}

#define COMMAND_RELEASE_BALLOON         1
#define COMMAND_DEPLOY_PARACHUTE        2
#define COMMAND_POSITION_VIDEO_OUT      3
#define COMMAND_POSITION_VIDEO_UP       4

#define COMMAND_RESET_DATA_TRANSFER     5

#define COMMAND_RESET_BALLOON           6
#define COMMAND_RESET_PARACHUTE         7

#define COMMAND_RESET_SERVOS            9

// callback for received data
void receiveData(int byteCount){

    while (Wire.available()) {
        int command = Wire.read();

        Serial.print(">>> Command Received: ");
        Serial.println(command);

        switch (command) {
            case COMMAND_RELEASE_BALLOON:         // Release Balloon
                _commandQueue.enqueue(command);
                break;

            case COMMAND_DEPLOY_PARACHUTE:         // Deploy Main Chute
                _commandQueue.enqueue(command);
                break;

            case COMMAND_POSITION_VIDEO_OUT:         // Position Video Out
                _commandQueue.enqueue(command);
                break;

            case COMMAND_POSITION_VIDEO_UP:         // Position Video Up
                _commandQueue.enqueue(command);
                break;

            case COMMAND_RESET_DATA_TRANSFER:         // Reset data transfer
                _initialCall = TRUE;
                break;

            case COMMAND_RESET_SERVOS:         // Reset Servos
                setupServos();
                break;
        }
    }
}


void execute_commands()
{
    if (!_commandQueue.isEmpty())
    {
        int command = _commandQueue.dequeue();
        
        switch (command) {
            case COMMAND_RELEASE_BALLOON:         // Release Balloon
                releaseBalloon();
                break;

            case COMMAND_DEPLOY_PARACHUTE:         // Deploy Main Chute
                deployParachute();
                break;

            case COMMAND_POSITION_VIDEO_OUT:         // Position Video Out
                positionCamera(FALSE);
                break;

            case COMMAND_POSITION_VIDEO_UP:         // Position Video Up
                positionCamera(TRUE);
                break;

            case COMMAND_RESET_BALLOON:         // Release Balloon
                resetBalloon();
                break;

            case COMMAND_RESET_PARACHUTE:         // Deploy Main Chute
                resetParachute();
                break;
        }
    }
}



// callback for sending data
void sendData() {

    if (_ledOn) {
      _ledOn = FALSE;
    } else {
      _ledOn = TRUE;
    }

    // Toggle the light every time data is requested/sent
    digitalWrite(LED_BUILTIN, _ledOn);
    
    // Initial call: send back amount of data to transfer
    if (_initialCall) {

        int datalen = strlen(_sensorData);

        if (datalen >= 64)
        {
            return;
        }

        strcpy(_sendBuffer, _sensorData);             // temporarily store the sensor data to send
        _transferBufferLen = datalen;
        _fullPacketsSent = 0;
        _xferBufferPtr = 0;

        _fullPackets = (int) (_transferBufferLen / PACKET_LENGTH);
        _partialPacketSize = _transferBufferLen % PACKET_LENGTH;

        _sizesBuffer[0] = _transferBufferLen;
        _sizesBuffer[1] = _fullPackets;
        _sizesBuffer[2] = _partialPacketSize;

        Wire.write(_sizesBuffer, SIZES_PACKET_LENGTH);

        _initialCall = FALSE;

        return;
    } 

    if (_fullPacketsSent < _fullPackets) {

        _fullPacketsSent++;

        Wire.write(&_sendBuffer[_xferBufferPtr], PACKET_LENGTH);

        _xferBufferPtr += PACKET_LENGTH;
    }
    else {

        if (_xferBufferPtr < _transferBufferLen) {
            unsigned int transferCount = _transferBufferLen - _xferBufferPtr;
        
            Wire.write(&_sendBuffer[_xferBufferPtr], _transferBufferLen - _xferBufferPtr);

            _xferBufferPtr += transferCount;
        }
        else {
            Serial.println("Data Rec Error");
        }
    }

    // Done - reset everything for the next call
    if (_xferBufferPtr >= _transferBufferLen) {
        _initialCall = TRUE;
    }
}


int strsplit(const char* str, const char* delim, char dataArray[][DATA_ARRAY_STR_LEN]) {
    // copy the original string so that we don't overwrite parts of it
    // (don't do this if you don't need to keep the old line,
    // as this is less efficient)
    char *s = strdup(str);

    // these three variables are part of a very common idiom to
    // implement a dynamically-growing array
    unsigned int tokens_alloc = 1;
    unsigned int tokens_used = 0;

    char **tokens = (char**)calloc(tokens_alloc, sizeof(char*));

    char *token, *strtok_ctx;
    for (token = strtok_r(s, delim, &strtok_ctx); token != NULL; token = strtok_r(NULL, delim, &strtok_ctx))
    {
        // check if we need to allocate more space for tokens
        if (tokens_used == tokens_alloc) {
            tokens_alloc *= 2;
            tokens = (char**)realloc(tokens, tokens_alloc * sizeof(char*));
        }

        tokens[tokens_used++] = strdup(token);
    }

    // cleanup
    if (tokens_used == 0) {
        free(tokens);
        tokens = NULL;

    }
    else {
        tokens = (char**)realloc(tokens, tokens_used * sizeof(char*));
    }

    free(s);

    for (unsigned int i = 0; i < tokens_used; i++)
    {
        strcpy(dataArray[i], tokens[i]);

        free(tokens[i]);
    }

    if (tokens != NULL)
    {
        free(tokens);
    }

    return tokens_used;
}


void serialEvent() {

    while (Serial1.available()) {
        
        char inChar = (char) Serial1.read();

        if (_geigerCharIndex < 99) {
            _geigerBuffer[_geigerCharIndex++] = inChar;
        } else {
            _geigerCharIndex = 0;
        }

        if (inChar == '\n') {
            
            _geigerCharIndex = 0;

            if (_geigerBuffer[0] == 'C') {
                strcpy(_geigerData, _geigerBuffer);

                strsplit(_geigerData, ",", _geiger_values);
            }
        }
    }
}


void doSensorSamples() {

    int batteryLevelMain = analogRead(BATTERY_LEVEL_MAIN_PIN);
    int batteryLevelVid = analogRead(BATTERY_LEVEL_AUX_PIN);

    float batteryVoltageMain = calcBatteryLevel(batteryLevelMain, BATTERY_VOLTAGE_FULL_MAIN);
    float batteryVoltageVid = calcBatteryLevel(batteryLevelVid, BATTERY_VOLTAGE_FULL_AUX);

    int bvmTransferValue = (int) (batteryVoltageMain * 10.0);
    int bvaTransferValue = (int) (batteryVoltageVid * 10.0);


    _uvLevelAverage.addValue(analogRead(UV_UVOUT_PIN));
    _refLevelAverage.addValue(analogRead(ARDUINO_REF_3V3));

    //int uvLevel = averageAnalogRead(UV_UVOUT_PIN);
    //int refLevel = averageAnalogRead(ARDUINO_REF_3V3);

    float refLevel = _refLevelAverage.getAverage();

    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    //float outputVoltage = 3.3 / refLevel * uvLevel;
    float outputVoltage = 3.3 / refLevel * _uvLevelAverage.getAverage();

    float calculatedUvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

    // dtostrf(float value, total chars in string including decimal and digits after decimal, 
    //                    total chars after decimal, destination)
    dtostrf(calculatedUvIntensity, 4, 2, _uvIntensityString);

    if (strlen(_geiger_values[1]) == 0)
    {
        strcpy(_geiger_values[1], "0");
    }

    if (strlen(_geiger_values[3]) == 0)
    {
        strcpy(_geiger_values[3], "0");
    }

    if (strlen(_geiger_values[5]) == 0)
    {
        strcpy(_geiger_values[5], "0");
    }

    sprintf(_sensorData, "990,990,0,0,%d,%d,%d,%s,%d,%s,%s,%s",
        //tempInTransferValue, tempOutTransferValue,          // Temp In/Out
        //_balloonParachuteRelayOn, _vidRelayOn,                // Relays On/Off
        bvmTransferValue, bvaTransferValue,                 // Battery Voltage times 10 (int)
        (int)refLevel, _uvIntensityString,                        // Reference Level, 
        _vidCameraUp, _geiger_values[1], _geiger_values[3], _geiger_values[5]);

    Serial.println(_sensorData);
}

//The Arduino Map function but for floats
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float calcBatteryLevel(int batteryLevel, float initialLevel) {

    if ((batteryLevel > 0) && (batteryLevel > 0)) {
        float ratio = (float) batteryLevel / ADC_HIGH_VALUE;

        float batVal = ratio * initialLevel;

        return (batVal > initialLevel) ? initialLevel : batVal;
    }

    return 0.0;
}


void positionCamera(bool vidUp) {
    
    if (vidUp) {
        positionVidServoUp();
    }
    else {
        positionVidServoOut();
    }

    _vidCameraUp = vidUp;
}

// Trigger the video servo up toward the balloon/parachute
void positionVidServoUp() {
    Serial.println("positionVidServoUp");

    setServoPositionSpeed(SERVO_VID_PIN, SERVO_UP_POSITION_VID, SERVO_SPEED_VID);
}

// Trigger the video servo out toward the horizon
void positionVidServoOut() {
    Serial.println("positionVidServoOut");

    setServoPositionSpeed(SERVO_VID_PIN, SERVO_OUT_POSITION_VID, SERVO_SPEED_VID);
}


void setupServos() {

    _commandQueue.enqueue(COMMAND_RESET_BALLOON);

    _commandQueue.enqueue(COMMAND_RESET_PARACHUTE);
    
    _commandQueue.enqueue(COMMAND_POSITION_VIDEO_OUT);
}

void resetParachute() {

    Serial.println("resetParachute");

    setServoPosition(SERVO_PARA_PIN, SERVO_LAUNCH_POSITION_PARACHUTE);
}


void resetBalloon() {

    Serial.println("resetBalloon");

    setServoPosition(SERVO_BAL_PIN, SERVO_LAUNCH_POSITION_BALLOON);
}


void deployParachute() {

    Serial.println("deployParachute");

    setServoPosition(SERVO_PARA_PIN, SERVO_RETRACT_POSITION_PARACHUTE);
}


void releaseBalloon() {

    Serial.println("releaseBalloon");

    setServoPosition(SERVO_BAL_PIN, SERVO_RETRACT_POSITION_BALLOON);
}



void setServoPosition(int pin, int positionValue) {

    //#0P1000
    char servoOutput[24];

    sprintf(servoOutput, "#%d P%d", pin, positionValue);

    _servoSerial.println(servoOutput);
}


void setServoPositionSpeed(int pin, int positionValue, int speed) {

    //#0P1000
    char servoOutput[24];

    sprintf(servoOutput, "#%d P%d T%d", pin, positionValue, speed);

    _servoSerial.println(servoOutput);
}



void setup()
{
    Wire.begin(I2C_ADDRESS);

    // Set up Serial Communication
    Serial.begin(115200);
    Serial1.begin(9600);  // Geiger Counter Input Data
    _servoSerial.begin(9600);  // Servo Controller

    _uvLevelAverage.clear();
    _refLevelAverage.clear();

    // Used to visually indicate when data is being sent 
    pinMode(LED_BUILTIN, OUTPUT);

    // UV Sensor
    pinMode(UV_UVOUT_PIN, INPUT);
    pinMode(ARDUINO_REF_3V3, INPUT);

    delay(2000);

    // Initialize all servo positions to start
    setupServos();


    //delay(8000);
    //setServoPosition(SERVO_BAL_PIN, SERVO_RETRACT_POSITION_BALLOON);*/

    //setServoPosition(SERVO_PARA_PIN, SERVO_RETRACT_POSITION_PARACHUTE);


    Wire.onReceive(receiveData);

    Wire.onRequest(sendData);

    int i;
    for (i = 0; i < 3; i++) {
        _geiger_values[i][0] = '\0';
    }

    watchdogSetup();

    _timer.every(1000, doSensorSamples);
    _timer.every(1000, watchdog_reset);
    _timer.every(1000, execute_commands);

    // I2C call setup
    _initialCall = TRUE;
}

void loop()
{
    _timer.update();

    serialEvent();
}

