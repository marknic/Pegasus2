#include <Servo.h>
#include "Timer.h"
#include <Wire.h>
#include <Thermistor.h>
#include <string.h>
#include <avr/wdt.h>



// Servo relay numbers:
// Relay 1: Arduino Pin 7: Video Servo
// Relay 2: Arduino Pin 6: Balloon and Parachute Servos

// Data Output
// tempIn, tempOut, balloonParachuteRelayOn, vidRelayOn, bvmTransferValue, bvaTransferValue, refLevel, uvIntensityString, vidCameraUp, geiger_values[1], geiger_values[3], geiger_values[5]

#define RELAY_BAL_PARA_SERVO_PIN        6   // Output: Digital pin that signals the parachute and balloon servo power
#define RELAY_VID_SERVO_PIN             7   // Output: Digital pin that signals the video camera servo power

#define SERVO_VID_PIN                  11   // Output: PWM pin for controlling the video camera position servo 
#define SERVO_BAL_PIN                  10   // Output: PWM pin for controlling the balloon release servo
#define SERVO_PARA_PIN                  9   // Output: PWM pin for controlling the parachute deployment servo

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

#define RELAY_ON_VALUE                LOW
#define RELAY_OFF_VALUE              HIGH

#define SERVO_LAUNCH_POSITION_BALLOON      20   
#define SERVO_RETRACT_POSITION_BALLOON     160   
#define SERVO_LAUNCH_POSITION_PARACHUTE     20   
#define SERVO_RETRACT_POSITION_PARACHUTE   160   
#define SERVO_OUT_POSITION_VID              39   
#define SERVO_UP_POSITION_VID              139   

#define DATA_ARRAY_STR_LEN                  12


#define PACKET_LENGTH           32
#define SIZES_PACKET_LENGTH     3

#define REFERENCE_VOLTAGE                 4.96

#define RELAY_TIMER_UNDEFINED               -1
#define RELAY_TIMER_DELAY_MS              4000
#define SERVO_TIMER_DELAY_MS               200

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif



void setupServos();
void receiveData(int byteCount);
void sendData();
void doSensorSamples();
void serialEvent();
void releaseBalloon();
void deployParachute();
void positionCamera(bool);
void turnVidRelayOn();
void turnVidRelayOff();
float calcBatteryLevel(int batteryLevel, float initialLevel);
int averageAnalogRead(int pinToRead);
void positionVidServoUp();
void positionVidServoOut();
void turnBalParaRelayOn();
void turnBalParaRelayOff();
void releaseBalloonServo();
void deployParachuteServo();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void setRelayVidOffTimer();
void setRelayBalParaOffTimer();

Servo vidServo;
Servo balloonServo;
Servo parachuteServo;

char sensorData[128];

bool _ledOn = FALSE;

int balloonParachuteRelayOn = 0;
int vidRelayOn = 0;
bool volatile vidCameraUp = FALSE;

char geigerData[100];
char geigerBuffer[100];
int geigerCharIndex = 0;
char geiger_values[7][DATA_ARRAY_STR_LEN];

char outputVoltageString[10];
char uvIntensityString[6];

Timer _timer;
Thermistor _thermistor_in(9950, REFERENCE_VOLTAGE);
Thermistor _thermistor_out(9960, REFERENCE_VOLTAGE);


char sendBuffer[64];
uint8_t sizesBuffer[3];
bool initialCall = TRUE;
int fullPackets;
int partialPacketSize;
int fullPacketsSent = 0;
int xferBufferPtr = 0;
uint8_t transferBufferLen;

int8_t relayBalParaTimerId = RELAY_TIMER_UNDEFINED;
int8_t relayVideoTimerId = RELAY_TIMER_UNDEFINED;

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


void setup()
{
    Wire.begin(I2C_ADDRESS);

    // Set up Serial Communication
    Serial.begin(115200);
    Serial1.begin(9600);  // Geiger Counter Input Data

    // Initialize the values before activating
    digitalWrite(RELAY_VID_SERVO_PIN, HIGH);
    digitalWrite(RELAY_BAL_PARA_SERVO_PIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Activate the pins
    pinMode(RELAY_VID_SERVO_PIN, OUTPUT);
    pinMode(RELAY_BAL_PARA_SERVO_PIN, OUTPUT);

    // Used to visually indicate when data is being sent 
    pinMode(LED_BUILTIN, OUTPUT);
    
    // UV Sensor
    pinMode(UV_UVOUT_PIN, INPUT);
    pinMode(ARDUINO_REF_3V3, INPUT);

    // Connect the servo objects to their pins
    vidServo.attach(SERVO_VID_PIN);
    balloonServo.attach(SERVO_BAL_PIN);
    parachuteServo.attach(SERVO_PARA_PIN);

    // Initialize all servo positions to start
    setupServos();
    
    Wire.onReceive(receiveData);

    Wire.onRequest(sendData); 

    int i;
    for (i = 0; i < 3; i++) {
        geiger_values[i][0] = '\0';
    }

    watchdogSetup();

    _timer.every(1000, doSensorSamples);
    _timer.every(1000, watchdog_reset);

    //_timer.oscillate(LED_BUILTIN, 500, HIGH, 10);

    // I2C call setup
    initialCall = TRUE;

    //Serial.println("end of setup");

}

void loop()
{
    _timer.update();

    serialEvent();
}


// callback for received data
void receiveData(int byteCount){

    while (Wire.available()) {
        int command = Wire.read();

        switch (command) {
            case 1:         // Release Balloon
                Serial.println("Release Balloon Command");
                releaseBalloon();
                break;

            case 2:         // Deploy Main Chute
                Serial.println("Deploy Main Chute Command");
                deployParachute();
                break;

            case 3:         // Position Video Out
                Serial.println("Position Video Out Command");
                positionCamera(FALSE);
                break;

            case 4:         // Position Video Up
                positionCamera(TRUE);
                Serial.println("Position Video Up Command");
                break;

            case 5:         // Reset data transfer
                Serial.println("Reset data transfer");
                initialCall = TRUE;
                break;
        }
    }
}


/*
strcpy(tmpBuffer, buffer);
xferBufferPtr = 0;
transferBufferLen = strlen(tmpBuffer);

fullPackets = (int) (transferBufferLen / PACKET_LENGTH);
partialPacketSize = transferBufferLen % PACKET_LENGTH;
fullPacketsSent = 0;

returnBuffer[0] = transferBufferLen;
returnBuffer[1] = fullPackets;
returnBuffer[2] = partialPacketSize;
*/




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
    if (initialCall) {
        //Serial.println("initial call");

        strcpy(sendBuffer, sensorData);             // temporarily store the sensor data to send
        transferBufferLen = strlen(sendBuffer);
        fullPacketsSent = 0;
        xferBufferPtr = 0;

        fullPackets = (int) (transferBufferLen / PACKET_LENGTH);
        partialPacketSize = transferBufferLen % PACKET_LENGTH;

        sizesBuffer[0] = transferBufferLen;
        sizesBuffer[1] = fullPackets;
        sizesBuffer[2] = partialPacketSize;

        //Serial.print("byte 1: "); Serial.println(sizesBuffer[0], HEX);
        //Serial.print("byte 2: "); Serial.println(sizesBuffer[1], HEX);
        //Serial.print("byte 3: "); Serial.println(sizesBuffer[2], HEX);

        Wire.write(sizesBuffer, SIZES_PACKET_LENGTH);

        initialCall = FALSE;

        return;
    } 

    if (fullPacketsSent < fullPackets) {

        fullPacketsSent++;

        //Serial.print("Sending: "); Serial.print(PACKET_LENGTH); Serial.println(" bytes.");
        
        Wire.write(&sendBuffer[xferBufferPtr], PACKET_LENGTH);

        xferBufferPtr += PACKET_LENGTH;
    }
    else {

        if (xferBufferPtr < transferBufferLen) {
            size_t transferCount = transferBufferLen - xferBufferPtr;
            //Serial.print("Sending: "); Serial.print(transferCount); Serial.println(" bytes.");
            Wire.write(&sendBuffer[xferBufferPtr], transferBufferLen - xferBufferPtr);

            xferBufferPtr += transferCount;
        }
        else {
            Serial.println("Error receiving data");
        }
    }

    // Done - reset everything for the next call
    if (xferBufferPtr >= transferBufferLen) {
        // We're done
        //Serial.println("Done!");
        initialCall = TRUE;
    }
}


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



int splitDataIntoArray(char *dataIn, char dataArray [][DATA_ARRAY_STR_LEN], int arraySize) {

    int pos = -1;
    int currentPos = 0;

    for (int i = 0; i < arraySize; i++) {
        pos = findChar(dataIn, ',', currentPos);

        dataIn[pos] = 0;

        strcpy(dataArray[i], trim(&dataIn[currentPos]));

        //Serial.print("data: "); Serial.println(dataArray[i]);

        currentPos = pos + 1;

        if (pos == -1) {
            //Serial.println("END!");
            return(i + 1);
            break;
        }
    }

    return arraySize + 1;
}



void serialEvent() {

    while (Serial1.available()) {
        
        char inChar = (char) Serial1.read();

        if (geigerCharIndex < 99) {
            geigerBuffer[geigerCharIndex++] = inChar;
        } else {
            geigerCharIndex = 0;
        }

        if (inChar == '\n') {
            
            geigerCharIndex = 0;

            if (geigerBuffer[0] == 'C') {
                strcpy(geigerData, geigerBuffer);

                splitDataIntoArray(geigerData, geiger_values, 7);
            }
        }
    }
}


void doSensorSamples() {
    
    //int tempInVoltageLevel = analogRead(TEMP_IN_THERMISTOR_PIN);
    //int tempOutVoltageLevel = analogRead(TEMP_OUT_THERMISTOR_PIN);

    //float tempIn = _thermistor_in.getAdjustedTemperature(tempInVoltageLevel);
    //float tempOut = _thermistor_out.getAdjustedTemperature(tempOutVoltageLevel);

    //int tempInTransferValue = (int) (tempIn * 10.0);
    //int tempOutTransferValue = (int) (tempOut * 10.0);

    //Serial.print("TempIn: "); Serial.print(tempInVoltageLevel);
    //Serial.print("  -  TempOut: "); Serial.println(tempOutVoltageLevel);

    int batteryLevelMain = analogRead(BATTERY_LEVEL_MAIN_PIN);
    int batteryLevelVid = analogRead(BATTERY_LEVEL_AUX_PIN);

    float batteryVoltageMain = calcBatteryLevel(batteryLevelMain, BATTERY_VOLTAGE_FULL_MAIN);
    float batteryVoltageVid = calcBatteryLevel(batteryLevelVid, BATTERY_VOLTAGE_FULL_AUX);

    int bvmTransferValue = (int) (batteryVoltageMain * 10.0);
    int bvaTransferValue = (int) (batteryVoltageVid * 10.0);

    int uvLevel = averageAnalogRead(UV_UVOUT_PIN);
    int refLevel = averageAnalogRead(ARDUINO_REF_3V3);

    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    float outputVoltage = 3.3 / refLevel * uvLevel;

    float calculatedUvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

    // dtostrf(float value, total chars in string including decimal and digits after decimal, 
    //                    total chars after decimal, destination)
    dtostrf(calculatedUvIntensity, 4, 2, uvIntensityString);

    char g1[8];
    char g2[8];
    char g3[8];

    if (strlen(geiger_values[1]) == 0)
    {
        strcpy(g1, "0");
    } else
    {
        strcpy(g1, geiger_values[1]);
    }

    if (strlen(geiger_values[3]) == 0)
    {
        strcpy(g2, "0");
    }
    else
    {
        strcpy(g2, geiger_values[3]);
    }
    if (strlen(geiger_values[5]) == 0)
    {
        strcpy(g3, "0");
    }
    else
    {
        strcpy(g3, geiger_values[5]);
    }

    sprintf(sensorData, "990,990,%d,%d,%d,%d,%d,%s,%d,%s,%s,%s",
        //tempInTransferValue, tempOutTransferValue,          // Temp In/Out
        balloonParachuteRelayOn, vidRelayOn,                // Relays On/Off
        bvmTransferValue, bvaTransferValue,                 // Battery Voltage times 10 (int)
        refLevel, uvIntensityString,                        // Reference Level, 
        vidCameraUp, g1, g2, g3);

    Serial.println(sensorData);

}

//The Arduino Map function but for floats
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
    unsigned int numberOfReadings = 8;
    unsigned int runningValue = 0;

    for (int x = 0; x < numberOfReadings; x++)
        runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;

    return(runningValue);
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
    
    Serial.println("In positionCamera()");

    turnVidRelayOn();

    if (vidUp) {
        _timer.after(SERVO_TIMER_DELAY_MS, positionVidServoUp);
    }
    else {
        _timer.after(SERVO_TIMER_DELAY_MS, positionVidServoOut);
    }

    setRelayVidOffTimer();

    vidCameraUp = vidUp;
}

// Trigger the video servo up toward the balloon/parachute
void positionVidServoUp() {
    Serial.println("Position Vid Up (Servo)");

    vidServo.write(SERVO_UP_POSITION_VID);
}

// Trigger the video servo out toward the horizon
void positionVidServoOut() {
    Serial.println("Position Vid Out (Servo)");

    vidServo.write(SERVO_OUT_POSITION_VID);
}


void setRelayBalParaOffTimer()
{

    if (relayBalParaTimerId != RELAY_TIMER_UNDEFINED)
    {
        _timer.stop(relayBalParaTimerId);
    }

    relayBalParaTimerId = _timer.after(RELAY_TIMER_DELAY_MS, turnBalParaRelayOff);

}


void setRelayVidOffTimer()
{

    if (relayVideoTimerId != RELAY_TIMER_UNDEFINED)
    {
        _timer.stop(relayVideoTimerId);
    }

    relayVideoTimerId = _timer.after(RELAY_TIMER_DELAY_MS, turnVidRelayOff);
}


void setupServos() {

    turnBalParaRelayOn();

    turnVidRelayOn();
 
    positionCamera(false);

delay(2000);

    balloonServo.write(SERVO_LAUNCH_POSITION_BALLOON);

delay(2000);

    parachuteServo.write(SERVO_LAUNCH_POSITION_PARACHUTE);
    
    setRelayBalParaOffTimer();
}



void deployParachute() {

    Serial.println("In deployParachute()");

    turnBalParaRelayOn();

    _timer.after(SERVO_TIMER_DELAY_MS, deployParachuteServo);

    setRelayBalParaOffTimer();
}


void releaseBalloon() {

    Serial.println("In releaseBalloon()");

    turnBalParaRelayOn();

    _timer.after(SERVO_TIMER_DELAY_MS, releaseBalloonServo);

    setRelayBalParaOffTimer();
}


// Trigger the Parachute servo to deploy it during decent
void deployParachuteServo() {
    Serial.println("Deploy Parachute (Servo)");

    parachuteServo.write(SERVO_LAUNCH_POSITION_PARACHUTE);

    delay(500);

    parachuteServo.write(SERVO_RETRACT_POSITION_PARACHUTE);
}


// Trigger the Balloon servo to release it from the payload
void releaseBalloonServo() {
    Serial.println("Release Balloon (Servo)");

    balloonServo.write(SERVO_LAUNCH_POSITION_BALLOON);

    delay(500);

    balloonServo.write(SERVO_RETRACT_POSITION_BALLOON);
}

// Turns the Balloon/Parachute relay ON or OFF
void signalBalParaRelay(bool relayOn) {

    Serial.print("Balloon/Parachute Relay ");
    Serial.println(relayOn ? "On" : "Off");
    
    balloonParachuteRelayOn = relayOn;

    if (relayOn){
        digitalWrite(RELAY_BAL_PARA_SERVO_PIN, RELAY_ON_VALUE);
    }
    else {
        digitalWrite(RELAY_BAL_PARA_SERVO_PIN, RELAY_OFF_VALUE);
    }
}

// Turns the Balloon/Parachute relay ON
void turnBalParaRelayOn() {
    signalBalParaRelay(TRUE);
}

// Turns the Balloon/Parachute relay OFF
void turnBalParaRelayOff() {
    relayBalParaTimerId = RELAY_TIMER_UNDEFINED;

    signalBalParaRelay(FALSE);
}


// Turns the Vid video relay ON or OFF
void signalVidRelay(bool relayOn) {

    Serial.print("VID (Video) Relay ");
    Serial.println(relayOn ? "On" : "Off");
    
    vidRelayOn = relayOn;

    if (relayOn){
        digitalWrite(RELAY_VID_SERVO_PIN, RELAY_ON_VALUE);
    }
    else {
        digitalWrite(RELAY_VID_SERVO_PIN, RELAY_OFF_VALUE);
    }
}



// Turns the Video relay ON
void turnVidRelayOn() {
    signalVidRelay(TRUE);
}

// Turns the Balloon/Parachute relay OFF
void turnVidRelayOff() {
    relayVideoTimerId = RELAY_TIMER_UNDEFINED;

    signalVidRelay(FALSE);
}

