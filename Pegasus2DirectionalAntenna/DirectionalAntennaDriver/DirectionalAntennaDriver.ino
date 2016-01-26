#include "CompassSensor.h"
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <avr/wdt.h>
//#include "AzimuthElevation.h"
#include "AppTimer.h"

#define SERIAL_BAUD_RATE              38400

// These values center the 2 servos
#define SERVO_AZIMUTH_CENTER           90.0
#define SERVO_ELEVATION_CENTER         90.0

#define SERVO_AZIMUTH_PIN                 5
#define SERVO_ELEVATION_PIN               6

#define SERVO_RELAY_PIN                   9

// Maximum and minumum servo values 
#define SERVO_AZIMUTH_MIN                42 
#define SERVO_AZIMUTH_MAX               140 

#define SERVO_ELEVATION_HIGH             15 // HIGH
#define SERVO_ELEVATION_LOW             100 // LOW

// Maximum and minimum degree range for the elevation servo
//#define ELEVATION_DEGREES_MIN         -15.0
//#define ELEVATION_DEGREES_MAX          90.0

// Maximum and minimum degree range for the azimuth (compass) servo
#define COMPASS_MAX_VAL                 359
#define COMPASS_MIN_VAL                   0
#define COMPASS_RANGE_TOTAL             (COMPASS_MAX_VAL - COMPASS_MIN_VAL + 1)  //256
#define COMPASS_RANGE_HALF              (COMPASS_RANGE_TOTAL / 2)  //128

#define DATA_REQUEST_LEN                  8
#define AVERAGE_COUNT_HEADING             4

#define DIRECTION_COUNTER_CLOCKWIZE       1
#define DIRECTION_CLOCKWIZE              -1

// This is the range amount (above and below) where movement will not occur
//  it is used to keep the position of the antenna/camera from constantly 
//  changing as the sensor changes
#define DISTANCE_WITHIN_RANGE           2.5


#define SERVO_AZIMUTH_POSITION_COUNT     (SERVO_AZIMUTH_MAX - SERVO_AZIMUTH_MIN + 1)

#define TARGET_DIRECTION_INIT_VALUE    -1.0

#define AZIMUTH_MOVE_PERCENTAGE        0.15
#define ELEVATION_MOVE_PERCENTAGE      0.75

#define FLOAT_EMPTY_FLAG             -999.0
#define FLOAT_EMPTY_LIMIT            -900.0
#define DEGREES_PER_RADIAN       57.2957795

#define AZ_EL_INDICATOR_CHAR            '|'
#define END_OF_LINE                    '\n'
#define RADIO_MSG_OFFSET_INIT            -1
#define AZ_EL_LEN                        36
#define AZ_EL_MSG_LEN_MAX   (AZ_EL_LEN - 1)

#define ELEVATION_CORRECTION           5.0

//#define BUTTON_PIN                        3

// Comment or uncomment the following to insert debug (print) statements
//#define DEBUG_AZIMUTH
//#define DEBUG_ELEVATION
//#define DEBUG_GENERAL

#define DEBUG_INITIALIZE
#define DEBUG_SERIAL_COM

// Custom type to manage compass and elevation values
struct DirectionalValues {
    float azimuth;
    float elevation;
};


Servo _servoElevation;
Servo _servoAzimuth;


// Values used to control the servo movement
//  They are coordinated with the compass and elevation values
DirectionalValues _servoLocation;

// destination compass and elevation values received from slave processor
DirectionalValues _targetDir;

// Values captured when the directional antenna initializes
//  They are used to ensure the device does not go out of range physically
DirectionalValues _centerDirectionalValues;

DirectionalValues _currentDirectionalValues;

float _compassMin;
float _compassMax;
float _adjustedCenter;

int _azElMsgInCount = 0;
char _az_el_msg[AZ_EL_MSG_LEN_MAX];


AppTimer _timer;

Adafruit_LSM9DS0 _lsm9ds0 = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

CompassSensor _compass_sensor;

double _elevation_correction = ELEVATION_CORRECTION;

double _servo_elevation_min;
double _servo_elevation_max;
double _degrees_high;
double _degrees_low;
double _servo_move_per_degree;

double get_servo_elevation_target(double target_angle);



void configureLsmSensor()
{
    // 1.) Set the accelerometer range
    _lsm9ds0.setupAccel(_lsm9ds0.LSM9DS0_ACCELRANGE_2G);
    //_lsm9ds0.setupAccel(_lsm9ds0.LSM9DS0_ACCELRANGE_4G);
    //_lsm9ds0.setupAccel(_lsm9ds0.LSM9DS0_ACCELRANGE_6G);
    //_lsm9ds0.setupAccel(_lsm9ds0.LSM9DS0_ACCELRANGE_8G);
    //_lsm9ds0.setupAccel(_lsm9ds0.LSM9DS0_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity
    _lsm9ds0.setupMag(_lsm9ds0.LSM9DS0_MAGGAIN_2GAUSS);
    //_lsm9ds0.setupMag(_lsm9ds0.LSM9DS0_MAGGAIN_4GAUSS);
    //_lsm9ds0.setupMag(_lsm9ds0.LSM9DS0_MAGGAIN_8GAUSS);
    //_lsm9ds0.setupMag(_lsm9ds0.LSM9DS0_MAGGAIN_12GAUSS);

    // 3.) Setup the gyroscope
    _lsm9ds0.setupGyro(_lsm9ds0.LSM9DS0_GYROSCALE_245DPS);
    //_lsm9ds0.setupGyro(_lsm9ds0.LSM9DS0_GYROSCALE_500DPS);
    //_lsm9ds0.setupGyro(_lsm9ds0.LSM9DS0_GYROSCALE_2000DPS);
}


float get_lsm_pitch() {

    // Read the data from the 2 sensors (LSM9DS0 is for pitch, L3GD20H/LSM303 for compass)
    _lsm9ds0.read();

    float t_pitch = (_lsm9ds0.accelData.y * _lsm9ds0.accelData.y) + (_lsm9ds0.accelData.z * _lsm9ds0.accelData.z);
    
    float pitchRadians = (float) atan2(_lsm9ds0.accelData.x, sqrt(t_pitch));
    
    float degrees_of_pitch = DEGREES_PER_RADIAN * pitchRadians;

    degrees_of_pitch += _elevation_correction;

    return degrees_of_pitch;
}


// Watchdog Timer

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


/**************************************************************************/
/*! @brief  Initialises all the sensors used by this example              */
/**************************************************************************/
void init_Sensors()
{

#ifdef DEBUG_INITIALIZE
    Serial.println("Starting Initialization...");
#endif

    _compass_sensor.init_Compass();

    if (!_lsm9ds0.begin())
    {
        /* There was a problem detecting the LSM9DS0 ... check your connections */
        Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
        while (1);
    }
    
#ifdef DEBUG_INITIALIZE
    Serial.println("Initialization Complete...");
#endif

    configureLsmSensor();
}



void initialize_averageDirectionalValues() {
    float tmpAz = 0.0;
    float tmpEl = 0.0;

    for (int i = 0; i < AVERAGE_COUNT_HEADING; i++){

        tmpAz += _compass_sensor.get_heading();

        tmpEl += get_lsm_pitch();

        delay(50);
    }
   
    _centerDirectionalValues.azimuth = tmpAz / AVERAGE_COUNT_HEADING;
    _centerDirectionalValues.elevation = tmpEl / AVERAGE_COUNT_HEADING;

#ifdef DEBUG_INITIALIZE
    Serial.print(">_centerDirVals.elev: "); Serial.println(_centerDirectionalValues.elevation);
#endif

    _targetDir.azimuth = _centerDirectionalValues.azimuth;
    _targetDir.elevation = _centerDirectionalValues.elevation;

#ifdef DEBUG_INITIALIZE
    Serial.print(">_targetDir.elev: "); Serial.println(_targetDir.elevation);
#endif

    _currentDirectionalValues.azimuth = _centerDirectionalValues.azimuth;
    _currentDirectionalValues.elevation = _centerDirectionalValues.elevation;

#ifdef DEBUG_INITIALIZE
    Serial.print(">_centerDirVals.elev: "); Serial.println(_currentDirectionalValues.elevation);
#endif
}



// Center the servo, then take the compass value as the "center"
// Calculate the "in range" compass values
void initialize_compassSettings(float center) {

    if (center - COMPASS_RANGE_HALF < 0){
        _compassMin = 360.0 - (COMPASS_RANGE_HALF - center);
    }
    else {
        _compassMin = center - COMPASS_RANGE_HALF;
    }

    if (center + COMPASS_RANGE_HALF >= 360.0){
        _compassMax = (COMPASS_RANGE_HALF + center) - 360.0;
    }
    else {
        _compassMax = center + COMPASS_RANGE_HALF;
    }

    if (center < _compassMin) {
        _adjustedCenter = center + 360.0;
    }
    else {
        _adjustedCenter = center;
    }

#ifdef DEBUG_AZIMUTH
    Serial.print("Min: "); Serial.print(_compassMin);
    Serial.print(" - Center: "); Serial.print(center);
    Serial.print(" - Max: "); Serial.print(_compassMax);
    Serial.print(" - AdjCenter: "); Serial.print(_adjustedCenter);
#endif
    // Final adjustment to "max" to get a linear set of values
    if ((center < _compassMin) || (_compassMax < center)) {
        _compassMax += 360.0;
    }

#ifdef DEBUG_AZIMUTH
    Serial.print(" - AdjMax: "); Serial.println(_compassMax);
#endif
}




bool calculate_compassGoto(float center, float currentPosition, float gotoPosition, float* distance) {
    
#ifdef DEBUG_AZIMUTH
    Serial.print("\ncalculate_compassGoto: ");
    Serial.print("center: "); Serial.print(center);
    Serial.print(" - currentPosition: "); Serial.print(currentPosition);
    Serial.print(" - gotoPosition(targetDir): "); Serial.println(gotoPosition);
#endif

    float adjustedGoto = gotoPosition;
    float adjustedCurrentPosition = currentPosition;

    if ((center < _compassMin) && (gotoPosition < _compassMin)){
        adjustedGoto += 360.0;
    }

    if (adjustedCurrentPosition < _compassMin) {
        adjustedCurrentPosition += 360.0;
    }

    bool inRange = ((adjustedGoto >= _compassMin) && (adjustedGoto <= _compassMax));

#ifdef DEBUG_AZIMUTH
    Serial.println("\n---------------------------------\n");
    Serial.print("adjustedGoto: "); Serial.print(adjustedGoto);
    Serial.print(" - adjustedCurrentPosition: "); Serial.print(adjustedCurrentPosition);
#endif    
    if (inRange) {
        *distance = adjustedCurrentPosition - adjustedGoto;

#ifdef DEBUG_AZIMUTH
        Serial.print(" - *distance: "); Serial.print(*distance);
#endif
    }

#ifdef DEBUG_AZIMUTH
    Serial.print(" - inRange: "); Serial.println(inRange);
    Serial.println("\n---------------------------------\n");
#endif

    return inRange;
}


float calculate_newAzimuthServoPosition(float distanceToMove, float currentPosition) {

    float newPosition = ((distanceToMove / COMPASS_RANGE_TOTAL) * SERVO_AZIMUTH_POSITION_COUNT) * AZIMUTH_MOVE_PERCENTAGE;

#ifdef DEBUG_AZIMUTH
    Serial.println("\ncalculate_newAzimuthServoPosition:");
    Serial.print(" -  currentPosition: "); Serial.print(currentPosition);
    Serial.print(" -  new Position: "); Serial.print(newPosition + currentPosition);
    Serial.println();
#endif

    return newPosition + currentPosition;
}


//float calculate_elevation_distance(float destinatinoElevation, float currentElevation) {
//    return currentElevation - destinatinoElevation;
//}

//float calculate_newElevationServoPosition(float destinatinoElevation, float currentElevation, float currentServoPosition, float* distance) {
//
//    float degreesToMove = currentElevation - destinatinoElevation;
//
//    *distance = (degreesToMove * ELEVATION_MOVE_PERCENTAGE) * 0.25;
//
//#ifdef DEBUG_AZIMUTH
//    Serial.println("\ncalculate_newElevationServoPosition:");
//    Serial.print(">>> destinatinoElevation: "); Serial.print(destinatinoElevation);
//    Serial.print(" -  currentElevation: "); Serial.print(currentElevation);
//    Serial.print(" -  currentServoPosition: "); Serial.print(currentServoPosition);
//    Serial.print(" -  distance: "); Serial.print(*distance);
//    Serial.println();
//#endif
//  
//    return currentServoPosition + *distance;
//}


float smoothData(float dataVal, float lastVal)
{
    float result;
    
    if (lastVal < FLOAT_EMPTY_LIMIT) {
        result = dataVal;
    }
    else {
        const float dt = (1.0 / 2.0);
        const double RC = 0.4;
        const double alpha = dt / (RC + dt);

        float x;
        x = (alpha * dataVal) + (1.0 - alpha) * (lastVal);

        result = x;
    }

    return result;
}

#define ELEVATION_LIMIT_COARSE  (10.0)
#define ELEVATION_LIMIT_FINE    (2.0)


void move_elevation_servo(double newElevation) {
    if ((newElevation >= _servo_elevation_min) && (newElevation <= _servo_elevation_max)) {

        _servoLocation.elevation = newElevation;
        _servoElevation.write(_servoLocation.elevation);
    }
}

int get_sign(double value) {
    if (value < 0.0) return -1;
    return 1;
}



void move_servos() {
    float distance;
    bool result;

    float heading = _compass_sensor.get_heading();
    float pitch = get_lsm_pitch();

    Serial.print("heading: "); Serial.print(heading);
    Serial.print("  -  pitch: "); Serial.print(pitch);

    _currentDirectionalValues.azimuth = smoothData(heading, _currentDirectionalValues.azimuth);
    _currentDirectionalValues.elevation = smoothData(pitch, _currentDirectionalValues.elevation);

    Serial.print("  ::  azimuth: "); Serial.print(_currentDirectionalValues.azimuth);
    Serial.print("  -  elevation: "); Serial.print(_currentDirectionalValues.elevation);
    Serial.println();

    if (_targetDir.azimuth > FLOAT_EMPTY_LIMIT) {
        result = calculate_compassGoto(_centerDirectionalValues.azimuth, _currentDirectionalValues.azimuth, _targetDir.azimuth, &distance);

        if (result && ((distance > 3.0) || (distance < -3.0))) {

            distance = distance * 0.5;

            //Serial.print("\n---> _servoLocation.azimuth: "); Serial.print(_servoLocation.azimuth);
            float newAzimuth = calculate_newAzimuthServoPosition(distance, _servoLocation.azimuth);

            //Serial.print("\n Distance to move: ");
            //Serial.println(newAzimuth - _servoLocation.azimuth);


            if ((newAzimuth >= SERVO_AZIMUTH_MIN) && (newAzimuth <= SERVO_AZIMUTH_MAX)) {
                _servoLocation.azimuth = newAzimuth;

               // Serial.print("\n---> _servoLocation.azimuth: "); Serial.print(_servoLocation.azimuth);

                _servoAzimuth.write(_servoLocation.azimuth);
            }
        }
    }

    if (_targetDir.elevation > FLOAT_EMPTY_LIMIT){


        // distance in degrees
        float elevDistanceInDegrees = _currentDirectionalValues.elevation - _targetDir.elevation;
        
        float absElevDistanceInDegrees = abs(elevDistanceInDegrees);
        //Serial.print("elevDistanceInDegrees: "); Serial.println(elevDistanceInDegrees);
        //Serial.print("absElevDistanceInDegrees: "); Serial.println(absElevDistanceInDegrees);

        //float newElevation = calculate_newElevationServoPosition(_targetDir.elevation, _currentDirectionalValues.elevation, _servoLocation.elevation, &elevDistance);

        if (absElevDistanceInDegrees > ELEVATION_LIMIT_COARSE) {
            float movement = get_servo_elevation_target(_targetDir.elevation);
            //Serial.print("Coarse Move: "); Serial.println(movement);
            move_elevation_servo(movement);

        }
        else if (absElevDistanceInDegrees > ELEVATION_LIMIT_FINE) {
            
            int signOfDiff = get_sign(elevDistanceInDegrees);

            float finemove = _servoLocation.elevation - (_servo_move_per_degree * signOfDiff);
            //Serial.print("Fine Move: "); Serial.println(finemove);
            move_elevation_servo(finemove);

        }
    }
}



void display_servo_data() {

    float azimuthDiff = _currentDirectionalValues.azimuth - _targetDir.azimuth;
    float elevationDiff = _currentDirectionalValues.elevation - _targetDir.elevation;

#ifdef DEBUG_GENERAL
    Serial.println("\n---------------------------------\n");

    Serial.print("Center Azimuth: ");
    Serial.print(_centerDirectionalValues.azimuth);
    Serial.print(" - Center Elevation: ");
    Serial.println(_centerDirectionalValues.elevation);
    Serial.println();

    Serial.print("azimuthDiff: ");
    Serial.print(azimuthDiff);
    Serial.print(" - elevationDiff: ");
    Serial.println(elevationDiff);
    Serial.println();

    Serial.print("_targetDir.azimuth: ");
    Serial.print(_targetDir.azimuth);
    Serial.print(" - _targetDir.elevation: ");
    Serial.print(_targetDir.elevation);
    Serial.println();

    Serial.print("actualAzimuth: ");
    Serial.print(_currentDirectionalValues.azimuth);
    Serial.print(" - actualElevation: ");
    Serial.println(_currentDirectionalValues.elevation);

    Serial.println("\n---------------------------------\n");
#endif
}


void delay_with_update(int8_t seconds) {

    while (seconds) {

        _timer.update();

        delay(1000);

        seconds--;
    }
}




double get_servo_elevation_target(double target_angle) {
    return ((target_angle - _degrees_low) * _servo_move_per_degree) + SERVO_ELEVATION_LOW;
}

void center_and_initialize() {

    Serial.println("Resetting");

    _servoLocation.azimuth = SERVO_AZIMUTH_CENTER;
    _servoLocation.elevation = SERVO_ELEVATION_CENTER;

    _servoAzimuth.write(_servoLocation.azimuth);


    _servoElevation.write(SERVO_ELEVATION_LOW);

    delay_with_update(4);

    _degrees_low = get_lsm_pitch();

    Serial.print("_degrees_low: "); Serial.println(_degrees_low);

    _servoElevation.write(SERVO_ELEVATION_HIGH);

    delay_with_update(4);

    _degrees_high = get_lsm_pitch();

    Serial.print("_degrees_high: "); Serial.println(_degrees_high);

    _servo_move_per_degree = (SERVO_ELEVATION_LOW - SERVO_ELEVATION_HIGH) / (_degrees_low - _degrees_high);

    Serial.print("_servo_move_per_degree: "); Serial.println(_servo_move_per_degree);

    _servoElevation.write(SERVO_ELEVATION_CENTER);

    delay_with_update(3);

    initialize_averageDirectionalValues();

    initialize_compassSettings(_centerDirectionalValues.azimuth);

    _timer.update();

}

//
//void center_and_initialize2() {
//
//    Serial.println("Resetting");
//
//    _servoLocation.azimuth = SERVO_AZIMUTH_CENTER;
//    _servoLocation.elevation = SERVO_ELEVATION_CENTER;
//
//    _servoAzimuth.write(_servoLocation.azimuth);
//    _servoElevation.write(_servoLocation.elevation);
//    
//    _timer.update();
//
//    delay(1000);
//
//    _timer.update();
//
//    delay(1000);
//
//    _timer.update();
//
//    delay(1000);
//
//    initialize_averageDirectionalValues();
//
//    initialize_compassSettings(_centerDirectionalValues.azimuth);
//
//    _timer.update();
//}

void setup() {

    Serial.println("Wire Begin");

    Wire.begin();

    Serial.println("Begin");
    Serial.begin(SERIAL_BAUD_RATE);  // start serial for output
     

    digitalWrite(SERVO_RELAY_PIN, HIGH);

    Serial.println("Pin Setup");

    pinMode(SERVO_RELAY_PIN, OUTPUT);

    

    _targetDir.elevation = FLOAT_EMPTY_FLAG;
    _targetDir.azimuth = FLOAT_EMPTY_FLAG;
    _currentDirectionalValues.azimuth = FLOAT_EMPTY_FLAG;
    _currentDirectionalValues.elevation = FLOAT_EMPTY_FLAG;


    _servo_elevation_min = min(SERVO_ELEVATION_HIGH, SERVO_ELEVATION_LOW);
    _servo_elevation_max = max(SERVO_ELEVATION_HIGH, SERVO_ELEVATION_LOW);

    _compass_sensor.setMinMax(-643, -737, -868, 713, 711, 531);
    
    init_Sensors();


#ifdef DEBUG_INITIALIZE
    Serial.println("Setting servo values");
#endif

    _servoLocation.azimuth = SERVO_AZIMUTH_CENTER;
    _servoLocation.elevation = SERVO_ELEVATION_CENTER;

    _servoAzimuth.write(_servoLocation.azimuth);
    _servoElevation.write(_servoLocation.elevation);

    _servoElevation.attach(SERVO_ELEVATION_PIN);
    _servoAzimuth.attach(SERVO_AZIMUTH_PIN);
 

#ifdef DEBUG_INITIALIZE
    Serial.println("Relay On");
#endif
    digitalWrite(SERVO_RELAY_PIN, LOW);

    delay(4000);


    center_and_initialize();

    watchdogSetup();
    _timer.every(1000, watchdog_reset);
    _timer.every(150, move_servos);

#ifdef DEBUG_INITIALIZE
    Serial.println(" Initialization Complete.");
#endif

    delay(1000);
}




void loop() {
   
    _timer.update();

    // Data from ground station 
    while (Serial.available() > 0) {

        char incomingByte = (char) Serial.read();
        
        Serial.print(incomingByte);

        // AzEl Data
        if (incomingByte == AZ_EL_INDICATOR_CHAR) {
            _azElMsgInCount = RADIO_MSG_OFFSET_INIT;
        }

        _azElMsgInCount++;

        if (_azElMsgInCount >= AZ_EL_MSG_LEN_MAX) {
            _azElMsgInCount = RADIO_MSG_OFFSET_INIT;

            Serial.print("#Data 2 Error: msg > "); Serial.print(AZ_EL_LEN); Serial.println(" bytes");
        }

        _az_el_msg[_azElMsgInCount] = incomingByte;

        if ((incomingByte == '\n') || (incomingByte == '\\')) {

            _az_el_msg[_azElMsgInCount] = 0;

            _azElMsgInCount = RADIO_MSG_OFFSET_INIT;

                // Received telemetry so pull out lat/lon/alt for positioning
            if (_az_el_msg[0] == AZ_EL_INDICATOR_CHAR) {

                if ((_az_el_msg[4] == '.') && (_az_el_msg[6] == ',') && (_az_el_msg[10] == '.') && (strlen(_az_el_msg) == 12)) {
                    
                    // MESSAGE Structure:  |###.#,###.#\n
                    _az_el_msg[6] = '\0';
                
                    //Serial.println("CHANGING AZIMUTH/ELEVATION...");

                    _targetDir.azimuth = atof(&_az_el_msg[1]);
                    _targetDir.elevation = atof(&_az_el_msg[7]);

#ifdef DEBUG_SERIAL_COM
                    Serial.print(" Az: "); Serial.print(_targetDir.azimuth);
                    Serial.print(" El: "); Serial.print(_targetDir.elevation);
                    Serial.println();
#endif
                }
                else {
                    Serial.print("Bad Data: "); Serial.println(_az_el_msg);
                }

            }
        }
    }

    // Reset!
    //int buttonState = digitalRead(BUTTON_PIN);

    //if (buttonState == LOW) {
    //    center_and_initialize();
    //}

    delay(75);
}


