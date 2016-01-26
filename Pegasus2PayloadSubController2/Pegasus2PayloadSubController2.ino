#include "Timer.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <SparkFunHTU21D.h>
#include <SparkFun_MS5803_I2C.h>
#include <Adafruit_MAX31855.h>
#include <avr/wdt.h>

#ifndef TRUE
#define TRUE                             1==1
#endif
#ifndef FALSE
#define FALSE                            0==1
#endif

#define DEBUG_DISPLAY                   TRUE

#define TMP_36_PIN                         A0

#define DO                                  4
#define CS                                  5
#define CLK                                 6

#define REFERENCE_VOLTAGE                4.96
#define ADC_COUNT                      1024.0
#define VOLTAGE_RATIO       (REFERENCE_VOLTAGE / ADC_COUNT)

void sendSensorSamples();
void configureSensors(void);

Adafruit_MAX31855 thermocouple(CLK, CS, DO);


/* Assign a unique base ID for this sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

Timer _timer;

//Create an instance of the object
HTU21D myHumidity;

//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77
MS5803 pressure_sensor(ADDRESS_HIGH);


char sensorData[160];

float pressure_temperature_c;
float pressure_abs;
float pressure_baseline;
float humidity;
float humidity_temp;
double thermocouple_temp_c;


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
    Wire.begin();

    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LED_BUILTIN, OUTPUT);

    // Set up Serial Communication
#if (DEBUG_DISPLAY)
    Serial.begin(115200);
#endif

    Serial1.begin(9600);  // Geiger Counter Input Data

    /* Setup the sensor gain and integration time */
    configureSensors();

    watchdogSetup();

    _timer.every(1000, watchdog_reset);
    _timer.every(250, sendSensorSamples);

    // Just for testing
    //_timer.every(1000, tick);

#if (DEBUG_DISPLAY)
    Serial.println("end of setup");
#endif
}

void loop()
{
    _timer.update();
}


//void tick()
//{
//    Serial.println("tick");
//}


char scratch01[8];
char scratch02[8];
char scratch03[8];
char scratch04[12];
char scratch05[12];
char scratch06[12];
char scratch07[12];
char scratch08[12];
char scratch09[12];
char scratch10[12];
char scratch11[12];
char scratch12[12];
char scratch13[12];
char scratch14[12];


// This can run at max, 4x/sec (250ms)
void sendSensorSamples() {

    lsm.read();

    humidity = myHumidity.readHumidity();
    humidity_temp = myHumidity.readTemperature();

    // Read temperature from the sensor in deg C. This operation takes about 
    pressure_temperature_c = pressure_sensor.getTemperature(CELSIUS, ADC_512);

    // Read pressure from the sensor in mbar.
    pressure_abs = pressure_sensor.getPressure(ADC_4096);

    thermocouple_temp_c = thermocouple.readCelsius();

    if (isnan(thermocouple_temp_c)) {
        thermocouple_temp_c = -999.0;
    }

    //getting the voltage reading from the temperature sensor
    int tmpReading = analogRead(TMP_36_PIN);
   
    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage_value = tmpReading * VOLTAGE_RATIO;

    // now print out the temperature
    float temperatureC = (voltage_value - 0.5) * 100;  //converting from 10 mv per degree wit 500 mV offset
                                                     //to degrees ((tmp_voltage - 500mV) times 100)
    sprintf(sensorData, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
        dtostrf(pressure_abs, 3, 1, scratch01),
        dtostrf(pressure_temperature_c, 3, 1, scratch02),  
        dtostrf(humidity, 3, 1, scratch03),
        dtostrf(thermocouple_temp_c, 3, 1, scratch04),
        dtostrf(lsm.accelData.x, 3, 1, scratch05),
        dtostrf(lsm.accelData.y, 3, 1, scratch06),
        dtostrf(lsm.accelData.z, 3, 1, scratch07),
        dtostrf(lsm.gyroData.x, 3, 1, scratch08),
        dtostrf(lsm.gyroData.y, 3, 1, scratch09),
        dtostrf(lsm.gyroData.z, 3, 1, scratch10),
        dtostrf(lsm.magData.x, 3, 1, scratch11),
        dtostrf(lsm.magData.y, 3, 1, scratch12),
        dtostrf(lsm.magData.z, 3, 1, scratch13),
        dtostrf(temperatureC, 3, 1, scratch14)
        );


    Serial1.print(sensorData);
    Serial1.print('\n');

#if (DEBUG_DISPLAY)
    Serial.print("--");
    Serial.print(sensorData);
    Serial.print('\n');
#endif

}


void configureSensors(void)
{
    // 1.) Set the accelerometer range
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);


    /* Initialise the sensor */
    if (!lsm.begin())
    {
#if (DEBUG_DISPLAY)
        /* There was a problem detecting the LSM9DS0 ... check your connections */
        Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
#endif

        //while (1);
        _timer.oscillate(LED_BUILTIN, 200, HIGH);
    }

#if (DEBUG_DISPLAY)
    Serial.println(F("Found LSM9DS0 9DOF"));
#endif

    myHumidity.begin();

    pressure_sensor.reset();
    pressure_sensor.begin();

    pressure_baseline = pressure_sensor.getPressure(ADC_4096);
}
