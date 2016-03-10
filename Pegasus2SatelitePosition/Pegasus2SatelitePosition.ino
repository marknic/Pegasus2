
#include <IridiumSBD/IridiumSBD.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus/TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <PString/PString.h> // String buffer formatting: http://arduiniana.org

#ifndef FALSE
#define FALSE   (1==0)
#endif

#ifndef TRUE
#define TRUE   (1==1)
#endif

#define TRANSMIT_AFTER_COUNT        25
#define GPS_DELAY_MS                2000

#define ROCKBLOCK_RX_PIN            8
#define ROCKBLOCK_TX_PIN            9
#define ROCKBLOCK_SLEEP_PIN         10

#define ROCKBLOCK_BAUD              19200
#define GPS_BAUD                    9600
#define CONSOLE_BAUD                115200
#define LED_PIN                     13

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);

IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);

TinyGPSPlus tinygps;

uint32_t timer = millis();
uint8_t _gps_got_fix = FALSE;
uint8_t _gps_got_fix_for_transmit = FALSE;
uint16_t _transmit_counter = TRANSMIT_AFTER_COUNT;

char _gps_data_string[60];


int _sat_return_val = 0;
int _sat_signal_quality;
int _sat_message_count;

void setup()
{
    // Start the serial ports
    Serial.begin(CONSOLE_BAUD);
    Serial1.begin(GPS_BAUD);

    ssIridium.begin(ROCKBLOCK_BAUD);

    pinMode(LED_PIN, OUTPUT);

    //Serial.println("Attach console");
    isbd.attachConsole(Serial);

    //Serial.println("Attach diags");
    isbd.attachDiags(Serial);

    //Serial.println("Set power Profile");
    isbd.setPowerProfile(0);

    //Serial.println("Begin");
    _sat_return_val = isbd.begin(); // Wake up the 9602 and prepare it to communicate.

    //if (_sat_return_val == ISBD_SUCCESS)
    //{
    //    Serial.println("Satelite Startup Complete");
    //} else
    //{
    //    Serial.println("Satelite Startup FAILED");
    //}

    isbd.useMSSTMWorkaround(false);


    // Step 1: Reset TinyGPS++ and begin listening to the GPS
    tinygps = TinyGPSPlus();
    
    //                         LAT      LON    ALT   SPD DIR
    strcpy(_gps_data_string, "00.0000,000.0000,000.0,0.0,000");
}


bool ISBDCallback()
{
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    while (Serial1.available())
    {
        tinygps.encode(Serial1.read());

        _gps_got_fix = tinygps.location.isValid() && tinygps.date.isValid() &&
            tinygps.time.isValid() && tinygps.altitude.isValid();
    }

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > GPS_DELAY_MS) {
        timer = millis(); // reset the timer

        if (_gps_got_fix)
        {
            _gps_got_fix_for_transmit = TRUE;

            _gps_data_string[0] = '\0';

            PString str(_gps_data_string, sizeof(_gps_data_string));
            str.print(tinygps.location.lat(), 6);
            str.print(",");
            str.print(tinygps.location.lng(), 6);
            str.print(",");
            str.print(tinygps.altitude.meters(), 1);
            str.print(",");
            str.print(tinygps.speed.mph(), 1);
            str.print(",");
            str.print(tinygps.course.value() / 100);
        }

        Serial.print("_gps_data_string: ");
        Serial.println(_gps_data_string);

        _transmit_counter++;
    }

    return true;
}


void loop()
{

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();


    while (Serial1.available())
    {
        tinygps.encode(Serial1.read());

        _gps_got_fix = tinygps.location.isValid() && tinygps.date.isValid() &&
            tinygps.time.isValid() && tinygps.altitude.isValid();
    }


    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > GPS_DELAY_MS) {
        
        timer = millis(); // reset the timer

        if (_gps_got_fix)
        {
            _gps_got_fix_for_transmit = TRUE;

            _gps_data_string[0] = '\0';

            PString str(_gps_data_string, sizeof(_gps_data_string));
            str.print(tinygps.location.lat(), 6);
            str.print(",");
            str.print(tinygps.location.lng(), 6);
            str.print(",");
            str.print(tinygps.altitude.meters(), 1);
            str.print(",");
            str.print(tinygps.speed.mph(), 1);
            str.print(",");
            str.print(tinygps.course.value() / 100);
        }

        Serial.print("_gps_data_string: ");
        Serial.println(_gps_data_string);

        _transmit_counter++;

        //_sat_return_val = isbd.getSignalQuality(_sat_signal_quality);
        //
        //if (_sat_return_val != ISBD_SUCCESS)
        //{
        //    Serial.print("Error: SAT - Get Signal Quality> ");
        //    Serial.print(_sat_return_val);
        //}
        //else
        //{
        //    Serial.print("Signal Quality: "); Serial.println(_sat_return_val);
        //}

        Serial.println("Checking to see if it is time to transmit!");

        // We have GPS data AND we've waited for the transmit interval
        if (_gps_got_fix_for_transmit && (_transmit_counter >= TRANSMIT_AFTER_COUNT))
        {
            Serial.println(">>> DO SATELITE TRANSMISSION! <<<");

            _sat_return_val = isbd.sendSBDText(_gps_data_string);

            if (_sat_return_val != ISBD_SUCCESS)
            {
                Serial.print("Error: SAT - sendSBDText> ");
                Serial.print(_sat_return_val);
            } else
            {
                _transmit_counter = 0;
            }
        }
    }




    //bool fixFound = false;
    //unsigned long loopStartTime = millis();

    ////ssGPS.listen();

    //// Step 2: Look for GPS signal for up to 7 minutes
    //for (unsigned long now = millis(); !fixFound && millis() - now < 7UL * 60UL * 1000UL;) {
    //    if (Serial1.available())
    //    {
    //        tinygps.encode(Serial1.read());

    //        fixFound = tinygps.location.isValid() && tinygps.date.isValid() &&
    //            tinygps.time.isValid() && tinygps.altitude.isValid();
    //    }
    //}

    //Serial.println(fixFound ? F("A GPS fix was found!") : F("No GPS fix was found."));

    //// Step 3: Start talking to the RockBLOCK and power it up
    //Serial.println("Beginning to talk to the RockBLOCK...");
    //ssIridium.listen();

    //if (isbd.begin() == ISBD_SUCCESS)
    //{
    //    char outBuffer[60]; // Always try to keep message short
    //    if (fixFound)
    //    {
    //        sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,",
    //            tinygps.date.year(), tinygps.date.month(), tinygps.date.day(),
    //            tinygps.time.hour(), tinygps.time.minute(), tinygps.time.second());
    //        int len = strlen(outBuffer);
    //        PString str(outBuffer, sizeof(outBuffer) - len);
    //        str.print(tinygps.location.lat(), 6);
    //        str.print(",");
    //        str.print(tinygps.location.lng(), 6);
    //        str.print(",");
    //        str.print(tinygps.altitude.meters());
    //        str.print(",");
    //        str.print(tinygps.speed.knots(), 1);
    //        str.print(",");
    //        str.print(tinygps.course.value() / 100);
    //    }
    //    else
    //    {
    //        sprintf(outBuffer, "No GPS fix found!");
    //    }

    //    Serial.print("Transmitting message: ");
    //    Serial.println(outBuffer);
    //    //isbd.sendSBDText(outBuffer);
    //}


}

















































//
//
//#include <Adafruit_GPS/Adafruit_GPS.h>
//
//#include <SoftwareSerial/SoftwareSerial.h>
//#include <IridiumSBD/IridiumSBD.h>
//
//
//#ifndef FALSE
//#define FALSE   (1==0)
//#endif
//
//#ifndef TRUE
//#define TRUE   (1==1)
//#endif
//
//#define TRANSMIT_AFTER_COUNT    25
//#define GPS_DELAY_MS            2000
//
//#define RX_PIN                  8
//#define TX_PIN                  9
//#define RADIO_ON_PIN            10
//#define LED_PIN                 13
//#define ROCKBLOCK_BAUD          19200
//#define GPS_BAUD                9600
//
////SoftwareSerial gpsSerial(RX_PIN, TX_PIN); // RockBLOCK serial port on 8/9
//
//SoftwareSerial ssIridium(RX_PIN, TX_PIN); // RockBLOCK serial port on 8/9
//
//IridiumSBD isbd(ssIridium, RADIO_ON_PIN);   // RockBLOCK SLEEP pin on 10
//
//Adafruit_GPS GPS(&Serial1);
//
//// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
//// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
//#define GPSECHO  true
//
//// this keeps track of whether we're using the interrupt
//// off by default!
//boolean usingInterrupt = false;
//
//
//uint32_t timer = millis();
//uint8_t _got_fix = FALSE;
//uint8_t _got_fix_for_transmit = FALSE;
//uint16_t _transmit_counter = TRANSMIT_AFTER_COUNT;
//
//char _gps_data_string[50];
//
//char _gps_string01[10];
//char _gps_string02[10];
//char _gps_string03[10];
//char _gps_string04[10];
//char _gps_string05[10];
//
//int _sat_return_val = 0;
//int _sat_signal_quality;
//int _sat_message_count;
//
//
//void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
//
//
//bool ISBDCallback()
//{
//    return true;
//}
//
//void setup()
//{
//    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
//    // also spit it out
//    Serial.begin(115200);
//
//    ssIridium.begin(ROCKBLOCK_BAUD);
//
//    pinMode(LED_PIN, OUTPUT);
//
//    Serial.println("Set power Profile");
//    isbd.setPowerProfile(0);
//
//    Serial.println("Attach console");
//    isbd.attachConsole(Serial);
//    
//    Serial.println("Attach diags");
//    isbd.attachDiags(Serial);
//
//
//    Serial.println("Begin");
//    isbd.begin(); // Wake up the 9602 and prepare it to communicate.
//
//    
//
//    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
//    GPS.begin(GPS_BAUD);
//
//    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
//    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//    // uncomment this line to turn on only the "minimum recommended" data
//    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//    
//    // Set the update rate
//    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
//
//    // Request updates on antenna status, comment out to keep quiet
//    //GPS.sendCommand(PGCMD_ANTENNA);
//
//    // the nice thing about this code is you can have a timer0 interrupt go off
//    // every 1 millisecond, and read data from the GPS for you. that makes the
//    // loop code a heck of a lot easier!
//    useInterrupt(true);
//
//    strcpy(_gps_data_string, "00.0000, -00.0000,  000.0,  0.0,000.0");
//
//    delay(1000);
//
//    // Ask for firmware version
//    //Serial1.println(PMTK_Q_RELEASE);
//}
//
//
//// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
//SIGNAL(TIMER0_COMPA_vect) {
//    char c = GPS.read();
//    // if you want to debug, this is a good time to do it!
//#ifdef UDR0
//    if (GPSECHO)
//        if (c) UDR0 = c;
//    // writing direct to UDR0 is much much faster than Serial.print 
//    // but only one character can be written at a time. 
//#endif
//}
//
//void useInterrupt(boolean v) {
//    if (v) {
//        // Timer0 is already used for millis() - we'll just interrupt somewhere
//        // in the middle and call the "Compare A" function above
//        OCR0A = 0xAF;
//        TIMSK0 |= _BV(OCIE0A);
//        usingInterrupt = true;
//    }
//    else {
//        // do not call the interrupt function COMPA anymore
//        TIMSK0 &= ~_BV(OCIE0A);
//        usingInterrupt = false;
//    }
//}
//
//
//
//
//void loop()                     // run over and over again
//{
//    // in case you are not using the interrupt above, you'll
//    // need to 'hand query' the GPS, not suggested :(
//    if (!usingInterrupt) {
//        // read data from the GPS in the 'main loop'
//        char c = GPS.read();
//        // if you want to debug, this is a good time to do it!
//        if (GPSECHO)
//            if (c) Serial.print(c);
//    }
//
//    // if a sentence is received, we can check the checksum, parse it...
//    if (GPS.newNMEAreceived()) {
//        // a tricky thing here is if we print the NMEA sentence, or data
//        // we end up not listening and catching other sentences! 
//        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
//
//        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
//            return;  // we can fail to parse a sentence in which case we should just wait for another
//    }
//
//    // if millis() or timer wraps around, we'll just reset it
//    if (timer > millis())  timer = millis();
//
//    // approximately every 2 seconds or so, print out the current stats
//    if (millis() - timer > GPS_DELAY_MS) {
//        timer = millis(); // reset the timer
//
//                          //$:2016-02-16T18:13:24Z,
//                          //Serial.print("\nTime: ");
//                          //    Serial.print(GPS.year, DEC); Serial.print('-');
//                          //    Serial.print(GPS.month, 2); Serial.print('-');
//                          //    Serial.print(GPS.day, 2);
//                          //    Serial.print("T");
//                          //    Serial.print(GPS.hour, 2); Serial.print(':');
//                          //    Serial.print(GPS.minute, 2); Serial.print(':');
//                          //    Serial.print(GPS.seconds, 2); 
//                          //    Serial.print("Z,");
//
//        _got_fix = GPS.fix;
//
//        if (_got_fix) {
//            
//            _got_fix_for_transmit = TRUE;
//
//            dtostrf(GPS.latitudeDegrees, 9, 4, _gps_string01);
//            dtostrf(GPS.longitudeDegrees, 9, 4, _gps_string02);
//            dtostrf(GPS.altitude, 7, 1, _gps_string03);
//            dtostrf(GPS.speed, 5, 1, _gps_string04);
//            dtostrf(GPS.angle, 5, 1, _gps_string05);
//
//            sprintf(_gps_data_string, "%s,%s,%s,%s,%s", _gps_string01, _gps_string02, _gps_string03, _gps_string04, _gps_string05);
//        }
//
//        _transmit_counter++;
//
//        _sat_return_val = isbd.getSignalQuality(_sat_signal_quality);
//
//        if (_sat_return_val != ISBD_SUCCESS)
//        {
//            Serial.print("Error: SAT - Get Signal Quality> ");
//            Serial.print(_sat_return_val);
//        } else
//        {
//            Serial.print("Signal Quality: "); Serial.println(_sat_return_val);
//        }
//
//
//
//        // We have GPS data AND we've waited for the transmit interval
//        if (_got_fix_for_transmit && (_transmit_counter >= TRANSMIT_AFTER_COUNT))
//        {
//            Serial.println(">>> DO SATELITE TRANSMISSION! <<<");
//
//            //_sat_return_val = isbd.sendSBDText(_gps_data_string);
//
//            //if (_sat_return_val != ISBD_SUCCESS)
//            //{
//            //    Serial.print("Error: SAT - sendSBDText> ");
//            //    Serial.print(_sat_return_val);
//            //}
//            
//            _transmit_counter = 0;
//        }
//
//        Serial.println(_gps_data_string);
//    }
//}