#include <Wire.h>
#include <string.h> // strlen
#include <Adafruit_NeoPixel.h>
#include "NeoPatterns.h"
#include <Timer.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>


// pin definition for the Uno
#define USER_MSG_MAX_LEN                                        80
#define USER_MSG_ADJUSTED_MAX_LEN          (USER_MSG_MAX_LEN + 20)

#define I2C_ADDRESS                                           0x06

#define NEOPIXEL_PIN                                             6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS                                               25

#ifndef TRUE
#define TRUE                                                     1
#endif

#ifndef FALSE
#define FALSE                                                    0
#endif

void Ring1Complete();

char _display_text[USER_MSG_ADJUSTED_MAX_LEN];

Timer _timer;

NeoPatterns Ring1(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800, &Ring1Complete);

char* _startup_text = "-- PEGASUS II -- Time to fly!";
char* _thank_msr = "PEGASUS II thanks our partnership with Microsoft Research!";
char* _people_of_earth = "People of Earth, We mean you no harm!  No, Really!";
char* _pegasus = "PEGASUS II Flight Leads Mark Nichols & Matt Long";
char* _himom = "Hi Mom! I'm in near space!";
char* _reached_100k = "PEGASUS II: We reached our altitude goal... 100,000 Feet!";
char* _going_down = "Going down!";
char* _going_up = "Going up!";


int userTextBytesRead = 0;
uint8_t displayTextLength;

// Ring1 Completion Callback
void Ring1Complete()
{
    Ring1.Reverse();
}


void ring_update() {

    Ring1.Update();
}


void leds_on(bool onOff){

    Serial.print("LED's ");
    
    if (onOff) {
      Serial.println("ON!");
    }
    else {
      Serial.println("OFF!");
    }
    
    if (onOff) {
        Ring1.setBrightness(255);
    }
    else {
        Ring1.setBrightness(0);
    }
}


void display_text(char* text_to_display)
{
    if (text_to_display == NULL) return;

    Serial1.println(text_to_display);

    Serial.print("Sending Text: '");
    Serial.print(text_to_display);
    Serial.println("'");
}


// callback for received data
void receiveData(int byteCount) {

    Serial.print("byteCount: ");
    Serial.println(byteCount);

    if (byteCount == 1) {
        int command = Wire.read();

        switch (command) {
            case 0:         // Reset Things
                Serial.println("Reset");
                _display_text[0] == '\0';
                userTextBytesRead = 0;
                displayTextLength = 0;
                break;
            case 1:         // Turn LEDs On
                Serial.println("Turn LEDs On");
                leds_on(TRUE);
                break;
            case 2:         // Turn LEDs Off
                Serial.println("Turn LEDs Off");
                leds_on(FALSE);
                break;
            case 3:         // Display MSR Thanks
                Serial.println("Display 3");
                display_text(_thank_msr);
                break;
            case 4:         // Display People of Earth
                Serial.println("Display 4");
                display_text(_people_of_earth);
                break;
            case 5:         // Display Pegasus Flight Team
                Serial.println("Display 5");
                display_text(_pegasus);
                break;
            case 6:         // Display Hi Mom
                Serial.println("Display 6");
                display_text(_himom);
                break;
            case 7:         // Display Reached 100,000
                Serial.println("Display 7");
                display_text(_reached_100k);
                break;
            case 8:         // Display Going Down
                Serial.println("Display 8");
                display_text(_going_down);
                break;            
            case 9:         // Display Going Up
                Serial.println("Display 9");
                display_text(_going_up);
                break;
            default:
                Serial.print("default - error: # ");
                Serial.println(command);
                break;
        }

        _display_text[0] == '\0'; // Clear this just to make sure

    }
    else {

        int byteCountToRead = byteCount;

        if (_display_text[0] == '\0') {
            displayTextLength = (uint8_t) Wire.read();

            Serial.print("message length: "); Serial.println(displayTextLength);
            byteCountToRead--;
        }

        Serial.print("userTextBytesRead: "); Serial.println(userTextBytesRead);
        size_t size = Wire.readBytes(&_display_text[userTextBytesRead], byteCountToRead);

        Serial.print("size: "); Serial.println(size);

        userTextBytesRead += size;

        Serial.print("userTextBytesRead: "); Serial.println(userTextBytesRead);
        
        if (displayTextLength == userTextBytesRead) {

            _display_text[userTextBytesRead] = '\0';

            Serial.print("Received for display: ");
            Serial.println(_display_text);

            display_text(_display_text);

            _display_text[0] = '\0';
            userTextBytesRead = 0;
            displayTextLength = 0;
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
    WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);  // 2 seconds
    //WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP0);  // 8 seconds

    sei(); // enable interrupts
}


void watchdog_reset() {
    wdt_reset();
}


void turn_leds_off() {
    leds_on(FALSE);
}

void tick() {
    Serial.println("-");
}

void display_startup_text() {
    display_text(_startup_text);
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600);

    Serial.println("Starting...");
    
    Serial.println("I2C...");
    
    Wire.begin(I2C_ADDRESS);

    Wire.onReceive(receiveData);

    Serial.println("NeoPixels...");
    
    Ring1.begin();
    
    Serial.println("Theater Chase...");
    
    // Kick off a pattern
    Ring1.TheaterChase(Ring1.Color(255, 255, 0), Ring1.Color(0, 0, 0), 100);

    Serial.println("Watchdog...");
    
    watchdogSetup();

    Serial.println("Timers...");
    
    _timer.every(250, ring_update);

    _timer.after(8000, turn_leds_off);

    _timer.every(1000, watchdog_reset);

    _timer.oscillate(LED_BUILTIN, 1000, HIGH, 10);

    Serial.println("Startup Text...");
    
    _timer.after(10000, display_startup_text);
}


void loop()
{
    _timer.update();
}

