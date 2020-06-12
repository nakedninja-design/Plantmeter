 /*
 * Sleep Program for Plantmeter ATtiny85
 * 
 * Company:    Naked Ninja (c) 2020
 * Website:    https://nakedninja.cc
 * Author(s):  Caner Erdem & Harm Verbeek
 * 
 * Pin configuration:
 *                                   +-\/-+ 
 *        PCINT5/RESET/ADC0/dW/PB5  1|    |8  Vcc 
 * PCINT3/XTAL1/CLKI/OC1B/ADC3/PB3  2|    |7  PB2/SCK/USCK/SCL/ADC1/T0/INT0/PCINT2
 * PCINT4/XTAL2/CLKO/OC1B/ADC2/PB4  3|    |6  PB1/MISO/DO/AIN1/OC0B/OC1A/PCINT1
 *                             GND  4|    |5  PB0/MOSI/DI/SDA/AIN0/OC0A/OC1A/AREF/PCINT0
 *                                   +----+
 * PB0  -(I)->  Wake up switch
 * PB1  -(Tx)-> Serial communication with ESP8266
 * PB2  -(O)->  Enable pin of voltage regulator
 * PB3  -(I)->  Wake up switch 2
 * PB4  -(RX)-> Serial communication with ESP8266
 * PB5  -(x)->  N/C
 * 
 * ATtiny85 upload settings:
 * (In order to program the ATtint85 an Arduino UNO with the ArduinoISP code is required.)
 *  Board:      Attiny25/45/85
 *  Processor:  Attiny85
 *  Clock:      Internal 1MHz
 *  Programmer: Arduino as ISP 
 *
 */

//Library's
#include <avr/sleep.h>      // Sleep Modes
#include <avr/power.h>      // Power management
#include <avr/wdt.h>        // Watchdog timer
#include <SoftwareSerial.h> // Serial communication

// Pin configuration
#define BUTTON_FRONT      0
#define BUTTON_SIDE       3
#define VREG_ENABLE       2

// Modes
#define MODE_PORTAL             0x01
#define MODE_SET_MOISTURE       0x02
#define MODE_TEST               0x03
#define WATCHDOG_TIMER          0x04
#define MODE_START_UP           0x05
#define MODE_CHARGE             0x06
#define MODE_PRESSED            0x07
#define MODE_ERROR              0x99

// Serial communication codes
#define COMMUNICATION_ID_1      0xAA
#define COMMUNICATION_ID_2      0x55
#define ESP_DONE                0xB1
#define REQUEST_MODE            0xA1
#define REQUEST_RESET           0xA2
#define RCVD_ERROR              0xAF
#define RCVD_SUCCESS            0xFE

// Delays & timers
#define CHARGE_DELAY            5000
#define SET_MOISTURE_DELAY      1000 
#define SERIAL_TIME_OUT         5000
#define ESP_BLINK_DELAY         6000

// Default value's
#define WAKE_TIME               300 // Default value (300/60 = 5min)  
#define SLEEP_MULTIPLIER        75  // Default value (75*8 = 10min)

// Object creations
SoftwareSerial mySerial(4, 1);  // (RX, TX)

// Timer variables
unsigned long timer_one;  // Timer of main program
unsigned long timer_two;  // Timer for getSettings();
unsigned long timer_three;// Timer for function button detection
unsigned long timer_four; // Timer for function button detection
int wake_time;            // Maximum time the ATtiny85 is awake
int sleep_multiplier;     // Multiplier for ATtiny85 sleep duration (8s*sleep_multiplier)
int wdt_sleep_counter;    // Watchdog sleep counter

boolean goto_sleep = true;
boolean flag_wdt = false;
boolean flag_moisture = false;
byte mode;
byte incoming_byte = 0;
byte message_status = 0;

ISR (WDT_vect) {
  //  wdt_disable();    // Disable watchdog
  flag_wdt = true; 
}

void reboot () {
  cli();
  WDTCR = 0xD8 | WDTO_60MS;
  sei(); 
  /*Options
    WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS,
    WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S
  */
  wdt_reset();
  while (true) {}
}

void resetWatchdog () {
  MCUSR = 0;                                    // Clear various "reset" flags
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);  // Allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0); // Set WDIE, and 8 seconds delay
  wdt_reset();                                  // Reset

  //Other delay settings:
  //  WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP1);             // Set WDIE, and 1 seconds delay
  //  WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP1)| bit (WDP0); // Set WDIE, and 2 seconds delay
  //  WDTCR = bit (WDIE) | bit (WDP3);                          // Set WDIE, and 4 seconds delay
  //  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0);             // Set WDIE, and 8 seconds delay 
}

void chargeSleep () {
  mySerial.end();
  delay(ESP_BLINK_DELAY);
  digitalWrite(VREG_ENABLE, LOW);   // disable voltage regulator
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;             // turn off ADC
  power_all_disable ();   // power off ADC, Timer 0 and 1, serial interface
  wdt_disable ();         // Disable watchdog incase it wasn't disabled
  noInterrupts ();        // Timed sequence coming up
  sleep_enable ();        // Ready to sleep
  interrupts ();          // Interrupts are required now
  sleep_cpu ();           // Sleep     

  /*Returns here after wakeup !!!*/
  
  sleep_disable ();                 // Precaution
  power_all_enable ();              // Power everything back on
  mode = MODE_TEST;     // Function button short press
  digitalWrite (VREG_ENABLE, HIGH); // enable voltage regulator to power the connected MCU
  
  timer_one = millis();
  goto_sleep = false;   // Disable goto_sleep
  message_status = 0;   // Reset message status
  mySerial.begin(9600);
  mySerial.flush();
}

void gotoSleep () {
  mySerial.end();
  digitalWrite(VREG_ENABLE, LOW);   // Disable voltage regulator
  
  wdt_sleep_counter = 0;  // Reset slept counter
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;             // Turn off ADC
  power_all_disable ();   // Power off ADC, Timer 0 and 1, serial interface
  
  while(wdt_sleep_counter < sleep_multiplier) {
    flag_wdt = false;
    noInterrupts ();       // Timed sequence coming up
    resetWatchdog ();      // Get watchdog ready
    sleep_enable ();       // Ready to sleep
    interrupts ();         // Interrupts are required now
    sleep_cpu ();          // Sleep     
    
    /*Returns here after wakeup !!!*/
    
    wdt_disable ();         // Disable watchdog
    sleep_disable ();       // Precaution
    if(flag_wdt == true) {
      wdt_sleep_counter++;
      mode = WATCHDOG_TIMER;// Watchdog Timer
    } 
    else {
      wdt_sleep_counter = sleep_multiplier;
      mode = MODE_START_UP;   // Reset mode if PCINT caused interrupt
    }
  }
  
  power_all_enable();        // Power everything back on
  
  if (mode == MODE_START_UP) {  // startup mode
    if (digitalRead(BUTTON_FRONT) == LOW) {
      timer_three = millis();

      while (1) {
        if (digitalRead(BUTTON_FRONT) == HIGH) {
          delay(50);
          if (digitalRead(BUTTON_FRONT) == HIGH) {
            break;
          }
        }

        if (millis() > (timer_three + CHARGE_DELAY)) { // time out
          break;
        }
        
        delay(50);
      }

      timer_four = millis();
      
      if (timer_four > timer_three + CHARGE_DELAY) { // time out
        mode = MODE_CHARGE;           // Charging mode
      }
      else if (timer_four > timer_three + SET_MOISTURE_DELAY) {
        mode = MODE_SET_MOISTURE;  // Funtion button long press
      }
      else {
         mode = MODE_TEST;   // Function button short press
      }
    }
    else if (digitalRead(BUTTON_SIDE) == LOW) {
      mode = MODE_PORTAL;  // Mini button pressed  
    }
    else {
      mode = MODE_ERROR;   // Interrupt cause unknown
    }
  }
  
  digitalWrite (VREG_ENABLE, HIGH); // Enable voltage regulator to power the connected MCU
  
  timer_one = millis();
  goto_sleep = false;   // Disable goto_sleep
  flag_moisture = false;
  message_status = 0;   // Reset message status   
  mySerial.begin(9600);
  mySerial.flush();
}

void serialSend (byte data) {
  delay(10); mySerial.write(COMMUNICATION_ID_1);
  delay(10); mySerial.write(COMMUNICATION_ID_2);
  delay(10); mySerial.write(data);
}

void getSettings () {
  //Create & reset values
  byte WT_upper = 0;
  byte SM_upper = 0;
  int temp_one = 0;
  
  incoming_byte = 0;
  message_status = 0;
  mode = MODE_START_UP; 
  
  timer_two = millis();
  while (1) {
    if (mySerial.available() > 0) {
      incoming_byte = mySerial.read();
      if (incoming_byte == COMMUNICATION_ID_1 && message_status == 0) { // Check 1
        message_status = 1;
      }
      else if (incoming_byte == COMMUNICATION_ID_2 && message_status == 1) { // Check 2
        message_status = 2;
      }
      else if (message_status == 2) { // Select correct response to incomming message
        switch (incoming_byte) {
          case REQUEST_MODE:  
            serialSend(mode);     //Send mode
            timer_two = millis(); //Reset Time out timer
            message_status = 0;
            break;
          case MODE_START_UP:
            message_status = 3;
            break;
          default:
            message_status = 0; 
            break;
        }
      }
      else if (message_status == 3) {  // Get wake time upper value
        message_status = 4; 
        WT_upper = incoming_byte;
      }
      else if (message_status == 4) {  // Get wake time lower value & calculate Wake time
        message_status = 5; 
        temp_one = (WT_upper << 8) + incoming_byte;
      }
      else if (message_status == 5) {  // Get Sleep multiplier upper value
        message_status = 6; 
        SM_upper = incoming_byte;
      }
      else if (message_status == 6) {  // Get Sleep multiplier lower value & calculate/store  
        sleep_multiplier = (SM_upper << 8) + incoming_byte;
        wake_time = temp_one;
        serialSend(RCVD_SUCCESS);
        delay(100);
        break;                // Break out while loop
      }
      else { // Default, reset message status
        message_status = 0; 
      }
    }
 
    if (millis() > (timer_two + SERIAL_TIME_OUT)) { // Time out => use default values
      wake_time = WAKE_TIME;                // Default value (300/60 = 5min)
      sleep_multiplier = SLEEP_MULTIPLIER;  // Default value (75*8 = 10min)   
      serialSend(RCVD_ERROR);               // Send ERROR message back
      delay(100);                           // Delay to ensure message is send before break
      break;                                // Break out while loop
    }
    delay(10);
  }
}

void setup () {
  // disable WDT to ensure WDT doesn't fire 
  MCUSR &= ~(1 << WDRF); 
  wdt_disable();  //Disable WDT
  
  //Inputs
  pinMode(BUTTON_FRONT, INPUT);    // 1st wake up switch
  digitalWrite(BUTTON_FRONT, HIGH);// Internal pull-up
  pinMode(BUTTON_SIDE, INPUT);        // 2nd wake up switch
  digitalWrite(BUTTON_SIDE, HIGH);    // Internal pull-up

  //Outputs
  pinMode(VREG_ENABLE, OUTPUT);        // Voltage regulator enable control
  digitalWrite(VREG_ENABLE, HIGH);     // Enable voltage regulator

  //Start serial communication
  mySerial.begin(9600);                 // Start serial communication
  mySerial.flush();                     // Flush serial buffer

  //Get Sleep & Wake time
  getSettings();

  //Pin change interrupt configuration
  PCMSK = bit (PCINT0) | bit (PCINT3); // Enable pin change interrupt on PB0(pin5) and PB3(pin2)
  GIFR  |= bit (PCIF);                 // Clear any outstanding interrupts
  GIMSK |= bit (PCIE);                 // Enable pin change interrupts 

  //Set sleep check to true
  goto_sleep = true;
}

void loop () {
  if (goto_sleep == true) { 
    gotoSleep(); // Call sleep funtion
  }

  if (flag_moisture) {
    if (digitalRead(BUTTON_FRONT) == LOW) {
      delay(50);
      if (digitalRead(BUTTON_FRONT) == LOW) {
        serialSend(MODE_PRESSED);
        flag_moisture = false;
      }
    }
  }
  
  if (millis()-timer_one >= wake_time*1000) { // if wake_time has passed
    goto_sleep = true;                       // goto sleep
  }

  if (mySerial.available() > 0) {
    byte incoming_byte = mySerial.read();
    if (incoming_byte == COMMUNICATION_ID_1 && message_status == 0) {
      message_status = 1; 
    }
    else if (incoming_byte == COMMUNICATION_ID_2 && message_status == 1) {
      message_status = 2; 
    }
    else if (message_status == 2) {
      message_status = 0; 
      switch (incoming_byte) {
        case REQUEST_MODE:
          serialSend(mode);
          if (mode == MODE_CHARGE) {
            chargeSleep();
          }
          else if (mode == MODE_SET_MOISTURE) {
            flag_moisture = true;
          }
          break;
        case ESP_DONE:
          goto_sleep = true; 
          break;
        case REQUEST_RESET:
          reboot();
          break;
        default:
          serialSend(RCVD_ERROR);
          break;
      }
    }
    else {
      message_status = 0; 
    }
  }
}  
