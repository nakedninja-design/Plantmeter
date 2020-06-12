/*
 * Web Interfacing Program for Plantmeter ESP
 * 
 * Company:    Naked Ninja (c) 2020
 * Website:    https://nakedninja.cc
 * Author(s):  Caner Erdem & Harm Verbeek
 * 
 * Pins that are connected on the PCB are:
 * GPIO4    -(I/O)->  SDA for I2C connection to the HTU21 & gy-49.
 * GPIO5    -(I/O)->  SCL for I2C connection to the HTU21 & gy-49.
 * GPIO12   -(TX)->   Serial communication with the ATtiny85
 * GPIO13   -(RX)->   Serial communication with the ATtiny85
 * GPIO14   -(O)->    RGB LED
 * ADC      -(I)->    Moisture sensor data input
 * 
 * Arduino IDE version 1.8.9
 * 
 * ESP8266 upload settings:
 *  Board:              Generic ESP8266 module
 *  Upload speed:       921600
 *  CPU freqyency:      80 MHz
 *  Flash frequency:    40 MHz
 *  Flash mode:         DOUT (compatible)
 *  Flash size:         4M (FS:1M)
 *  Crystal frequency:  26MHz
 *  Reset method:       nodemcu
 *  Debug port:         Disabled
 *  Debug level:        None
 *  IwIP variant:       v2 Lower Memory
 *  VTables:            Flash
 *  Exceptions:         Disabled
 *  Builtin LED:        2
 *  Erase flash:        All flash contents
 *  Espressif FW:       nonos-sdk 2.2.1 (legacy)
 *  SSL support:        All SSL ciphers (most compatible)
 * 
 * Programmer: AVRISP mkII
 * 
 * Component datasheets & usefull links:
 *  - GY-49: https://www.digchip.com/datasheets/parts/datasheet/280/MAX44009-pdf.php
 *  - HTU21: http://www.farnell.com/datasheets/1780639.pdf
 *  - Cap. Moisture sensor: https://wiki.dfrobot.com/Capacitive_Soil_Moisture_Sensor_SKU_SEN0193
 * 
 */

// Libraries
#include <ArduinoJson.h>
#include <FS.h>
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <Ticker.h>

// Create timer
Ticker ticker;

// Code version
#define VERSION     "v1.27"

// Enable / Disable serial debug output
#define DEBUG true // flag to turn on/off debugging
#define USE_SERIAL if(DEBUG)Serial 

// Pin configuration
#define RGB_LED_PIN     14  // Connected RGB LED

// RGB settings & control modes
#define RGB_BRIGHTNESS    64          // Between 0 and 255
#define BLINK_DELAY       250         // Time in ms
#define LED_OFF           0,0,0       // OFF
#define LED_RED           255,0,0     // Red
#define LED_GREEN         0,255,0     // Green
#define LED_BLUE          0,0,255     // Blue
#define LED_YELLOW        255,255,0   // Yellow
#define LED_PURPLE        128,0,255   // Purple
#define LED_ORANGE        255,128,0   // Orange
#define LED_PINK          255,0,255   // Pink
#define LED_WHITE         255,255,255 // White

/* RGB LED status messages
 *    <COLOR>, <NUMBER>
 * <COLOR> is the color that will be visualized on the RGB LED
 * <NUMBER> is the amount of times the RGB will blink the given color
 * If <NUMBER> is set to -1, it will toggle the RGB LED to the given color
*/
#define BLINK_SAVE_CONFIG_SUCCESS       LED_GREEN, 2
#define BLINK_SAVE_CONFIG_ERROR         LED_RED, 2
#define BLINK_HTTP_RCV_OK               LED_GREEN, 2
#define BLINK_HTTP_RCV_ERROR            LED_RED, 2
#define BLINK_SSID_MISSING              LED_RED, 2
#define BLINK_WIFI_MAX_RETRIES          LED_RED, 2
#define TOGGLE_PORTAL                   LED_YELLOW, -1
#define TOGGLE_MOISTURE_MEASURE         LED_WHITE, -1
#define BLINK_MOISTURE_MEASURE_TIMEOUT  LED_RED, 2
#define BLINK_MOISTURE_MEASURE_FAIL     LED_RED, 2
#define BLINK_MODE_ERROR                LED_RED, 2
#define BLINK_SERIAL_NO_RESPONSE        LED_RED, 2
#define BLINK_MODE_CHARGE               LED_PURPLE, 5

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
#define COMMUNICATION_ID_1      0xAA  // Header code 1 
#define COMMUNICATION_ID_2      0x55  // Header code 2
#define ESP_DONE                0xB1  // HEX code for shutdown command
#define REQUEST_MODE            0xA1  // HEX code for start up mode request
#define REQUEST_RESET           0xA2  // HEX code for ATtiny85 reset request
#define RCVD_ERROR              0xAF  // HEX code for Error message
#define RCVD_SUCCESS            0xFE  // HEX code for success message

// Delays, timers & retries
#define SERIAL_TIMEOUT          3000  // Max serial communication time per attempt
#define MOISTURE_TIMEOUT        20000 // Max time to put device in soil and press the function button
#define SERIAL_ATTEMPTS         5     // Amount of serial communication attempts before timeout
#define MAX_WIFI_RETRIES        12    // Amount of Wifi communication attempts before timeout
#define CMP_OFFSET              50    // Value for dry and wet offset check

// Default values
#define DEF_URL                 "http://%s.local/plantmeter"  // $s will be replaced with ssid
#define DEF_ID                  "Plantmeter"                  // Device ID
#define DEF_AUTH_USER           "user"                        // Basic Authentication User
#define DEF_AUTH_PASSWORD       "user"                        // Basic Authentication Password
#define DEF_SOIL_MOISTURE_DRY   600                           // Default dry level
#define DEF_SOIL_MOISTURE_WET   "250"                         // Default wet level
#define DEF_WAKE_TIME           300                           // Seconds the ATtiny85 will be awake if no ESP_DONE is received
#define DEF_SLEEP_TIME          "600"                         // Seconds the ATtiny85 will sleep

// Sensor settings
#define GY49_ADDR 0x4A  // Light sensor addres
HTU21D HTU21;           // HTU21 object creation

// Create softwareserial connection
SoftwareSerial mySerial(13, 12);  // RX, TX

// Create RGB LED class
Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Allocate a JsonDocument
StaticJsonDocument<200> json;

// Global value's
char my_id[50]                = "";
char my_auth_password[10]     = "";
char my_soil_moisture_wet[10] = "";
char my_sleep_time[10]        = "";

// flags 
bool shouldSaveConfig = false;
bool flag_color_toggle = false;

// Mode
byte mode_state = 0;

// Call for timer interrupt
void tick() {
  controlRGB(TOGGLE_PORTAL);
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  USE_SERIAL.println("Should save config");
  shouldSaveConfig = true;
}

void loadConfig () { 
    if (SPIFFS.begin()) {
      USE_SERIAL.println("Mounted file system");    
      File configFile = SPIFFS.open("/config.json", "r");
      
      if (configFile) {
        USE_SERIAL.println("Reading config file");
        // Deserialize the JSON document
        DeserializationError error = deserializeJson(json, configFile);
        if (error) {
          USE_SERIAL.println("Failed to load json config");
        } 
        else {    
          USE_SERIAL.println("\nparsed json");

          strcpy(my_id, json["id"]);
          strcpy(my_auth_password, json["auth_password"]);
          strcpy(my_soil_moisture_wet, json["soil_moisture_wet"]);
          strcpy(my_sleep_time, json["sleep_time"]);

          USE_SERIAL.println(my_id);
          USE_SERIAL.println(my_auth_password);
          USE_SERIAL.println(my_soil_moisture_wet);
          USE_SERIAL.println(my_sleep_time);
        }
        
        configFile.close();
      }
      else { // No config file found
        // create config file with default values
        USE_SERIAL.println("Creating config file with default values");
    
        strcpy(my_id, DEF_ID);
        strcpy(my_auth_password, DEF_AUTH_PASSWORD);
        strcpy(my_soil_moisture_wet, DEF_SOIL_MOISTURE_WET);
        strcpy(my_sleep_time, DEF_SLEEP_TIME);

        USE_SERIAL.println("Saving config");

        json["id"] = my_id;
        json["auth_password"] = my_auth_password;
        json["soil_moisture_wet"] = my_soil_moisture_wet;
        json["sleep_time"] = my_sleep_time;
    
        File configFile = SPIFFS.open("/config.json", "w+");
        if (!configFile) {
          USE_SERIAL.println("Failed to open config file for writing");
        } 
        else {
          // Serialize JSON to file
          if (serializeJson(json, configFile) == 0) {
            USE_SERIAL.println(F("Failed to write to file"));
           } else {
            USE_SERIAL.println(F("Succeeded to write to file"));
          }
        
          // Close the file
          configFile.close();
        }
   }
   
  } 
  else {
    USE_SERIAL.println("Failed to mount FS");
  } 
} 

void saveConfig () { 
   if (SPIFFS.begin()) {
      USE_SERIAL.println("Mounted file system");    
      // If you get here you have connected to the WiFi
      USE_SERIAL.println("Connected and saving configuration ...");

      json["id"] = my_id;
      json["auth_password"] = my_auth_password;
      json["soil_moisture_wet"] = my_soil_moisture_wet;
      json["sleep_time"] = my_sleep_time;

      File configFile = SPIFFS.open("/config.json", "w+");
      if (!configFile) {
        USE_SERIAL.println("Failed to open config file for writing");
      } 
      else {
        // Serialize JSON to file
        if (serializeJson(json, configFile) == 0) {
          USE_SERIAL.println(F("Failed to write to file"));
          controlRGB(BLINK_SAVE_CONFIG_ERROR);
        } 
        else {
          USE_SERIAL.println(F("Succeeded to write to file"));
          controlRGB(BLINK_SAVE_CONFIG_SUCCESS);
        }
      
        // Close the file
        configFile.close();
      }
    } 
    else {
      USE_SERIAL.println("Failed to mount FS");
    } 
} 

float readLuminance () {
  unsigned int data[2];
  Wire.beginTransmission(GY49_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.requestFrom(GY49_ADDR, 2);  // Request 2 bytes of data
  if (Wire.available() == 2) {     // Read 2 bytes of data luminance msb, luminance lsb
    data[0] = Wire.read();         // Lux High-Byte Register 0x03
    data[1] = Wire.read();         // Lux Low-Byte Register 0x04
  }
  
  // Convert the data to lux
  int exponent = (data[0] & 0xF0) >> 4;                       // Exponent = 8xE3 + 4xE2 + 2xE1 + E0
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);  // Mantissa = Mantissa = 128xM7 + 64xM6 + 32xM5 + 16xM4 + 8xM3 + 4xM2 + 2xM1 + M0
  float luminance = pow(2, exponent) * mantissa * 0.045;      // Lux = 2(exponent) x mantissa x 0.045

  return luminance;
}

float readSoilMoisture () {
  float total = 0;
  float temp_one = 0;
  
  for (int y = 0; y < 2; y++) {
    temp_one = system_adc_read(); 
    delay(10);
  }
  
  for (int x = 0; x < 10; x++) {
    temp_one = system_adc_read();
//    USE_SERIAL.println(tempp);
    total += temp_one;
    delay(10);
  }
  
  float average = total / 10;
  return average;
}

void controlRGB (int r, int g, int b, int blink) {
  if (blink == -1) {
      if (flag_color_toggle == false) {
        pixel.setPixelColor(0, pixel.Color(LED_OFF)); 
        pixel.show(); 
        flag_color_toggle = true;
      } else {
        pixel.setPixelColor(0, pixel.Color(r,g,b)); 
        pixel.show();
        flag_color_toggle = false;
      }        
  }
  else {
    for (int i = 0; i < blink; i++) {
      pixel.setPixelColor(0, pixel.Color(r,g,b)); 
      pixel.show(); // This sends the updated color to the hardware.
      delay(BLINK_DELAY);
      pixel.setPixelColor(0, pixel.Color(LED_OFF)); // switch led off.
      pixel.show();
      delay(BLINK_DELAY);
    }  
  }
}

void serialSend (byte mode, byte dbyte1,byte dbyte2=0,byte dbyte3=0,byte dbyte4=0) {
  delay(10); mySerial.write(COMMUNICATION_ID_1);
  delay(10); mySerial.write(COMMUNICATION_ID_2);
  
  if (mode == MODE_START_UP) {
   delay(10); mySerial.write(mode);
   delay(10); mySerial.write(dbyte1);
   delay(10); mySerial.write(dbyte2);
   delay(10); mySerial.write(dbyte3);
   delay(10); mySerial.write(dbyte4);      
  }
  else {
    delay(10); mySerial.write(dbyte1);
  }
}

bool serialCheck(byte mode){
  byte incoming_byte = 0;
  byte message_status = 0;
  
  if (mySerial.available() > 0) {
    long int timer_one = millis();
    while (millis() < (timer_one + SERIAL_TIMEOUT)) {
      incoming_byte = mySerial.read();
      //USE_SERIAL.print("  Received: ");
      //USE_SERIAL.println(incoming_byte, HEX);
      if (incoming_byte == COMMUNICATION_ID_1 && message_status == 0) {
        message_status = 1; 
      }
      else if (incoming_byte == COMMUNICATION_ID_2 && message_status == 1)  {
        message_status = 2; 
      } 
      else if (incoming_byte == mode && message_status == 2)  {
        USE_SERIAL.println("serialCheck ok for mode");
        return true;         
      }      
      else {
        return false;
      }
    }
  }
  
  return false;
}

byte serialMessage (byte mode, byte dbyte1,byte dbyte2=0,byte dbyte3=0,byte dbyte4=0) {
  USE_SERIAL.println("Requesting data");
  byte incoming_byte = 0;
  byte message_status = 0;
  long int timer_one = millis();
  bool check_one = true;
  int count_one = 0;
  byte res;
  
  while (check_one) {
    serialSend(mode, dbyte1,dbyte2,dbyte3,dbyte4);
    count_one++;
   
    while (millis() < (timer_one + SERIAL_TIMEOUT)) {
      if (mySerial.available() > 0) {
        incoming_byte = mySerial.read();
        USE_SERIAL.print("  Received: ");
        USE_SERIAL.println(incoming_byte, HEX);
        if (incoming_byte == COMMUNICATION_ID_1 && message_status == 0) {
          message_status = 1; 
        } 
        else if (incoming_byte == COMMUNICATION_ID_2 && message_status == 1)  {
          message_status = 2; 
          break;
        } 
        else if (message_status == 2) {
          USE_SERIAL.println("  MSG status 2, reading data");

          switch (incoming_byte) {
            case MODE_START_UP: 
              USE_SERIAL.println("  MODE_START_UP");
              return MODE_START_UP;
            case MODE_PORTAL:       
              USE_SERIAL.println("  MODE_PORTAL");
              return MODE_PORTAL;
            case MODE_TEST: 
              USE_SERIAL.println("  MODE_TEST");
              return MODE_TEST;
            case MODE_SET_MOISTURE:
              USE_SERIAL.println("  MODE_SET_MOISTURE");
              return MODE_SET_MOISTURE;
            case WATCHDOG_TIMER: 
              USE_SERIAL.println("  WATCHDOG_TIMER ");
              return WATCHDOG_TIMER;
            case RCVD_SUCCESS:
              USE_SERIAL.println("Received OK"); 
              return RCVD_SUCCESS;
            case RCVD_ERROR:
              USE_SERIAL.println("Received ERROR"); 
              return RCVD_ERROR;
            case MODE_CHARGE:
              USE_SERIAL.println("  MODE_CHARGE");
              return MODE_CHARGE;
            case MODE_ERROR:
              USE_SERIAL.println("  MODE_ERROR");
              return MODE_ERROR;
            default:
              USE_SERIAL.println("  No match in serial communication"); 
              message_status = 0; 
              break;          
          } 
        }
        else {
          message_status = 0; 
        }
      }
    }
      

    if (count_one >= SERIAL_ATTEMPTS) {
      USE_SERIAL.println("  Failed to get response");
      check_one = false;
    }
  }
  
  // Only comes here if no serial response was received
  controlRGB(BLINK_SERIAL_NO_RESPONSE);
  shutdownESP();
}

void shutdownESP () {
  USE_SERIAL.println("\nSignal the ATtiny to turn off the voltage regulator...");

  serialSend(0, ESP_DONE); // we are done, shutdown
  serialSend(0, ESP_DONE); // we are done, shutdown
  serialSend(0, ESP_DONE); // we are done, shutdown
  delay(200);  
  controlRGB(LED_OFF, 0);  // turn off the activity led
  ESP.deepSleep(0);
  yield();
}

void setup() {
  WiFi.forceSleepBegin();
  delay(100);
  
  mySerial.begin(9600);
  
  USE_SERIAL.begin(115200);
  USE_SERIAL.println("\nStarting");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off built in LED

  // initialize NeoPixel library
  pixel.begin(); 
  delay(10);
  pixel.setBrightness(RGB_BRIGHTNESS);
  controlRGB(LED_OFF, 0);  // turn off the activity led
   
  loadConfig();

  //Get mode settings from ATtiny85
  mode_state = serialMessage(0, REQUEST_MODE);  
 
  if (mode_state == MODE_CHARGE) {              // Blink RGB and go to sleep
    USE_SERIAL.println("Charging mode enabled");
    USE_SERIAL.println("To disable, the main funtion button must be pressed once.");
    controlRGB(BLINK_MODE_CHARGE);
    ESP.deepSleep(0);
    yield();
  }
  else if (mode_state == MODE_START_UP) {       // Send wake & sleep time settings to ATtiny85
    USE_SERIAL.println();
    USE_SERIAL.println("Start up mode");
    uint16_t mask = B11111111;

    int sleep_multiplier = (int)(atoi(my_sleep_time)*60/8);
      
    byte WT_Upper = DEF_WAKE_TIME >> 8;
    byte WT_Lower = DEF_WAKE_TIME & mask;
    byte SM_Upper = sleep_multiplier >> 8;
    byte SM_Lower = sleep_multiplier & mask;
    
    // Send wake_time and sleep_multiplier and wait for confirmation
    serialMessage(MODE_START_UP, WT_Upper, WT_Lower, SM_Upper, SM_Lower);
     
    USE_SERIAL.println("Sent wake time and sleep multiplier");
    
    ESP.deepSleep(0);
    yield();
  }
  else if (mode_state == MODE_PORTAL) { // Configuration portal requested
    // WiFiManager
    // Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //reset settings - for testing
    //wifiManager.resetSettings();
    
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    wifiManager.setTimeout(180); // 3 minutes

    //it starts an access point with the specified name
    //and goes into a blocking loop awaiting configuration

    //WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.1
    //WiFi.mode(WIFI_STA);

    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    
    // Exit after saving configuration
    wifiManager.setBreakAfterConfig(true);

    // (id/name, placeholder/prompt, default, length)
    WiFiManagerParameter custom_id("id", "id", my_id, 50);
    wifiManager.addParameter(&custom_id);

    WiFiManagerParameter custom_auth_password("Auth password", "password", my_auth_password, 10);
    wifiManager.addParameter(&custom_auth_password);
    
    WiFiManagerParameter custom_sleep_time("Sleep time", "Sleep time (minutes)", my_sleep_time, 10);
    wifiManager.addParameter(&custom_sleep_time);

    ticker.attach(0.25, tick);
    
    //start an access point with the specified name
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal(my_id)) {
      USE_SERIAL.println("Failed to connect and hit timeout");
    }
    
    ticker.detach();
    controlRGB(LED_OFF, 0);
    delay(1000);
    
    if (shouldSaveConfig) {
       // Save configuration changes
      strcpy(my_id, custom_id.getValue());
      strcpy(my_auth_password, custom_auth_password.getValue());
      strcpy(my_sleep_time, custom_sleep_time.getValue());
      saveConfig();
        
      //controlRGB(LED_GREEN,1);
      serialMessage(0, REQUEST_RESET);
    }
    
    // shut down
    shutdownESP();        
  }
  else if (mode_state == MODE_SET_MOISTURE) {// Configure mode
    long int timer_one = millis();
    bool btn_pressed;
    
    while (millis() < (timer_one + MOISTURE_TIMEOUT)) {
      controlRGB(TOGGLE_MOISTURE_MEASURE);
      btn_pressed = serialCheck(MODE_PRESSED);
      if (btn_pressed) {
        break;
      }
      delay(250);
    }
    
    controlRGB(LED_OFF, 0);
    
    if (!btn_pressed) {
      controlRGB(BLINK_MOISTURE_MEASURE_TIMEOUT);
      shutdownESP();
    }

    USE_SERIAL.println("Configuring new wet moisture level");
    
    //Run moisture sensor once in order to filter out random value's
    readSoilMoisture();
    
    float wet_moisture = 0;
    for (int x = 0; x < 10; x++) {
      wet_moisture += readSoilMoisture(); // average of 10 measurements
    }
    wet_moisture = wet_moisture / 10;
    USE_SERIAL.print("wet_moisture: ");
    USE_SERIAL.println(wet_moisture);

    if ((wet_moisture + CMP_OFFSET) < DEF_SOIL_MOISTURE_DRY) {
      USE_SERIAL.println("Measurement succeeded");
      sprintf(my_soil_moisture_wet, "%.2f", wet_moisture);
      delay(50);
      saveConfig();
      //controlRGB(LED_GREEN, 3);
    } 
    else {
      USE_SERIAL.println("Measurement failed");
      controlRGB(BLINK_MOISTURE_MEASURE_FAIL);
    }

    shutdownESP(); 
  } 
  else if (mode_state == MODE_ERROR) {          // MODE_ERROR never happens!
    controlRGB(BLINK_MODE_ERROR);

    // shut down
    shutdownESP();        
  }

  // Start I2c communication & initialise connected sensors
  Wire.begin();
  delay(10);
  HTU21.begin();   // Initialize HTU21 sensor
  delay(10);

  Wire.beginTransmission(GY49_ADDR); // Initialize GY-49 sensor
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.endTransmission();
  delay(100);
  
  USE_SERIAL.println();
  USE_SERIAL.printf("SSID: %s\n", WiFi.SSID().c_str());
   
  if (WiFi.SSID() == "" || WiFi.psk() == "") {
    USE_SERIAL.printf("SSID or PSK was empty");
    controlRGB(BLINK_SSID_MISSING);
    shutdownESP();
  }

  WiFi.begin();
  
  USE_SERIAL.print("[SETUP] WAIT ");
  int wait_counter = 0;

  while ((WiFi.status() != WL_CONNECTED) && (wait_counter++ < MAX_WIFI_RETRIES)) {
    USE_SERIAL.print(".");
    delay(1000);
  }
  
  USE_SERIAL.flush();
  USE_SERIAL.println();
  
  // After trying to connect MAX_WIFI_RETRIES times ...
  if (wait_counter == MAX_WIFI_RETRIES) {
    // blink Error LED ...
    controlRGB(BLINK_WIFI_MAX_RETRIES);
    
    // shut down
    shutdownESP();        
  }

  WiFi.mode(WIFI_STA);
  HTTPClient http;

  // to be replaced by auth_user and auth_password
  if (strlen(DEF_AUTH_USER) > 0 && strlen(my_auth_password) > 0) {
     http.setAuthorization(DEF_AUTH_USER, my_auth_password);
  }

  char my_url[80];
  sprintf(my_url, DEF_URL , WiFi.SSID().c_str());
  USE_SERIAL.print("[HTTP] begin...\n");
  USE_SERIAL.println(my_url);
  http.begin(my_url);
  int http_code = 0;

  // POST DATA
  USE_SERIAL.print("[HTTP] POST...\n");
  float humd = HTU21.readHumidity();
  float temp = HTU21.readTemperature();
  float luminance = readLuminance();
  float soil_moisture = readSoilMoisture();
  String postData =  + "ID=" + String(my_id) + "&Version=" + VERSION +"&Temperature=" + String(temp) + "&Humidity=" + String(humd) + "&Luminance=" + String(luminance) + "&Soil=" + String(soil_moisture)+ "&WetSoil=" + String(my_soil_moisture_wet)+ "&DrySoil=" + String(DEF_SOIL_MOISTURE_DRY); 
  http_code = http.POST(postData);     
  
  // http_code will be negative on error
  if (http_code > 0) {
    // HTTP header has been send and Server response header has been handled
    USE_SERIAL.printf("[HTTP] Result code: %d\n", http_code);

    // file found at server
    if (http_code == HTTP_CODE_OK) {
      String payload = http.getString();
      USE_SERIAL.println(payload);
      
      controlRGB(BLINK_HTTP_RCV_OK);
    } 
    else {
      controlRGB(BLINK_HTTP_RCV_ERROR);
    }
  } 
  else {
    USE_SERIAL.printf("[HTTP] POST failed, error: %s\n", http.errorToString(http_code).c_str());
    
    // blink Error LED ...
    controlRGB(BLINK_HTTP_RCV_ERROR);
  }

  http.end();

  // shut down
  shutdownESP(); 
}

// the loop function runs over and over again forever
void loop () {
   yield();
}
