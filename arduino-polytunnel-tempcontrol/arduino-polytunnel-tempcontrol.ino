/*
 * arduino-polytunnel-tempcontrol
 * 
 * monitor temperature and try to automatically control it using a linear acutator to
 * open and close a door based on a minimum and maximum temperature
 * allow access by using a momentary switch
 * keep the door open for a minimum amount of time when the button is pressed
 * 
 */

#include <Arduino.h>
#include <U8g2lib.h>

//Temperature Sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// constants won't change :
// how often to log the temperature in miliseconds
unsigned long tempinterval = 30;        // 30 seconds
unsigned long previousMillis = 0;        // time of last temperature check
// variables to store the max and min temperatures records
float maxtemp = 0.00;
float mintemp = 100.00;
// Set temperature at which to open or close the door
const float opentemp = 28.00;
const float closetemp = 24.00;


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

/*
  push button between pin2 & gnd no resistor needed as internal used
  Relay is using pins 3 & 4
  connection to the linear acutator may need to be reversed if working
   in the wrong direction
*/
#define DoorAOpen 4
#define DoorAClose 3

volatile byte stateDoorAOpen = LOW;
volatile byte stateDoorAClose = LOW;
volatile byte laststateDoorA = LOW; // Door 1 Closed when LOW
volatile byte stateDoorA = LOW;
// relayOnTime is the amount of time it takes to open the door fully
// timed how long it took the actuator to extent far enough
// on a full battery charge that the door was held firmly open.
int relayOnTime = 13000;
// relayCloseTime needs to be long enough to make sure the door closes
// fully even at a lower battery charge i've allowed +7 seconds
int relayCloseTime = 20000;
// the following variables are unsigned long's because the time,
// measured in miliseconds, will quickly become a bigger number than
// can be stored in an int.

// the last time the output pin was toggled
volatile unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// minimum time to keep the door open set in millis 
// otherwise if it's below them min temp it closes almost right away
unsigned long mindooropentime = 180000;
// counter to keep track how long the door was open
volatile unsigned long lastdooropentime = 0;

int lastButtonState = HIGH;
int sensorVal;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup");
  // Initialise BME280
  bool status;
  // had to edit Adafruit_BME280.h and change address to 0x76 to get sensor recognised
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  u8g2.begin();
  int displayWidth = u8g2.getDisplayWidth();
  int displayHeight = u8g2.getDisplayHeight();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_timB12_tr); // u8g2_font_ncenB14_tr
    u8g2.drawStr(3, 14, "Stand By!");
    u8g2.drawStr(3, 32, "Setting Doors....");
    u8g2.drawRFrame(0, 0, displayWidth, displayHeight, 3);
  } while ( u8g2.nextPage() );

  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(2), buttonstate, CHANGE);
  //setup an interup on the button pin
  pinMode(DoorAClose, OUTPUT);
  pinMode(DoorAOpen, OUTPUT);

  digitalWrite( DoorAClose, HIGH );
  delay(2000);
  digitalWrite( DoorAOpen, HIGH );
  delay(2000);

  // Start off by setting the actuator in the closed state
  Serial.println("Setup close door");
  closedoor(displayWidth, displayHeight);
  printValuesOnly(laststateDoorA, displayWidth, displayHeight);
}

void loop() {
  // get the display width & height
  static int displayWidth = u8g2.getDisplayWidth();
  static int displayHeight = u8g2.getDisplayHeight();

  //read the pushbutton value into a variable
  sensorVal = digitalRead(2);

  // If the switch changed, due to noise or pressing:
  if (sensorVal == LOW ) {
    if ((unsigned long)(millis() - lastDebounceTime) > debounceDelay) {
      if ( !laststateDoorA ) {
        opendoor(displayWidth, displayHeight);
        laststateDoorA = HIGH;
        printValuesOnly( laststateDoorA, displayWidth, displayHeight );
        lastdooropentime = millis();
      } else {
        closedoor(displayWidth, displayHeight);
        laststateDoorA = LOW;
        printValuesOnly( laststateDoorA, displayWidth, displayHeight );
      }
      //reset timer for debounce
      lastDebounceTime = millis();
      delay( 500 );
    }
  }

  if ( (unsigned long)(millis() - previousMillis ) >= ( tempinterval * 1000 ) ) {
    if ( (unsigned long)(millis() - lastdooropentime ) >= mindooropentime ) {
      printValues( laststateDoorA, displayWidth, displayHeight );
      previousMillis = millis();
    } else {
      printValuesOnly( laststateDoorA, displayWidth, displayHeight );
      previousMillis = millis();
    }
  }
}

void closedoor( int displayWidth, int displayHeight) {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor( 3, 28 );
    u8g2.print("Closing");
    u8g2.setCursor( 3, 50 );
    u8g2.print("Door.....");
    u8g2.drawRFrame( 0, 0, displayWidth, displayHeight, 3 );
  } while ( u8g2.nextPage() );

  digitalWrite(DoorAClose, LOW);
  delay(relayCloseTime);
  digitalWrite(DoorAClose, HIGH);
  delay(2000);
  // needed a delay here - cannot remember why ?
}

void opendoor( int displayWidth, int displayHeight) {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor( 3, 28 );
    u8g2.print("Opening");
    u8g2.setCursor( 3, 50 );
    u8g2.print("Door.....");
    u8g2.drawRFrame( 0, 0, displayWidth, displayHeight, 3 );
  } while ( u8g2.nextPage() );

  digitalWrite(DoorAOpen, LOW);
  delay(relayOnTime);
  digitalWrite(DoorAOpen, HIGH);
  delay(2000);
  lastdooropentime = millis();
}

void printValues( byte dstate, int displayWidth, int displayHeight ) {
  float temperature = bme.readTemperature();
  if ( mintemp > temperature ) {
    mintemp = temperature;
  }
  if ( maxtemp < temperature ) {
    maxtemp = temperature;
  }
  if ( temperature > opentemp && !dstate ) {
    opendoor( displayWidth, displayHeight);
    laststateDoorA = HIGH;
    dstate = laststateDoorA;
    lastdooropentime = millis();
  }
  if ( temperature < closetemp && dstate && ( (unsigned long)( millis() - lastdooropentime ) >= mindooropentime ) ) {
    closedoor(displayWidth, displayHeight);
    laststateDoorA = LOW;
    lastdooropentime = 0;
    Serial.println("close the door again");
    dstate = laststateDoorA;
  }

  float humidity = bme.readHumidity();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB08_tf);
    if ( dstate ) {
      u8g2.setCursor( displayWidth / 2 - u8g2.getStrWidth("Door Open") / 2 , 13 );
      u8g2.print("Door Open");
    } else {
      u8g2.setCursor( displayWidth / 2 - u8g2.getStrWidth("Door Closed") / 2 , 13 );
      u8g2.print("Door Closed");
    }
    u8g2.drawHLine( 0, 14, displayWidth );

    u8g2.setCursor(3, 27);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print("Temperature ");
    u8g2.setCursor(3, 43);
    u8g2.setFont(u8g2_font_helvB14_tn);
    u8g2.print(temperature);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print(" *C");

    u8g2.setCursor( displayWidth / 2 + 3, 27);
    u8g2.print("Humidity ");
    u8g2.setCursor(displayWidth / 2 + 3, 43);
    u8g2.setFont(u8g2_font_helvB14_tn);
    u8g2.print(humidity);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print(" %");

    u8g2.setCursor(3, 61);
    u8g2.print("min ");
    u8g2.print(mintemp);
    u8g2.print(" *c max ");
    u8g2.print(maxtemp);
    u8g2.print(" *c");

    u8g2.drawRFrame( 0, 0, displayWidth, displayHeight, 3 );
    u8g2.drawVLine( displayWidth / 2, 14, 32 );
    u8g2.drawHLine( 0, 47, displayWidth );
  } while ( u8g2.nextPage() );

}

void printValuesOnly( byte dstate, int displayWidth, int displayHeight ) {
  float temperature = bme.readTemperature();

  float humidity = bme.readHumidity();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB08_tf);
    if ( dstate ) {
      u8g2.setCursor( displayWidth / 2 - u8g2.getStrWidth("Door Open") / 2 , 13);
      u8g2.print("Door Open");
    } else {
      u8g2.setCursor( displayWidth / 2 - u8g2.getStrWidth("Door Closed") / 2 , 13);
      u8g2.print("Door Closed");
    }
    u8g2.drawHLine( 0, 14, displayWidth );

    u8g2.setCursor(3, 27);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print("Temperature ");
    u8g2.setCursor(3, 43);
    u8g2.setFont(u8g2_font_helvB14_tn);
    u8g2.print(temperature);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print(" *C");

    u8g2.setCursor( displayWidth / 2 + 3, 27);
    u8g2.print("Humidity ");
    u8g2.setCursor(displayWidth / 2 + 3, 43);
    u8g2.setFont(u8g2_font_helvB14_tn);
    u8g2.print(humidity);
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.print(" %");

    u8g2.setCursor(3, 61);
    u8g2.print("min ");
    u8g2.print(mintemp);
    u8g2.print(" *c max ");
    u8g2.print(maxtemp);
    u8g2.print(" *c");

    u8g2.drawRFrame( 0, 0, displayWidth, displayHeight, 3 );
    u8g2.drawVLine( displayWidth / 2, 14, 32 );
    u8g2.drawHLine( 0, 47, displayWidth );
  } while ( u8g2.nextPage() );
}

