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
const long tempinterval = 300000;        // 30 seconds
unsigned long previousMillis = 0;        //
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
* push button between pin2 & gnd no resistor needed as internal used
* Relay is using pins 3 & 4
* connection to the linear acutator may need to be reversed if working
*  in the wrong direction
*/
#define DoorAOpen 4
#define DoorAClose 3
#define LED_BUILTIN 13
int stateDoorAOpen = LOW;
int stateDoorAClose = LOW;
int laststateDoorA = LOW; // Door 1 Closed when LOW
int stateDoorA = LOW;
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
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int lastButtonState = HIGH;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup");
  // Initialise BME280
  bme.begin();

  u8g2.begin();
//delay(2000);
  u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
      u8g2.drawStr(0,14,"Stand By!");
      u8g2.drawStr(0,28,"Setting Doors....");
    } while ( u8g2.nextPage() );

  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(DoorAClose, OUTPUT);
  pinMode(DoorAOpen, OUTPUT);

  digitalWrite( DoorAClose, HIGH );
  delay(2000);
  digitalWrite( DoorAOpen, HIGH );
  delay(2000);

  // Start off by setting the actuator in the closed state
  closedoor();
  printValuesOnly(laststateDoorA);

}

void readymessage( int dstate ){
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
    u8g2.drawStr(0,14,"Ready ...");
    if ( !dstate ) {
      u8g2.drawStr(0,28, "Doors - Open" );
    } else {
      u8g2.drawStr(0,28, "Doors - Closed" );
    }
  } while ( u8g2.nextPage() );

}
void closedoor() {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tf);
    u8g2.drawStr(0,28,"Closing Doors....");
  } while ( u8g2.nextPage() );

  digitalWrite(DoorAClose, LOW);
  delay(relayCloseTime);
  digitalWrite(DoorAClose,HIGH);
  delay(2000);
  // needed a delay here - cannot remember why ?
//  readymessage( stateDoorA );
}

void opendoor() {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
    u8g2.drawStr(0,28,"Opening Doors....");
  } while ( u8g2.nextPage() );

  digitalWrite(DoorAOpen, LOW);
  delay(relayOnTime);
  digitalWrite(DoorAOpen,HIGH);
  delay(2000);
//  readymessage( stateDoorA );
}

void printValues( int dstate ) {
    float temperature = bme.readTemperature();
    if ( mintemp > temperature ) { mintemp = temperature; }
    if ( maxtemp < temperature ) { maxtemp = temperature; }
    if ( temperature > opentemp && !dstate ) {
      opendoor();
      laststateDoorA = HIGH;
      dstate = laststateDoorA;
    }
    if ( temperature < closetemp && dstate ) {
      closedoor();
      laststateDoorA = LOW;
      dstate = laststateDoorA;
    }

  float humidity = bme.readHumidity();

  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_helvB08_tf);
      u8g2.setCursor(0, 14);
      if ( dstate ) {
        u8g2.print("Door Open ");
      } else {
        u8g2.print("Door Closed ");
      }

      u8g2.setCursor(0, 28);
      u8g2.print("Temperature ");
      u8g2.print(temperature);
      u8g2.print(" *C");

      u8g2.setCursor(0, 42);
      u8g2.print("min ");
      u8g2.print(mintemp);
      u8g2.print(" *C max ");
      u8g2.print(maxtemp);
      u8g2.print(" *C");

      u8g2.setCursor(0, 56);
      u8g2.print("Humidity ");
      //u8g2.setCursor(40, 56);
      u8g2.print(humidity);
      u8g2.print(" %");
    } while ( u8g2.nextPage() );

}

void printValuesOnly( int dstate ) {
  float temperature = bme.readTemperature();

  float humidity = bme.readHumidity();
  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_helvB08_tf); // u8g2_font_ncenB14_tr u8g2_font_ncenB08_tf
      u8g2.setCursor(0, 14);
      if ( dstate ) {
        u8g2.print("Door Open ");
      } else {
        u8g2.print("Door Closed ");
      }

      u8g2.setCursor(0, 28);
      u8g2.print("Temperature ");
      u8g2.print(temperature);
      u8g2.print(" *C");

      u8g2.setCursor(0, 42);
      u8g2.print("min ");
      u8g2.print(mintemp);
      u8g2.print(" *C max ");
      u8g2.print(maxtemp);
      u8g2.print(" *C");

      u8g2.setCursor(0, 56);
      u8g2.print("Humidity ");
      //u8g2.setCursor(40, 56);
      u8g2.print(humidity);
      u8g2.print(" %");
    } while ( u8g2.nextPage() );
}

void loop() {
  unsigned long currentMillis = millis();

  //read the pushbutton value into a variable
  int sensorVal = digitalRead(2);

  // If the switch changed, due to noise or pressing:
  if (sensorVal != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (sensorVal == HIGH) {
      digitalWrite(13, LOW);
    } else {
      digitalWrite(13, HIGH);
      delay( 500 );
      int DoorAOpenVal = digitalRead( DoorAOpen );
      int DoorACloseVal = digitalRead( DoorAClose );

      if ( laststateDoorA == LOW ) {
        opendoor();
        laststateDoorA = HIGH;
        printValuesOnly( laststateDoorA );
      } else {
        closedoor();
        laststateDoorA = LOW;
        printValuesOnly( laststateDoorA );
      }
    }

  }

  if (currentMillis - previousMillis >= tempinterval) {
    previousMillis = currentMillis;
    printValues( laststateDoorA );
  }

lastButtonState = sensorVal;
}
