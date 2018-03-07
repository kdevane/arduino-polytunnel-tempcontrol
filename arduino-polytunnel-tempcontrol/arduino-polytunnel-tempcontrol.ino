#include <Arduino.h>
#include <U8g2lib.h>

//Temperature Sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// constants won't change :
const long tempinterval = 300000;           // interval at which to blink (milliseconds)
unsigned long previousMillis = 0;        // will store last time LED was updated
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

//push button between pin2 & gnd no resistor needed as internal used
// Relay 3 & 4 connection pins
#define DoorAOpen 4
#define DoorAClose 3
#define LED_BUILTIN 13
int stateDoorAOpen = LOW;
int stateDoorAClose = LOW;
int laststateDoorA = LOW; // Door 1 Closed when LOW
int stateDoorA = LOW;
int relayOnTime = 13000;
int relayCloseTime = 20000;
// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
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

  closedoor();
  printValuesOnly(laststateDoorA);
/*

  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
    u8g2.drawStr(0,14,"Ready ...");
  } while ( u8g2.nextPage() );
*/

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
    u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
    //u8g2.drawStr(0,14,"Stand By!");
    u8g2.drawStr(0,28,"Closing Doors....");
  } while ( u8g2.nextPage() );

  digitalWrite(DoorAClose, LOW);
  delay(relayCloseTime);
  digitalWrite(DoorAClose,HIGH);
  delay(2000);
//  readymessage( stateDoorA );
}

void opendoor() {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tf); // u8g2_font_ncenB14_tr
    //u8g2.drawStr(0,14,"Stand By!");
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
/*    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
*/
  float humidity = bme.readHumidity();
/*    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.println();
  */
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

void printValuesOnly( int dstate ) {
    float temperature = bme.readTemperature();
/*    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
*/
  float humidity = bme.readHumidity();
/*    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.println();
  */
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

  // put your main code here, to run repeatedly:
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(2);
  //Serial.println(sensorVal);
  // If the switch changed, due to noise or pressing:
  if (sensorVal != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {


  //print out the value of the pushbutton
  //Serial.println("sensorVal" + laststateDoorA );
    if (sensorVal == HIGH) {
      digitalWrite(13, LOW);
    } else {
      digitalWrite(13, HIGH);
      delay( 500 );
      int DoorAOpenVal = digitalRead( DoorAOpen );
      int DoorACloseVal = digitalRead( DoorAClose );
      //Serial.println(laststateDoorA);
      if ( laststateDoorA == LOW ) {
        //Serial.println( "Door was closed" );
        opendoor();
        laststateDoorA = HIGH;
        printValuesOnly( laststateDoorA );
      } else {
        //Serial.println( "Door was open" );
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
