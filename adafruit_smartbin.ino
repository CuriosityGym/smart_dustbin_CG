/***************************************************
  Adafruit ESP8266 Sensor Module

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  Works great with Adafruit's Huzzah ESP board:
  ----> https://www.adafruit.com/product/2471
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
//#include <NewPing.h>

//#define TRIGGER_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
const int trigPin = 5;
const int echoPin = 4;
float Max_value = 28.75;//cm
long duration;
float distance;
//int doorPin = A0;
// WiFi parameters
#define WLAN_SSID       "DT_LAB"
#define WLAN_PASS       "fthu@050318"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "CG_Workshops"
#define AIO_KEY         "aio_puza175BsuJ9JJJQJdf1k92J5DjB"  // Obtained from account info on io.adafruit.com

WiFiClient client;
 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
 
/****************************** Feeds ***************************************/
 
Adafruit_MQTT_Publish DistanceSensorState = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CG_Dustbin");

unsigned long currentMillis = 0;
int interval = 5000;

int doorPinState =0;
/*************************** Sketch Code ************************************/

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
//Serial.begin(9600); // Starts the serial communication
  //pinMode(doorPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println(F("Adafruit IO Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  // connect to adafruit io
  connect();

}

void loop() {
  digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
  Serial.println(distance);
  //float distance = sonar.ping_cm();
  float value = Max_value - distance;
    //bin_empty = digitalRead(doorPin);
  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

// if(millis() - currentMillis > interval){
//  
//  if(doorPinState == 1){
//   if (! doorState.publish("CLOSED"))
//    Serial.println(F("Failed to publish SmartBin State"));
//   else
//    Serial.println(F("Door State published!"));
//  }
//  if(doorPinState == 0){
//   if (! doorState.publish("OPEN"))
//    Serial.println(F("Failed to publish SmartBin State"));
//   else
//    Serial.println(F("SmartBin State published!"));
//  } 
//   
//  currentMillis = millis();  
// }   

 if(millis() - currentMillis > interval){
  if (value>0&&value<=Max_value){
    DistanceSensorState.publish(value);
  }
  
  
  
  currentMillis = millis();
 }

}

// connect to adafruit io via MQTT
void connect() {

  Serial.print(F("Connecting to Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(5000);

  }

  Serial.println(F("Adafruit IO Connected!"));

}
