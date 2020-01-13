#include <MotorDriver.h>
#include <NewPing.h>

#define TRIGGER_PIN  A3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define DELAYVAL 3000 //Play with this number to determine the idea start and sstop time for the motor.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

MotorDriver m;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
int distance = 0;
distance = sonar.ping_cm();
Serial.print("Ping: ");
Serial.print(distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
Serial.println("cm");
if (distance > 0 && distance < 5 ){
  Bin();
}
else{
  Brake();
}

}

void Bin(void){
m.motor(2,FORWARD,255);
delay(DELAYVAL);
m.motor(2,BACKWARD,255);
delay(DELAYVAL/4);
}

void Brake(void){
m.motor(2,BRAKE,0);  
}
