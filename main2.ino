#include "Enes100.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
Servo a;
Servo b;
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define m1p1 22 //motor 1 pin 1 
#define m1p2 23 //motor 1 pin 2
#define m2p1 24 //motor 2 pin 1
#define m2p2 25 //motor 2 pin 2
#define m3p1 26 //motor 3 pin 1
#define m3p2 27 //motor 3 pin 2
#define m4p1 28 //motor 4 pin 1
#define m4p2 29 //motor 4 pin 2

#define echoPinL 4
#define trigPinL 5
#define echoPinR 2
#define trigPinR 3
long durationL;
long durationR;
int distanceL;
int distanceR;


double xCoord;
double yCoord;
double theta;

void setup() {
  a.attach(6);

  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);
  pinMode(m3p1, OUTPUT);
  pinMode(m3p2, OUTPUT);
  pinMode(m4p1, OUTPUT);
  pinMode(m4p2, OUTPUT);

  Enes100.begin("Crash Site Team", CRASH_SITE, 359, 53, 51);

  Enes100.print("Destination is at (");
  updateOTVLocation();
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y);
  Enes100.println(")");
  /*
    // Any other setup code...
  //For color
   Serial.begin(9600);
  Serial.println("Color View Test!");

  //Setup Laser Sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;



  }



  //setup Ultrasonic Sensors
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(echoPinL, INPUT);
  Serial.begin(9600);
  */
  Enes100.println("setup done");

}

void loop() {
  //printLocation();
  Enes100.println("loop begun");

 moveToMissionSite();
 Enes100.println("move to site done");
stopAllMotors();
delay(100000);
 /*
 dl = getultraleft();
 delay(200); 
 */

 /*
 Serial.println("DistanceL ");
 Serial.println(dl);
 */
/*
  xCoord = Enes100.location.x;
  yCoord = Enes100.location.y;
  theta = Enes100.location.theta;

  printLocation();

  if (yCoord < 1) {
    turnInPlace(1.271, 1.871);
    updateOTVLocation();
    delay(1000);
    while (yCoord < 1.5) {
      driveForward(500);
    }
    stopAllMotors();


  } else {
    turnInPlace(-1.371, -1.771);
  }

*/





}

void printLocation() {

  while(!Enes100.updateLocation()) {
        Enes100.println("404 Not Found");
 }  
  Enes100.print("OSV is at (");
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y); 
  Enes100.print(", ");
  Enes100.print(Enes100.location.theta);
  Enes100.println(")");
}

void stopAllMotors() {
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, LOW);
  digitalWrite(m3p1, LOW);
  digitalWrite(m3p2, LOW);
  digitalWrite(m4p1, LOW);
  digitalWrite(m4p2, LOW);
}

void turnInPlace(double target) {
  Enes100.println("started turning");
  while (theta > target+.05 or theta < target-.05) {


    digitalWrite(m1p1, LOW);
    digitalWrite(m1p2, HIGH);

    digitalWrite(m2p1, HIGH);
    digitalWrite(m2p2, LOW);

    digitalWrite(m3p1, HIGH);
    digitalWrite(m3p2, LOW);

    digitalWrite(m4p1, LOW);
    digitalWrite(m4p2, HIGH);
    delay(200);
   
    Enes100.updateLocation();
    theta = Enes100.location.theta;
    Enes100.println(theta);

  }
  stopAllMotors();
}

void driveForward(int d) {

  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, HIGH);

  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, HIGH);

  digitalWrite(m3p1, LOW);
  digitalWrite(m3p2, HIGH);

  digitalWrite(m4p1, LOW);
  digitalWrite(m4p2, HIGH);
  delay(d);
}

void updateOTVLocation() {
  Enes100.updateLocation();
  xCoord = Enes100.location.x;
  yCoord = Enes100.location.y;
  theta = Enes100.location.theta;
  
}

int getred() {

  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");
  return(int(red));
}

void dropmarker() {
  a.write(180);
}

void moveToMissionSite() {
  Enes100.println("begun moving to site");
  updateOTVLocation();
  if (yCoord < 1) {
      Enes100.println("in location 1");
    turnInPlace(1.57);
    while (yCoord < 1.35) {
      Enes100.println("moving");
      driveForward(200);
      stopAllMotors();
      updateOTVLocation;
    }

  }
  else {
      Enes100.println("in location 2");
    turnInPlace(-1.57);
    while (yCoord > .65) {
      Enes100.println("moving");
      driveForward(200);
      stopAllMotors();
      updateOTVLocation();
    }

  }
}
int getultraleft() {
  long durationL;
  int distanceL;


  // Clears the trigPin condition
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationL = pulseIn(echoPinL, HIGH);
  // Calculating the distance
  distanceL = durationL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  //Serial.print("DistanceL: ");
  //Serial.print(distanceL);
  //Serial.println(" cm");
  return distanceL;
}

void getultraright() {
  long durationR;
  int distanceR;

   // Clears the trigPin condition
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationR = pulseIn(echoPinR, HIGH);
  // Calculating the distance
  distanceR = durationR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("DistanceR: ");
  Serial.print(distanceR);
  Serial.println(" cm");

}
