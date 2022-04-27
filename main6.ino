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

//Laser
unsigned char ok_flag;
unsigned char fail_flag;
unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;

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
  
  Serial.begin(9600);
  Enes100.begin("Crash Site Team", CRASH_SITE, 359, 53, 51);
 Enes100.println("Phase 1: Setup");
 delay(1000);
    
  Enes100.println("Current location");
  delay(500);
  updateOTVLocation();
  Enes100.print(Enes100.location.x);
  delay(20);
  Enes100.print(", ");
  delay(20);
  Enes100.print(Enes100.location.y);
  delay(20);
  Enes100.println(")");
  delay(500);

  //Laser setup
Wire.begin(); 
  printf_begin(); 
  
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
  
   Serial.println("setup done");
   delay(10);
}

void loop() {
  //printLocation();
 Serial.println("beginning loop");
 delay(10);
  Enes100.println("loop begun");
  delay(1000);
  
 String side;
 moveToMissionSite(side);
 Enes100.println("move to site done");
 delay(1000);
stopAllMotors();
delay(1000);
Enes100.println("starting box phase");

bool progress = false;

updateOTVLocation();
delay(10);
bool z = rlone();
Enes100.println("if you see this there is an error");
delay(20);
/*
if (side == "l" && progress == false) {
  progress = slone();
  progress = sltwo();
  progress = slthree();
  progress = slfour():
}

if (side == "r" && progress == false) {
  progress = rlone();
  //progress = rltwo();
  //progress = rlthree();
  //progress = rlfour();
}
*/
/*if (progress == false) {
  Enes100.println("Side one is not red");
  progress = redfindneartwo();
}
if (progress == false) {
  Enes100.println("Side two is not red");
  progress = redfindnearthree();
}
if (progress == false) {
  Enes100.println("Side three is not red");
  progress = redfindnearfour();
}
if (progress == false) {
  Enes100.println("if you see this message, there is an error");
}
*/

/*
else {
  //redfindfar();
  //if (yCoord<1){
  //progress = redfindnearone();
//}
//if (progress == false) {
  //Enes100.println("Side one is not red");
  //progress = redfindfartwo();
//}
//if (progress == false) {
  //Enes100.println("Side two is not red");
  //progress = redfindfarthree();
//}
//if (progress == false) {
  //Enes100.println("Side three is not red");
  //progress = redfindfarfour();
//}
//if (progress == false) {
  //Enes100.println("if you see this message, there is an error");
//}
//}

*/
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
  delay(10);
  while (theta > target+.04 or theta < target-.04) {


    digitalWrite(m1p1, LOW);
    digitalWrite(m1p2, HIGH);

    digitalWrite(m2p1, HIGH);
    digitalWrite(m2p2, LOW);

    digitalWrite(m3p1, HIGH);
    digitalWrite(m3p2, LOW);

    digitalWrite(m4p1, LOW);
    digitalWrite(m4p2, HIGH);
    delay(20);
   stopAllMotors();
   delay(50);
    Enes100.updateLocation();
    theta = Enes100.location.theta;
    Enes100.println(theta);
      delay(20);
  }
  stopAllMotors();
  Enes100.println("turn complete");
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
void driveLeft(int d) {
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, HIGH);

  digitalWrite(m2p1, HIGH);
  digitalWrite(m2p2, LOW);

  digitalWrite(m3p1, LOW);
  digitalWrite(m3p2, HIGH);

  digitalWrite(m4p1, HIGH);
  digitalWrite(m4p2, LOW);
  delay(d);
}
void driveRight(int d) {
  digitalWrite(m1p1, HIGH);
  digitalWrite(m1p2, LOW);

  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, HIGH);

  digitalWrite(m3p1, HIGH);
  digitalWrite(m3p2, LOW);

  digitalWrite(m4p1, LOW);
  digitalWrite(m4p2, HIGH);
  delay(d);
}

void updateOTVLocation() {
  Enes100.updateLocation();
  Enes100.println("location updated");
  delay(500);
  xCoord = Enes100.location.x;
  yCoord = Enes100.location.y;
  theta = Enes100.location.theta;
  
}

int getred() {
   Serial.println("getred loop started");
   delay(20);
  float red, green, blue;
  int highred = 0;
  //tcs.setInterrupt(false);  // turn on LED
  for (int n = 0; n<10; n++) { 
  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
   Serial.println("tcs get rgb");
  delay(100);
  //tcs.setInterrupt(true);  // turn off LED

  //Serial.print("R:\t"); Serial.print(int(red)); 
  //Serial.print("\tG:\t"); Serial.print(int(green)); 
  //Serial.print("\tB:\t"); Serial.print(int(blue));

//  Serial.print("\t");
//  Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  //Serial.print("\n");
  Enes100.println("Current value of red: ");
 delay(20);
  Enes100.println(int(red));
  delay(20);
  if (int(red) > highred) {
    highred = int(red);
  }
  }
   Enes100.println("Highest value of red: ");
 delay(20);
  Enes100.println(highred);
  delay(20);
  return(highred);
}

void dropmarker() {
  a.write(180);
}

void moveToMissionSite(String a) {
  Enes100.println("Phase 2: Moving to Site");
  delay(100);
  updateOTVLocation();
  float currentd = getLaser();
  if (yCoord < 1) {
      Enes100.println("in right starting location");
      delay(100);
       a = "l";
    turnInPlace(1.57);
    while (yCoord < 1.6 && currentd > 60) {
      Enes100.println("moving");
      delay(50);
      driveForward(100);
      stopAllMotors();
      delay(50);
      updateOTVLocation;
      delay(10);
      currentd  = getLaser();
      delay(10);
    }

  }
  
  else {
      Enes100.println("in left starting location");
      delay(100);
       a = "r";
    turnInPlace(-1.57);
    while (yCoord > .65 && currentd > 60) {
      Enes100.println("moving");
      delay(50);
      driveForward(100);
      stopAllMotors();
      delay(50);
      updateOTVLocation;
      delay(10);
      currentd  = getLaser();
      delay(10);
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
float getLaser() {
  SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(300); 
    return lenth_val;
}

 bool rlone() {
  bool x = false;
  Enes100.println("begun rlone");
  delay(20);
  int cur = getred();
  delay(5000);
  if (cur > 150) {
    delay(10);
    x = true;
    Enes100.println("red side found");
    delay(10);
    updateOTVLocation();
    delay(10);
    Enes100.println("Red Side X: ");
    delay(10);
     Enes100.println(xCoord);
     delay(10);
     Enes100.println("Red Side Y: ");
     delay(10);
     Enes100.println(yCoord);
     delay(10);
    delay(100);
  }
   if (cur < 150) {
    delay(10);
    Enes100.println("this side is not red");
    delay(100);
  }
  if (x == true) {
    float currentdist;
     int color;
    float xonestore;
     float xtwostore;
    float yonestore;
    float ytwostore;
    float meas;
    currentdist = getLaser();
    float upperlim = currentdist + 10;
      Enes100.println("driving to left edge");
      delay(100);
    while(currentdist < upperlim) {
      driveLeft(40); 
      stopAllMotors();
      delay(200);
      currentdist = getLaser();
      delay(50);
      updateOTVLocation();
    }
      
      xonestore = xCoord;
     Enes100.println("driving to right edge");
     delay(500);
     driveRight(20);
      stopAllMotors;
      delay(100);
    while(currentdist < upperlim) {
      driveRight(40);
      stopAllMotors();
      delay(200);
      currentdist = getLaser();
      delay(50);
      updateOTVLocation();
    }
    updateOTVLocation();
    xtwostore = xCoord;
    meas = xtwostore-xonestore;
    
    Enes100.mission(LENGTH, meas);
    delay(100);
    Enes100.println("distance is");
    delay(50);
    Enes100.println(meas);
    delay(100);
 }
 if (x == false) {
      Enes100.println("Moving to next side");
 }
 return x;
 }
 /*
bool redfindnearone() {
   Enes100.println("started redfindnearone");
   delay(100);
  float currentdist;
  int color;
  float xonestore;
  float xtwostore;
  float yonestore;
  float ytwostore;
  float meas;
  bool finished = false;
  //turnInPlace(3.141593/2);
  //dropmarker();
  //driveForward(50);
  //stopAllMotors;
  color = getred();
  if (color > 140) {
    //currentdist = getLaser();
    //upperlim = currentdist + 10;
      Enes100.println("driving to left edge");
      delay(100);
    while(color>140) {
      driveLeft(20); 
      stopAllMotors;
      delay(200);
      //currentdist = getLaser();
      color = getred();
      delay(50);
    }
    updateOTVLocation();
    xonestore = xCoord;
    Enes100.println("driving to right edge");
    delay(500);
     driveRight(50);
      stopAllMotors;
    while(color >140) {
      driveRight(20);
      stopAllMotors;
      delay(200);
      //currentdist = getLaser();
      color = getred();
      delay(50);
    }
    updateOTVLocation();
    xtwostore = xCoord;
    meas = xtwostore-xonestore;
    
    Enes100.mission(LENGTH, meas);
    delay(100);
    Enes100.println("distance is");
    delay(50);
    Enes100.println(meas);
    delay(100);
    finished = true;
  }  
  return finished;
}
*/



// For laser
int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}
void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}
