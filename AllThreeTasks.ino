#include "WiFi.h"
#include "html510.h"
#include "motor.h"
#include <Wire.h>
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>
//#include <Servo.h>
#include <PID_v1_bc.h>

#define RGBLED 18 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 5 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 145 // choose a teammembers assigned IP number
#define teamNumber 19
#define FREQ 1 // in Hz

#define MOTOR1_CHANNEL 0 
#define MOTOR2_CHANNEL 1 
#define RESOLUTION_BITS 12
#define RESOLUTION  ((1 << RESOLUTION_BITS)-1) 
#define FREQ_HZ 300

#define MOTOR1_INPUT1 37
#define MOTOR1_INPUT2 38 
#define enableA 15
#define encoder1A 8     // Replace with your actual pin numbers
#define encoder1B 9

#define MOTOR2_INPUT1 39
#define MOTOR2_INPUT2 40
#define enableB 16
#define encoder2A 10
#define encoder2B 11

#define QMC5883L_ADDRESS 0x0D // I2C address for QMC5883L

bool wallfollow = false;
bool pushcar = false;
bool beacon = false;

const int trigPin = 33;  // Connect to the Trig pin on the sensor
const int echoPin = 34; // Connect to the Echo pin on the sensor
const int trigPin2 = 3;  // Connect to the Trig pin on the sensor
const int echoPin2 = 4;
const int inputPin = 6;

long duration;

volatile unsigned long pulseCount = 0;
volatile unsigned long startTime = 0;
volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

const int bufferLength = 3;
int distanceBuffer[bufferLength];
int bufferIndex = 0;
int distance;

double LinSetpoint, LinInput, LinOutput;
double AngSetpoint, AngInput, AngOutput;

float KpLin = 7;
float KiLin = 0;
float KdLin = 2;

float KpAng = 7;
float KiAng = 0;
float KdAng = 2;

PID pidLinear(&LinInput, &LinOutput, &LinSetpoint, KpLin, KiLin, KdLin, DIRECT);
PID pidAng(&AngInput, &AngOutput, &AngSetpoint, KpAng, KiAng, KdAng, DIRECT);

//Servo servoLeft;
//Servo servoRight;
HTML510Server h(80);
Vive510 vive1(SIGNALPIN1);

const char* ssid = "Team19";
const char* password = "";

WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast
IPAddress myIPaddress(192, 168, 1, 145); // change to your IP

////////// COMMUNICATIONS /////

int handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];
   int xf;
   int yf;

   int cb = UDPTestServer.parsePacket(); // if there is no message cb=0
   if (cb) {
      
      packetBuffer[13]=0; // null terminate string

    UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE);
      xf = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
      yf = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
      Serial.print("From Team ");
      Serial.println((char *)packetBuffer);
      Serial.println(xf);
      Serial.println(yf);
   }
   return xf, yf;
}

void UdpSend(int x, int y)
{
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x,y);                                   
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}

////// PID DISTANCE //////

void moveForward(int desiredDistance) {
    double distancePerCount = 1.0;  // Adjust this based on your robot's characteristics
    long encoder1PrevPos = encoder1Pos;
    long encoder2PrevPos = encoder2Pos;

    digitalWrite(MOTOR1_INPUT1, LOW);
    digitalWrite(MOTOR1_INPUT2, HIGH);
    digitalWrite(MOTOR2_INPUT1, LOW);
    digitalWrite(MOTOR2_INPUT2, HIGH);

    // Calculate the time required to cover the desired distance
    double timeRequired = (desiredDistance / distancePerCount) * 50;  // Adjust the factor based on your robot

    analogWrite(enableA, 140);
    analogWrite(enableB, 140);

    unsigned long startTime = millis();

    while ((millis() - startTime) < timeRequired) {
        LinSetpoint = desiredDistance;
        LinInput = (encoder1Pos + encoder2Pos) / 2;
        pidLinear.Compute();
        adjustMotorSpeeds(140, LinOutput);
    }

    // Stop the motors after covering the desired distance
    analogWrite(enableA, 0);
    analogWrite(enableB, 0);
}

void adjustMotorSpeeds(int baseSpeed, double correction) {
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    // Clip speeds to stay within valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    analogWrite(enableA, rightSpeed);
    analogWrite(enableB, leftSpeed);
}

void updateEncoder1() {
    if (digitalRead(encoder1B) == HIGH) {
        encoder1Pos++;
    } else {
        encoder1Pos--;
    }
}

void updateEncoder2() {
    if (digitalRead(encoder2B) == HIGH) {
        encoder2Pos++;
    } else {
        encoder2Pos--;
    }
}

/////// ULTRASONIC SENSORS ////////

void updateBuffer(int distance) {
  distanceBuffer[bufferIndex] = distance;
  bufferIndex = (bufferIndex + 1) % bufferLength;
}

void countPulse() {
  pulseCount++;
}

///////// MAGNETOMETER //////////
float getAngle() {
  int16_t x, y, z;
  // Read magnetometer data
  readMagData(&x, &y, &z);
  // Calculate heading in degrees
  float angle = atan2(y, x) * (180.0 / PI);
  // Ensure the heading is between 0 and 360 degrees
  if (angle < 0) {
    angle += 360.0;
  }
  Serial.println("Angle: " + String(angle));
  delay(500); // Adjust delay as needed for your application
  return angle;
}

void readMagData(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x00); // Register address for data
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDRESS, 6);
  while (Wire.available() < 6);
  *x = Wire.read() | (Wire.read() << 8);
  *z = Wire.read() | (Wire.read() << 8);
  *y = Wire.read() | (Wire.read() << 8);
}

void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}


///// ULTRASONIC
bool isCloseToWall() {
  // Only consider the last three measurements in the buffer
  int sum = 0;
  for (int i = 0; i < bufferLength; i++) {
    sum += distanceBuffer[i];
  }
  int averageDistance = sum / bufferLength;
  // Adjust the threshold as needed
  return averageDistance < 25; // Example threshold: 20 cm
}

void stop(){
    analogWrite(enableA, 0); 
    analogWrite(enableB, 0);
    digitalWrite(MOTOR1_INPUT1, LOW);
    digitalWrite(MOTOR1_INPUT2, HIGH);
    digitalWrite(MOTOR2_INPUT1, LOW);
    digitalWrite(MOTOR2_INPUT2, HIGH);
    delay(50);
} 

int getDistance(int trigPin, int echoPin) {
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Measure the duration of the pulse from the echo pin
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance in centimeters (you can adjust the speed of sound as needed)
  distance = duration * 0.034 / 2;
  return distance;
}

//// PID ANGLE ///////
void turnAngle(double angle) {
  if (angle < 0) {
    leftTurnSpeed(abs(angle));
  }
  else if (angle > 0) {
    rightTurnSpeed(abs(angle));
  }
  else {
    stop();
  }
}

void leftTurnSpeed(int speed) {
  digitalWrite(MOTOR1_INPUT1, LOW);
  digitalWrite(MOTOR1_INPUT2, LOW);
  digitalWrite(MOTOR2_INPUT1, LOW);
  digitalWrite(MOTOR2_INPUT2, HIGH);
  analogWrite(enableA, 0);  // RIGHT
  analogWrite(enableB, speed);
}

void rightTurnSpeed(int speed) {
  digitalWrite(MOTOR1_INPUT1, LOW);
  digitalWrite(MOTOR1_INPUT2, HIGH);
  digitalWrite(MOTOR2_INPUT1, LOW);
  digitalWrite(MOTOR2_INPUT2, LOW);
  analogWrite(enableA, speed);
  analogWrite(enableB, 0);  // LEFT
}

void wallFollowing(){
  int d = 25;
  int distance;
  int sidedistance;
  int sum1 = 0;
  int sum2 = 0;
  // updateBuffer(distance);

  for(int i = 0; i < 5; i++){ 
    distance = getDistance(trigPin, echoPin);
    sidedistance = getDistance(trigPin2, echoPin2);
    sum1 = sum1 + distance;
    sum2 = sum2 + sidedistance;
  }

  int forward = sum1 / 5;
  int side = sum2 / 5;
  Serial.println(forward);
  Serial.println(side);
  // Corner case: handling turns
  if (forward < 30) {
      if (side < 28) {
        analogWrite(enableA, 0);
        analogWrite(enableB, 0);
        digitalWrite(MOTOR1_INPUT1, LOW);
        digitalWrite(MOTOR1_INPUT2, LOW);
        digitalWrite(MOTOR2_INPUT1, LOW);
        digitalWrite(MOTOR2_INPUT2, LOW);
        delay(500);

        Serial.println("Turn");
        analogWrite(enableB, 100);
        digitalWrite(MOTOR2_INPUT1, LOW);
        digitalWrite(MOTOR2_INPUT2, HIGH);
        delay(500); 
      }
      Serial.println("Turn");
      analogWrite(enableB, 100);
      digitalWrite(MOTOR2_INPUT1, LOW);
      digitalWrite(MOTOR2_INPUT2, HIGH);
      delay(500); 
  }

  if (forward > 30) {
    Serial.println("Forward");
    if (side < 20) {

      AngSetpoint = 100;
      AngInput = getAngle();
      pidAng.Compute();
      turnAngle(AngOutput);
      turnAngle(100);
    }
    if (side > 25) {
      AngSetpoint = 50;
      AngInput = getAngle();
      pidAng.Compute();
      turnAngle(AngOutput);
    }
    moveForward(200);
  }
}

///// PUSH POLICE CAR /////
void vivefunc(){
  static long int ms = millis();
  static uint16_t xi,yi;

  int xf;
  int yf;

  xf, yf = handleUDPServer();
  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(xi,yi);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    xi = vive1.xCoord();
    yi = vive1.yCoord();
    neopixelWrite(RGBLED,0,xi/200,yi/200);  // blue to greenish
  }
  else {
    xi=0;
    yi=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  delay(20);
  int xit = xi;
  int yit = yi;
  pushPoliceCar(xit, yit, xf, yf);
}

void gripperPushMode(){
  //servoLeft.write(83);
  //servoRight.write(98);
}

void pushPoliceCar(int xi, int yi, int xf, int yf) {
  gripperPushMode();
  float orientationForPoliceCar = atan2(yf - yi, xf - xi);
  // Rotating the robot to face the police car 
  
  AngSetpoint = orientationForPoliceCar;
  AngInput = getAngle();
  pidAng.Compute();
  turnAngle(AngOutput);
  
  moveForward(xi-xf);
  
  if (yf - yi >= 35){
    AngSetpoint = 90;
    AngInput = getAngle();
    pidAng.Compute();
    turnAngle(AngOutput);
  }

  else if (yi - yf >= 35){
    AngSetpoint = -90;
    AngInput = getAngle();
    pidAng.Compute();
    turnAngle(AngOutput);
  }

  moveForward(yi-yf);

  float distanceToPoliceCar = sqrt((xi - xf)*(xi-xf) + (yi -yf)*(yi-yf));
  float tol = 5;

  // Move only if the distance is above a threshold
  if (distanceToPoliceCar > tol) {
    int movement = 0;
    while (movement < 10) {
      moveForward(100);
      Serial.println("Pushing police car");
      handleUDPServer();
      movement = sqrt((xi - xf)*(xi-xf) + (yi -yf)*(yi-yf));
    }
  }
}

////// BEACON TRACKING /////////

void trackBeacon(){
  //servoLeft.write(0);  // 0 degree position is the fully open position
  //servoRight.write(180); //180 degree position is the fully open position

  // Record the start time
  startTime = micros();
  pulseCount = 0;

  // Wait for one second
  while (micros() - startTime < 1000000UL) {
  }

  // Detach the interrupt during calculations
  detachInterrupt(digitalPinToInterrupt(inputPin));

  // Calculate frequency in Hertz
  float frequency = pulseCount / 2.0;  // Divided by 2 because we are counting rising edges

  // Print the result
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  // Reattach the interrupt for the next iteration
  attachInterrupt(digitalPinToInterrupt(inputPin), countPulse, RISING);
  // Wait for 1 second before measuring again
  delay(5);

 if ((frequency < 24) && (frequency > 22)) {
  moveForward(100);
  neopixelWrite(RGBLED,255,255,255); 
  Serial.print("Going for the trophy ");
  grabBeacon();
 } 
 else if ((frequency < 552) && (frequency > 548)){
  moveForward(100);
  neopixelWrite(RGBLED,255,255,255); 
  Serial.print("Going for the fake trophy");
  pushBeacon();
 }
 else {
    AngSetpoint = 25;
    AngInput = getAngle();
    pidAng.Compute();
    turnAngle(AngOutput);
  Serial.print("turn");
  neopixelWrite(RGBLED,128,0,0);  
 }
}

void grabBeacon(){
  // Wait for the servos to reach the desired position
  delay(1000);
  int distance = getDistance(trigPin, echoPin);
  // If an object is detected within a certain range, close the servos
  if (distance < 2) {
    //servoLeft.write(180);  // 180 degree position is the fully closed position
    //servoRight.write(0); // 0 degree position is the fully closed position
    delay(1000);
  }
}

void pushBeacon(){
  grabBeacon();
  moveForward(100);
}

void handleRoot() {
  Serial.println("handleRoot");
  h.sendhtml(motor);
}

void handleMode() {
  Serial.println("Mode selected: ");
  int Mode = h.getVal();
  if (Mode == 1){
    wallfollow = true;
    pushcar = false;
    beacon = false;
  }

  else if (Mode == 4){
    wallfollow = false;
    Serial.println("Push Police Car");
    vivefunc();
    pushcar = true;
    beacon = false;

  } 
  else if (Mode == 0){
    wallfollow = false;
    Serial.println("Stop");
    pushcar = false;
    beacon = false;
    stop();
  }
  else if (Mode == 2){
    wallfollow = false;
    pushcar = false;
    beacon = true;
    Serial.println("Beacon Track");
  }
}

void setup() {
  int i=0;
  Serial.begin(115200);
  WiFi.softAP(ssid, password);

  delay(1000);

  IPAddress myIP = WiFi.softAPIP();
  Serial.println("Access Point IP address: ");
  Serial.println(myIP);

  Serial.printf("team  #%d ", teamNumber); 
  Serial.print("Connecting to ");  Serial.println(ssid);
  
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   Serial.print(".");
  }
  
  if (i<19) {
    Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
  } 
  else {
    Serial.printf("Could not connect err: %d ",i); 
  }
  
  UDPTestServer.begin(UDPPORT);

  h.begin();
  h.attachHandler("/Mode?val=", handleMode);
  h.attachHandler("/", handleRoot);

  pinMode(MOTOR1_INPUT1, OUTPUT);
  pinMode(MOTOR1_INPUT2, OUTPUT);
  pinMode(enableA, OUTPUT);

  pinMode(MOTOR2_INPUT1, OUTPUT);
  pinMode(MOTOR2_INPUT2, OUTPUT);
  pinMode(enableB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(inputPin, INPUT);
  
  //servoLeft.attach(13);  
  //servoRight.attach(14);

  attachInterrupt(digitalPinToInterrupt(inputPin), countPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), updateEncoder2, CHANGE);

  Wire.begin(); // Initialize QMC5883L for magnetometer I2C communication
  writeRegister(QMC5883L_ADDRESS, 0x0B, 0x01); // Set mode to continuous measurement
  Serial.println("Setup complete");

  pidLinear.SetMode(AUTOMATIC);
  pidAng.SetMode(AUTOMATIC);

  pidLinear.SetOutputLimits(0, 255);  // Adjust these limits based on your motor controller
  pidAng.SetOutputLimits(0, 255);

  LinSetpoint = 0;
  AngSetpoint = 0;
}

void loop() {

  h.serve();
  if (pushcar) {
    vivefunc();
  }
  if (wallfollow) {
    wallFollowing();
  }
  if(beacon) {
    trackBeacon();
  }
}