#include <AFMotor.h>
#include <Wire.h>             //Include the Wire Library
#include <HTInfraredSeeker.h> //Include the IR Seeker Library
#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

class Grayscale{
  private:
    int result;
    int pin;

  public:
    Grayscale(int pin_num);
    int readShade();
};

Grayscale::Grayscale(int pin_num){
  pin = pin_num;
}
int Grayscale::readShade(){
  return analogRead(pin);
}

class PingSensor{
private:
  int pin;
  long distance;

public:
  PingSensor(int pin_num);
  long readDist();
};

PingSensor::PingSensor(int pin_num){
  pin = pin_num;
}
long microsecondsToCentimeters(long in){
  return in/58;
}
//from https://www.arduino.cc/en/Tutorial/Ping
long PingSensor::readDist(){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  distance = microsecondsToCentimeters(pulseIn(pin, HIGH));
  return distance;
}

/* Pin overview:
 * 7-10: Grayscales
 * 11-14: Pings
 */
// motors will be in the order of front-left first, then clockwise from there.
AF_DCMotor frontLeftMotor(3);
AF_DCMotor frontRightMotor(2);
AF_DCMotor backLeftMotor(4);
AF_DCMotor backRightMotor(1);
AF_DCMotor motors[4] = {frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor};
const int leftG=15;
const int  frontG=14;
const int rightG=12;
const int backG=13;
int grayScale = 7;

Grayscale fGrayscale(7);
Grayscale rGrayscale(8);
Grayscale bGrayscale(9);
Grayscale lGrayscale(10);
//Grayscale grayscales[4] = {fGrayscale, rGrayscale, bGrayscale, lGrayscale};

PingSensor fPingSensor(11);
PingSensor rPingSensor(12);
PingSensor bPingSensor(13);
PingSensor lPingSensor(14);
PingSensor pingSensors[4] = {fPingSensor, rPingSensor, bPingSensor, lPingSensor};


MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

void moveRobot(int xSpeed, int ySpeed)
{

  float m0_2 = ySpeed + (xSpeed / 2);
  float m1_3 = ySpeed - (xSpeed / 2);
  m0_2=map(m0_2,0,380,0,255);
  m1_3=map(m1_3,0,380,0,255);
 // Serial.println("zero and two");
 Serial.println(m0_2);
 Serial.println(m1_3);
  if (m1_3 < 0)
  {
    motors[1].run(BACKWARD);
    motors[3].run(BACKWARD);
  }
  else
  {
    motors[1].run(FORWARD);
    motors[3].run(FORWARD);
  }
  if (m0_2 < 0)
  {
    motors[0].run(BACKWARD);
    motors[2].run(BACKWARD);
  }
  else
  {
    motors[0].run(FORWARD);
    motors[2].run(FORWARD);
  }
  motors[0].setSpeed(abs(m0_2));
  motors[1].setSpeed(abs(m1_3));
  motors[2].setSpeed(abs(m0_2));
  motors[3].setSpeed(abs(m1_3));
}

void stopRobot(){
  moveRobot(0, 0);
}

void setupAccelGyro(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void readAccelGyro(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  #ifdef OUTPUT_READABLE_ACCELGYRO
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
  #endif
}

int IRStr(){
  InfraredResult readIn = InfraredSeeker::ReadAC();
  int out = readIn.Strength;
  return out;
}

int IRDir(){
  InfraredResult readIn = InfraredSeeker::ReadAC();
  int out = readIn.Direction;
  return out;
}
<<<<<<< HEAD:soccer-robot.ino
int IRStr(){
InfraredResult readIn=InfraredSeeker::ReadAC();
  int out=readIn.Strength;
  return out;
  
}
void testMotors()
{
  Serial.println("one");
  moveRobotHeading(0, 100);
  delay(1000);
  Serial.println("two");
  moveRobotHeading(90, 100);
  delay(1000);
  Serial.println("three");  
=======

void testMotors(){
  moveRobotHeading(0, 100);
  delay(1000);
  moveRobotHeading(90, 100);
  delay(1000);
>>>>>>> IR:code/code.ino
  moveRobotHeading(180, 100);
  delay(1000);

  Serial.println("mark");

  moveRobotHeading(270, 100);
  delay(1000);
  moveRobotHeading(45, 100);
  delay(1000);
  moveRobotHeading(135, 100);
  delay(1000);
  moveRobotHeading(225, 100);
  delay(1000);
  moveRobotHeading(315, 100);
  delay(1000);

  for (int i = 0; i < 360; i++){
    moveRobotHeading(i, 100);
    delay(30);
  }
}

void setup(){
  Serial.begin(250000); // set baud rate to 250k
  Serial.println("Motor test!");
<<<<<<< HEAD:soccer-robot.ino
Serial.println("Dir\tStrength"); //Prints Dir & Strength at top
  //InfraredSeeker::Initialize(); //initializes the IR sensor
  
}

void loop()
{
  Serial.println("running");
  moveRobot(200,200);

  setupAccelGyro();
  readAccelGyro();
}
