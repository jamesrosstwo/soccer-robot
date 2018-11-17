#include <SPI.h>
#include <Wire.h>             //Include the Wire Library
#include <HTInfraredSeeker.h> //Include the IR Seeker Library
#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// L9958 slave select pins for SPI
#define SS_M4 14
#define SS_M3 13
#define SS_M2 12
#define SS_M1 11
// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7
// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
#define PWM_M4 6     // Timer0
#define ENABLE_MOTORS 8
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

long microsecondsToCentimeters(long microseconds)
{
 return microseconds / 29 / 2;
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
int MotorDir[]={DIR_M1,DIR_M2,DIR_M3,DIR_M4};
int MotorStr[]={PWM_M1,PWM_M2,PWM_M3,PWM_M4};
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
    digitalWrite(MotorDir[1],0);
    digitalWrite(MotorDir[3],0);
    
  }
  else
  {
    
    digitalWrite(MotorDir[1],1);
    digitalWrite(MotorDir[3],1);
    
  }
  if (m0_2 < 0)
  {
  
    digitalWrite(MotorDir[0],0);
    digitalWrite(MotorDir[2],0);
    
  }
  else
  {
    digitalWrite(MotorDir[0],1);
    digitalWrite(MotorDir[2],1);
  }
  analogWrite(MotorStr[0],m0_2);
  analogWrite(MotorStr[2],m0_2);
  analogWrite(MotorStr[1],m1_3);
  analogWrite(MotorStr[3],m1_3);
}

//moves robot in 360 degree direction with heading.
void moveRobotHeading(int heading, int str){
  float x = cos((-heading + 90) * (PI / 180)) * str; //offset so 0 is the front of the robot, and goes clockwise
  float y = sin((heading + 90) * (PI / 180)) * str;
  moveRobot((int)x, (int)y);
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

void testMotors(){
  moveRobotHeading(0, 100);
  delay(1000);
  moveRobotHeading(90, 100);
  delay(1000);
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

//  for (int i = 0; i < 360; i++){
//    moveRobotHeading(i, 100);
//    delay(30);
//  }
}

void setup(){
  unsigned int configWord;
  Serial.begin(250000); // set baud rate to 250k
  Serial.println("Motor test!");
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, LOW);  // HIGH = not selected
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, LOW);
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, LOW);
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, LOW);

  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);

  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0

  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT); 
 digitalWrite(ENABLE_MOTORS, HIGH);  // HIGH = disabled
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH);
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);

  //Set initial actuator settings to pull at 0 speed for safety


digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled  
InfraredSeeker::Initialize();
}
void loop(){
  Serial.println("running");
  Serial.println(IRDir());

  setupAccelGyro();
  readAccelGyro();
}

