#include <AFMotor.h>
#include <Wire.h> //Include the Wire Library
#include <HTInfraredSeeker.h> //Include the IR Seeker Library
// motors will be in the order of front-left first, then clockwise from there.
AF_DCMotor frontLeftMotor(1);
AF_DCMotor frontRightMotor(2);
AF_DCMotor backLeftMotor(3);
AF_DCMotor backRightMotor(4);
AF_DCMotor motors[4] = {frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor};

int grayScale = 7;

void moveRobot(int xSpeed, int ySpeed)
{

  float y = map(ySpeed, 0, 255, 0, 180) * sqrt(2);
  float x = map(xSpeed, 0, 255, 0, 180) * sqrt(2);
  float m0_2 = y + (x / 2);
  float m1_3 = y - (x / 2);
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
    motors[1].run(BACKWARD);
    motors[3].run(BACKWARD);
  }
  else
  {
    motors[1].run(FORWARD);
    motors[3].run(FORWARD);
  }
  motors[0].setSpeed(abs(m0_2));
  motors[1].setSpeed(abs(m1_3));
  motors[2].setSpeed(abs(m0_2));
  motors[3].setSpeed(abs(m1_3));
}

//moves robot in 360 degree direction with heading.
void moveRobotHeading(int heading, int str)
{
  float x = cos((-heading + 90) * (PI / 180)) * str; //offset so 0 is the front of the robot, and goes clockwise
  float y = sin((heading + 90) * (PI / 180)) * str;
  moveRobot((int)x, (int)y);
}

void stopRobot()
{
  moveRobot(0, 0);
}

class Grayscale
{
private:
  int result;
  int pin;

public:
  Grayscale(int pin_num);
  int readShade();
};

Grayscale::Grayscale(int pin_num)
{
  pin = pin_num;
}
int Grayscale::readShade()
{
  return analogRead(pin);
}

//from https://www.arduino.cc/en/Tutorial/Ping
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

class PingSensor
{
private:
  int pin;
  long distance;

public:
  PingSensor(int pin_num);
  long readDist();
};

PingSensor::PingSensor(int pin_num)
{
  pin = pin_num;
}

long PingSensor::readDist()
{
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

int IRDir(){
  InfraredResult readIn=InfraredSeeker::ReadAC();
  int out=readIn.Direction;
  return out;
}
int IRStr(){
InfraredResult readIn=InfraredSeeker::ReadAC();
  int out=readIn.Strength;
  return out;
  
}
void testMotors()
{
  moveRobotHeading(0, 100);
  moveRobotHeading(90, 100);
  moveRobotHeading(180, 100);
  moveRobotHeading(270, 100);
}

void setup()
{
  Serial.begin(250000); // set baud rate to 250k
  Serial.println("Motor test!");
Serial.println("Dir\tStrength"); //Prints Dir & Strength at top
  InfraredSeeker::Initialize(); //initializes the IR sensor
  
}

void loop()
{
   InfraredResult InfraredBall = InfraredSeeker::ReadAC();
   
  Serial.println(IRDir());
  delay(100); //delay a tenth of a second
  // put your main code here, to run repeatedly:
}
