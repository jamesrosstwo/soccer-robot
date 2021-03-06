#include <SPI.h>
#include <Wire.h>             //Include the Wire Library
#include <HTInfraredSeeker.h> //Include the IR Seeker Library
#include <I2Cdev.h>

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
#define blackLimit 410
#define whiteLimit 345
#define motorLimit 255
#define ANGLELIMIT 6
int whiteLimits[4]={300,430,300,320};
class Grayscale {
private:
    int result;
    int pin;

public:
    Grayscale(int pin_num);

    int readShade();
};

Grayscale::Grayscale(int pin_num) {
    pin = pin_num;
}

int Grayscale::readShade() {
    return analogRead(pin);
}

class GyroSensor {
private:
    int Address, slave, i, heading;
    byte headingData[2];
public:
    GyroSensor(int useless);

    int getHeading();
};

int startDeg = 0;

GyroSensor::GyroSensor(int useless) {
    Address = 0x42;
    slave = Address >> 1;
    i = 0;
    heading = 0;

}

int GyroSensor::getHeading() {
    Wire.beginTransmission(slave);
    Wire.write("A");              // The "Get Data" command
    Wire.endTransmission();
    delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
    // after this command.  Using 10ms just makes it safe
    // Read the 2 heading bytes, MSB first
    // The resulting 16bit word is the compass heading in 10th's of a degree
    // For example: a heading of 1345 would be 134.5 degrees
    Wire.requestFrom(slave, 2);        // Request the 2 byte heading (MSB comes first)
    i = 0;
    while (Wire.available() && i < 2) {
        headingData[i] = Wire.read();
        i++;
    }
    heading = headingData[0] * 256 + headingData[1];
    return heading / 10;
}

class PingSensor {
private:
    int pin;
    long distance;

public:
    PingSensor(int pin_num);

    long readDist();
};


PingSensor::PingSensor(int pin_num) {
    pin = pin_num;
}

long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
}

//from https://www.arduino.cc/en/Tutorial/Ping
long PingSensor::readDist() {
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


Grayscale fGrayscale(9);
Grayscale rGrayscale(8);
Grayscale bGrayscale(11);
Grayscale lGrayscale(10);
Grayscale grayscales[4] = {fGrayscale, rGrayscale, bGrayscale, lGrayscale};

PingSensor fPingSensor(11);
PingSensor rPingSensor(12);
PingSensor bPingSensor(13);
PingSensor lPingSensor(14);
PingSensor pingSensors[4] = {fPingSensor, rPingSensor, bPingSensor, lPingSensor};


boolean locks[4];
int Motors[][4] =
{{DIR_M2, DIR_M3, DIR_M4, DIR_M1},
                   {PWM_M2, PWM_M3, PWM_M4, PWM_M1}};

void moveRobot(int xSpeed, int ySpeed) {
    ySpeed *= -1;
    float m0_2 = ySpeed + (xSpeed / 2);
    float m1_3 = ySpeed - (xSpeed / 2);
    m0_2 = map(m0_2, 0, 380, 0, 255);
    m1_3 = map(m1_3, 0, 380, 0, 255);
    if (m1_3 < 0) {
        digitalWrite(Motors[0][1], 0);
        digitalWrite(Motors[0][3], 0);

    } else {
        digitalWrite(Motors[0][1], 1);
        digitalWrite(Motors[0][3], 1);

    }
    if (m0_2 < 0) {
        digitalWrite(Motors[0][0], 0);
        digitalWrite(Motors[0][2], 0);

    } else {
        digitalWrite(Motors[0][0], 1);
        digitalWrite(Motors[0][2], 1);
    }
    analogWrite(Motors[1][0], abs(m0_2));
    analogWrite(Motors[1][2], abs(m0_2));
    analogWrite(Motors[1][1], abs(m1_3));
    analogWrite(Motors[1][3], abs(m1_3));
}

//moves robot in 360 degree direction with heading.
void moveRobotHeading(int heading, int str) {
    float x = cos((-heading + 90) * (PI / 180)) * str; //offset so 0 is the front of the robot, and goes clockwise
    float y = sin((heading + 90) * (PI / 180)) * str;
    Serial.print("Heading:");
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    moveRobot((int) x, (int) y);
}

void stopRobot() {
    moveRobot(0, 0);
}


int IRStr() {
    InfraredResult readIn = InfraredSeeker::ReadAC();
    int out = readIn.Strength;
    return out;
}

int IRDir() {
    Serial.print("IR Direction: ");
    InfraredResult readIn = InfraredSeeker::ReadAC();
    
    return readIn.Direction;
    
}

boolean grayscaleWheelLock(int x, int y) {
    return (x < 0 && locks[3]) || (x > 0 && locks[1]) || (y > 0 && locks[0]) || (y < 0 && locks[2]);
}

void defend() {
    float in = (float) IRDir();
    if (in == 0) {
        stopRobot();
        return;
    }

    int x = 0;
    if(in!=0){
      if(in<5){
        x=-motorLimit;
      }
      else if(in>5){
        x=motorLimit;
      }
      else{
        x=0;
      }
    }
    else{
      stopRobot();
      return;
    }
    if (grayscaleWheelLock(x, 0)) {
        stopRobot();
    } else{
        moveRobot(x, 0); 
    }
    
    Serial.print(" ");
    Serial.print(x);
    Serial.print(" ");
    delay(50);
}

void testMotors() {
    Serial.println("Motor test!");
    moveRobotHeading(0, 100);
    delay(1000);
    moveRobotHeading(90, 100);
    delay(1000);
    moveRobotHeading(180, 100);
    delay(1000);
    moveRobotHeading(270, 100);
    delay(1000);
    moveRobot(100, 0);
    delay(1000);
    moveRobot(-100, 0);
    delay(1000);
    moveRobot(0, 100);
    delay(1000);
    moveRobot(0, -100);
    delay(1000);
    turnLeft(100);
    delay(1000);
    turnRight(100);
    delay(1000);
}

GyroSensor gSensor(1);

void initMotors() {
    unsigned int configWord;
    pinMode(SS_M1, OUTPUT);
    digitalWrite(SS_M1, LOW);  // HIGH = not selected
    pinMode(SS_M2, OUTPUT);
    digitalWrite(SS_M2, LOW);
    pinMode(SS_M3, OUTPUT);
    digitalWrite(SS_M3, LOW);
    pinMode(SS_M4, OUTPUT);
    digitalWrite(SS_M4, LOW);

    // L9958 DIRection pins
    pinMode(DIR_M1, OUTPUT);
    pinMode(DIR_M2, OUTPUT);
    pinMode(DIR_M3, OUTPUT);
    pinMode(DIR_M4, OUTPUT);

    // L9958 PWM pins
    pinMode(PWM_M1, OUTPUT);
    digitalWrite(PWM_M1, LOW);
    pinMode(PWM_M2, OUTPUT);
    digitalWrite(PWM_M2, LOW);    // Timer1
    pinMode(PWM_M3, OUTPUT);
    digitalWrite(PWM_M3, LOW);
    pinMode(PWM_M4, OUTPUT);
    digitalWrite(PWM_M4, LOW);    // Timer0

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
}

void turnLeft(int str) {
    digitalWrite(Motors[0][0], 0);
    digitalWrite(Motors[0][1], 0);
    digitalWrite(Motors[0][2], 1);
    digitalWrite(Motors[0][3], 1);
    for (int count = 0; count < 4; count++) {
        analogWrite(Motors[1][count], str);
    }

}

void turnRight(int str) {
    digitalWrite(Motors[0][0], 1);
    digitalWrite(Motors[0][1], 1);
    digitalWrite(Motors[0][2], 0);
    digitalWrite(Motors[0][3], 0);
    for (int count = 0; count < 4; count++) {
        analogWrite(Motors[1][count], str);
    }
}

int degreesAdjust(int in) {
    if (abs(in - startDeg - 360) < 181) {
        in = in - startDeg - 360;
    }else if(abs(in-startDeg+360)<181){
       in = in - startDeg + 360;
    }
    else {
        in = in - startDeg;
    }
    return in;
}

unsigned long timeSoFar = 0;
void reorient() {
    
    if(millis() - timeSoFar <= 0){ //only refresh every 500ms
        return;
    }
    Serial.print("reorienting: ");
    Serial.print(gSensor.getHeading());
    Serial.print(" => ");
    Serial.print(degreesAdjust(gSensor.getHeading()));
    Serial.print(" ");
    timeSoFar = millis();

    int adjustedHeading = degreesAdjust(gSensor.getHeading());
    if (adjustedHeading < -ANGLELIMIT) {
        while (adjustedHeading < -ANGLELIMIT) {
            adjustedHeading = degreesAdjust(gSensor.getHeading());
            turnRight(100);
        }
    } else if (adjustedHeading > ANGLELIMIT) {
        while (adjustedHeading > ANGLELIMIT) {
            adjustedHeading = degreesAdjust(gSensor.getHeading());
            turnLeft(100);
        }
    }
    stopRobot();
//    delay(200);
}

void setLocks(){
  Serial.println("cycle");
    for(int count=0;count<4;count++){
    int got=grayscales[count].readShade();
    Serial.println(got);
    if(got<whiteLimits[count]){
      locks[count]=true;
    }
    else{
      locks[count]=false;
    }
  }
}

void moveToBack(){
  int i = 0;
  moveRobot(0,-motorLimit);
  for(i = 0; i < 17 && grayscales[2].readShade() >whiteLimit; i++){
//    moveRobot(0,-motorLimit);
    delay(100);
  }
  if(i == 17){
    moveRobot(0, 100);
  }
  delay(300);
  stopRobot();
}


void setup() {
    timeSoFar = millis();
    Serial.begin(250000); // set baud rate to 250k
    Wire.begin();
    initMotors();
    startDeg = gSensor.getHeading();
    Serial.println("IR init");
    InfraredSeeker::Initialize();
    Serial.println(InfraredSeeker::Test());
    for (int count = 0; count < 4; count++) {
        locks[count] = false;
    }
    moveToBack();
    Serial.println("done setup");
}

void loop() {
//  moveRobot(0,100);
//  Serial.println(grayscales[1].readShade());
    moveRobot(0,200);
//  setLocks();
//  defend();
//  reorient();
//  Serial.println(gSensor.getHeading());
}
