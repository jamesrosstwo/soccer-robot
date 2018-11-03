#include <AFMotor.h>
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
  pin=pin_num;
}
int Grayscale::readShade(){
  return analogRead(pin);
}

//from https://www.arduino.cc/en/Tutorial/Ping
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
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

class IRSensor{
  
  public:
    IRSensor(){
      
    }
};


/* Pin overview:
 * 7-10: Grayscales
 * 11-14: Pings
 */
AF_DCMotor frontLeftMotor(1);
AF_DCMotor frontRightMotor(2);
AF_DCMotor backLeftMotor(3);
AF_DCMotor backRightMotor(4);
AF_DCMotor motors[4] = {frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor};

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
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif
}

void moveRobot(int xSpeed, int ySpeed){
  
  float y = map(ySpeed, 0, 255, 0, 180) * sqrt(2);
  float x = map(xSpeed, 0, 255, 0, 180) * sqrt(2);
  float m0_2 = y + (x/2);
  float m1_3 = y - (x/2);
  if(m1_3 < 0){
    motors[1].run(BACKWARD);
    motors[3].run(BACKWARD);
  }else{
    motors[1].run(FORWARD);
    motors[3].run(FORWARD);
  }
  if(m0_2 < 0){
    motors[0].run(BACKWARD);
    motors[2].run(BACKWARD);
  }else{
    motors[0].run(FORWARD);
    motors[2].run(FORWARD);
  }
  motors[0].setSpeed(abs(m0_2));
  motors[1].setSpeed(abs(m1_3));
  motors[2].setSpeed(abs(m0_2));
  motors[3].setSpeed(abs(m1_3));
}

//moves robot in 360 degree direction with heading. 
void moveRobotHeading(int heading, int str){
  float x = cos((-heading + 90) * (PI/180)) * str; //offset so 0 is the front of the robot, and 
  float y = sin((heading + 90) * (PI/180)) * str;
  moveRobot((int)x,(int)y);
}

void stopRobot(){
  moveRobot(0, 0);
}

void testMotors(){
  moveRobotHeading(0,100);
  delay(300);
  moveRobotHeading(90,100);
  delay(300);
  moveRobotHeading(180, 100);
  delay(300);
  moveRobotHeading(270, 100);
  delay(300);
  moveRobotHeading(45,100);
  delay(300);
  moveRobotHeading(135,100);
  delay(300);
  moveRobotHeading(225, 100);
  
  delay(300);
  moveRobotHeading(315, 100);
  delay(300);
  for(int i = 0; i < 360; i++){
    moveRobotHeading(i, 100);
    delay(30);
  }
}

void setup() {
  Serial.begin(250000);           // set baud rate to 250k
  Serial.println("Motor test!");
  setupAccelGyro();
}

void loop() {
  readAccelGyro();
}
