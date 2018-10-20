#include <AFMotor.h>

// motors will be in the order of front-left first, then clockwise from there.
AF_DCMotor frontLeftMotor(1);
AF_DCMotor frontRightMotor(2);
AF_DCMotor backLeftMotor(3);
AF_DCMotor backRightMotor(4);
AF_DCMotor motors[4] = {frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor};

void moveRobot(int xSpeed, int ySpeed){
  float y = ySpeed * sqrt(2);
  float x = xSpeed * sqrt(2);
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
    motors[1].run(BACKWARD);
    motors[3].run(BACKWARD);
  }else{
    motors[1].run(FORWARD);
    motors[3].run(FORWARD);
  }
  motors[0].setSpeed(abs(m0_2));
  motors[1].setSpeed(abs(m1_3));
  motors[2].setSpeed(abs(m0_2));
  motors[3].setSpeed(abs(m1_3));
}

//moves robot in 360 degree direction with heading
void moveRobotHeading(int heading, int str){
  float x = cos(heading * (PI/180)) * str;
  float y = sin(heading * (PI/180)) * str;
  moveRobot(x,y);
}

void stopRobot(){
  moveRobot(0, 0);
}

class Grayscale{
  public:
    Grayscale(){
    
    }
};

class PingSensor{
  public:
    PingSensor(){
      
    }
};

class IRSensor{
  public:
    IRSensor(){
      
    }
};
//  delay(2000);
//  FL.setSpeed(0); 

void testMotors(){
  
//  FL.setSpeed(250);
//  FL.run(FORWARD);
//  FR.setSpeed(250);
//  FR.run(FORWARD);
//  delay(2000);
//  FR.setSpeed(0);
//  BL.setSpeed(250);
//  BL.run(FORWARD);
//  delay(2000);
//  BL.setSpeed(0);
//  BR.setSpeed(250);
//  BR.run(FORWARD);
//  delay(2000);
//  BR.setSpeed(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);           // set baud rate to 250k
  Serial.println("Motor test!");
  
  // turn on motor
//  testMotors(); 

}

void loop() {
  // put your main code here, to run repeatedly:

}
