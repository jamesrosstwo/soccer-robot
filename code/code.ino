#include <AFMotor.h>



class Motor{
  private:
  public:
    int pin;
    Motor(int pin_num){
      pin = pin_num;
      AF_DCMotor motor(pin);
    }

    void forwards(){
      
    }
};

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


//get all motors setup
AF_DCMotor FL(1);
AF_DCMotor FR(2);
AF_DCMotor BL(3);
AF_DCMotor BR(4);



void testMotors(){
//  FL.setSpeed(250);
//  FL.run(FORWARD);
//  delay(2000);
//  FL.setSpeed(0); 
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
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  // turn on motor
  testMotors(); 

}

void loop() {
  // put your main code here, to run repeatedly:

}


