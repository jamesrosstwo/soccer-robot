#include <AFMotor.h>
#include <Arduino.h>
int grayScale=7;


class Motor{
  private:
  public:
    int pin;
    Motor(int pin_num){
    }

    void forwards(){
      
    }
};

class Grayscale{
  private:
  int result;
  int pin;
 
  public:
    Grayscale(int pin_num);  
    int readShade();
};

Grayscale::Grayscale(int pin_num){
  Serial.println("haha");
  Serial.println(pin_num);
  pin=pin_num;
}
int Grayscale::readShade(){

  
  
  Serial.println(pin);
  return analogRead(pin);
}


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

class Pixy{
  public:
  Pixy();
};


//get all motors setup
AF_DCMotor FL(1);
AF_DCMotor FR(2);
AF_DCMotor BL(3);
AF_DCMotor BR(4);



void testMotors(){
  FL.setSpeed(250);
  FL.run(FORWARD);
  delay(2000);
  FL.setSpeed(0); 
  FR.setSpeed(250);
  FR.run(FORWARD);
  delay(2000);
  FR.setSpeed(0);
  BL.setSpeed(250);
  BL.run(FORWARD);
  delay(2000);
  BL.setSpeed(0);
  BR.setSpeed(250);
  BR.run(FORWARD);
  delay(2000);
  BR.setSpeed(0);
}

Grayscale test(grayScale);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  
  // turn on motor
   
 
}

void loop() {
//   put your main code here, to run repeatedly:
 int val=analogRead(7);
 Serial.println("main");
 Serial.println(val);
 Serial.println("obj");
 Serial.println(test.readShade());
 delay(1000);
}


