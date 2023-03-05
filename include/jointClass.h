
#pragma once //so that each header is only imported once
#include <Arduino.h>
#include <AccelStepper.h>

class Joint
{

private:

public:

  AccelStepper stepper;
  int EstopPin;
  int jointNum;
  int maxDeg;
  int defaultSpeed = 40; //defaultSpeed when no other speed is given In degrees per second
  int maxSpeed = 3200;
  float offset;
  float stepsperdeg;//
  bool isHomed;
  bool EstopOnState;
  bool checkBounds;

  Joint(); //Default Constructor

  Joint (AccelStepper s1, int anzAchse, float sperdeg, int EPin, bool EonState, float off_set, int maxDegrees, bool invertDir, int mSpeed, bool checkB = true) {

    stepper = s1; //AccelStepper Object of the stepper
    jointNum = anzAchse; //Number of the Joint 
    //homeDir1 = richtung1; //Which direction the steppers have to turn to get to the home pos
    stepsperdeg = sperdeg; //Number of steps the stepper has to make to turn 1 degree
    EstopPin = EPin; //On which pin the Endstop is connected
    EstopOnState = EonState; //Defines if the Endstop is acivated when 0 or 1
    offset = off_set; //Offset from homing switch
    maxDeg = maxDegrees; //Maximum Degrees the joint can rotate
    isHomed = false; //State of the homed joint
    defaultSpeed = deg2steps(defaultSpeed);
    mSpeed = maxSpeed;
    checkBounds = checkB;

    pinMode(EstopPin, INPUT_PULLUP);

    stepper.setMaxSpeed(maxSpeed);
    stepper.setSpeed(defaultSpeed);
    stepper.setAcceleration(defaultSpeed); //no acc
    stepper.setPinsInverted(invertDir, false, false);
		
  }

  int deg2steps(float deg) { //Converts given degrees into steps for a motor using the given stepsperdeg
  return round(deg * stepsperdeg);
  }

  float steps2deg(int steps) { //Converts given steps into degrees
  return steps / stepsperdeg;
  }

  void homing(bool returnToPos = false) {

    Serial.print("Now homing Joint ");
    Serial.println(jointNum);
    int oldPos = 0;
    stepper.setCurrentPosition(0);
    if (digitalRead(EstopPin) == EstopOnState) { //if the Endstop is already pressed the joint moves away to rehome and get an accurate reading

        stepper.runToNewPosition(deg2steps(15)); //Moves stepper 30 deg away from the homing switch

        delay(500);
    }
    stepper.setSpeed(-500 * 2);
    while (true) { //Normal homing routine
      if (digitalRead(EstopPin) == EstopOnState) { // Checks switch 2 times to counter interferance
        delayMicroseconds(500);
        if (digitalRead(EstopPin) == EstopOnState) {
          break;
        }
      }
      stepper.runSpeed();
      stepper.runSpeed();
    }

    delay(500);
    setSpeed();
    stepper.runToNewPosition(stepper.currentPosition() + deg2steps(offset)); //Move to offset
    oldPos = -1 * stepper.currentPosition(); //Save currently negavtive pos and turn it into a positive
    stepper.setCurrentPosition(0); //Set real start point at offset


    if (returnToPos == true && steps2deg(oldPos) <= maxDeg && oldPos > 0) { //Drive to position before homing if in boundaries
      delay(500);
      stepper.runToNewPosition(oldPos);
      Serial.print("Returned Joint ");
      Serial.print(jointNum);
      Serial.print(" back to now: ");
      Serial.print(steps2deg(oldPos));
      Serial.println("°");
    } else if (returnToPos == true && steps2deg(oldPos) > maxDeg) {
      Serial.print("Position outside boundaries: ");
      Serial.println(steps2deg(oldPos));
    }

    isHomed = true;
    Serial.print("Homing of Joint ");
    Serial.print(jointNum);
    Serial.println(" finished!");
  };

  void setOffset() { //Function to get offset once to then set in firmware

      offset = 0;
      float deg = 1;
      float tempOffset = 0;
      homing(false);
      Serial.println("Insert offset in degrees");
      while (deg != 0) {
        
        if (Serial.available() > 0) {
          deg = Serial.parseFloat();
          tempOffset = tempOffset + deg;
          stepper.runToNewPosition(stepper.currentPosition() + deg2steps(deg));
          
        }
      }   
      Serial.print("Set offset to: ");
      Serial.println(tempOffset);
  }

  void move(float deg, int speed = 0, int acc = 0) { //Moves joint to absolute Positon instantly with given speed. Positive deg are always away from the home positon 
    int pos = deg2steps(deg); //Calculates given degrees to steps once so less rounding errors occur

    checkHomed(); //Check if joint is homed else home it

    if (validatePos(pos)) {return;} // Check if positon is good to move to

    stepper.moveTo(pos);
    setSpeed(speed, acc);


    if (acc == 0) {
      while (stepper.distanceToGo() != 0) {
        stepper.runSpeedToPosition();
      }
    } else {
      while (stepper.distanceToGo() != 0) {
        stepper.runToPosition();
      }
    }
    stepper.stop();
  }

  void moveTo(float deg, int speed = 0, int acc = 0) { //sets next absoulte position where the stepper will move to
    int pos = deg2steps(deg);

    checkHomed();

    if (validatePos(pos)) {return;} // Check if positon is good to move to

    stepper.moveTo(pos);
    setSpeed(speed, acc);
  }

  void checkHomed(bool returnToPos = false) {
    if (isHomed == false) { //Homes automatically and returns to pos
      homing(returnToPos);
    }
  }

  bool validatePos(int pos) { //Checks if given position in steps is in bound(s) else returns 1

    if ((pos > deg2steps(maxDeg) || pos < 0) && checkBounds) {
      Serial.print("Position out of Boundaries: ");
      Serial.print(steps2deg(pos));
      Serial.print("° (");
      Serial.print(pos);
      Serial.println(")");
      Serial.print("Max allowed degrees: ");
      Serial.println(deg2steps(maxDeg));
      return 1; //Return 1 when out of bounds
    }
    return 0;
  }

  void setSpeed(int speed = 0, int acc = 0) { //Sets given steps/sec as speed if 0 default will be used

    stepper.setMaxSpeed(maxSpeed);

    if (acc == 0 && speed == 0) {
      stepper.setSpeed(defaultSpeed);
    } else if (acc == 0 && speed != 0) {
      stepper.setSpeed(speed);
    } else if (acc != 0 && speed == 0) {
      stepper.setAcceleration(acc);
      stepper.setMaxSpeed(defaultSpeed);
    } else if (acc != 0 && speed != 0) {
      stepper.setMaxSpeed(speed);
      stepper.setSpeed(speed);
      stepper.setAcceleration(acc);
    }
  }

};