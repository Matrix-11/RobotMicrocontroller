#pragma once //so that each header is only imported once
#include <Arduino.h>
#include <AccelStepper.h>
#include <jointClass.h>
#include <multiJointClass.h>

class DiffJoint: public MultiJoint {

  using MultiJoint::MultiJoint; // conquer the constructor of MultiJoint

  public:

  void setOffset() { //Function to get offset once to then set in firmware

  float deg = 1;
  float tempOffset = 0;
  homing(false);
  Serial.println("Insert offset in degrees");
  while (deg != 0) {
     
    if (Serial.available() > 0) {
      deg = Serial.parseFloat();
      tempOffset = tempOffset + deg;
      move(joint1.steps2deg(joint1.stepper.currentPosition()) + deg, 0, 0, 0, false);
      
      }
    }   
    Serial.print("Set offset to: ");
    Serial.println(tempOffset);
  }

  void homing(bool returnToPos = false) {


    //joint2.checkHomed(false); //Homes joint2 if necassary
    Serial.print("Now homing Joint ");
    Serial.println(joint1.jointNum);
    Serial.print("and Joint ");
    Serial.println(joint2.jointNum);
    joint1.stepper.setCurrentPosition(0);
    joint2.stepper.setCurrentPosition(0);


    //------------------------Homing of Joint1 (tilt)------------------------//

    if (digitalRead(joint1.EstopPin) == joint1.EstopOnState) { //if the Endstop is already pressed the joint moves away to rehome and get an accurate reading

      move(30, 0, 500 * 2, 0, false);

      delay(500);
    }

    joint1.stepper.setSpeed(-500 * 2);
    joint2.stepper.setSpeed(-500 * 2);

    while (digitalRead(joint1.EstopPin) != joint1.EstopOnState) { //Normal homing routine
      joint1.stepper.runSpeed();
      joint2.stepper.runSpeed();
    }

    delay(500);

    joint1.stepper.setCurrentPosition(0);
    joint2.stepper.setCurrentPosition(0);

    move(joint1.offset, 0, 500 * 2, 0, false); //Move to offset
    joint1.stepper.setCurrentPosition(0); // Set real position at offset

    joint1.isHomed = true;
    Serial.print("Homing of Joint ");
    Serial.print(joint1.jointNum);
    Serial.println(" finished!");
    delay(500);


    //------------------------Homing of Joint2 (rotate)------------------------//

    Serial.print("Now homing Joint ");
    Serial.println(joint2.jointNum);
    joint2.stepper.setCurrentPosition(0);

    if (digitalRead(joint2.EstopPin) == joint2.EstopOnState) { //if the Endstop is already pressed the joint moves away to rehome and get an accurate reading

      move(0, 30, 500, 0, false);

      delay(500);
    }

    joint1.stepper.setSpeed(-500 * 2);
    joint2.stepper.setSpeed(500 * 2);

    while (digitalRead(joint2.EstopPin) != joint2.EstopOnState) { //Normal homing routine
      joint1.stepper.runSpeed();
      joint2.stepper.runSpeed();
    }

    delay(500);

    joint1.stepper.setCurrentPosition(0); // Set real position at offset
    joint2.stepper.setCurrentPosition(0); // Set real position at offset

    move(0, joint2.offset, 500 * 2, 0, false); //Move to offset
    joint2.stepper.setCurrentPosition(0); //Set real position at offset

    joint2.isHomed = true;
    Serial.print("Homing of Joint ");
    Serial.print(joint2.jointNum);
    Serial.println(" finished!");
  }

  void move(float deg1, float deg2 , int speed = 0, int acc = 0, bool safetyCheck = true) { //Moves joint to absolute Positon instantly with given speed. Positive deg are always away from the home positon 

    int pos1 = joint1.deg2steps(deg1);
    int pos2 = joint2.deg2steps(deg2);
    int togo1 = pos1 - joint1.stepper.currentPosition();
    int togo2 = pos2 - joint2.stepper.currentPosition();
    double speed2;

    Serial.println(joint1.steps2deg(pos1));
    Serial.println(joint2.steps2deg(pos2));

    Serial.println(joint1.steps2deg(togo1));
    Serial.println(joint2.steps2deg(togo2));

    if ((joint1.isHomed || joint2.isHomed) == false && safetyCheck == true) {homing();}

    if (safetyCheck) {
      if (joint1.validatePos(pos1) || joint2.validatePos(pos2)) {
        return;
      }
    }


    //joint1.stepper.setCurrentPosition(joint1.stepper.currentPosition() - togo2);
    //joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() + togo1);
    //joint1.stepper.moveTo(pos1 + togo2); // 180 + 0
    //joint2.stepper.moveTo(togo1 - togo2); // -90 + 90

    //joint1.stepper.moveTo(pos1);
    //joint2.stepper.moveTo(pos1);

    //all moves have to be relative!
    joint1.stepper.move(togo1 + togo2); //calculates both positions so they can be reached in one continous move
    joint2.stepper.move(togo1 - togo2); // "first both motors get to pos1 then both to pos2 (for which joint2 in opposite dir)" but calculated in one move 
  
    
    joint1.setSpeed(speed, acc); //Sets target position for stepper1

    //Serial.println(joint2.stepper.currentPosition());
    //joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() + (joint1.stepper.currentPosition() - joint1.stepper.targetPosition()) * movementLink); //joint2 doesnt move in reality, this has to be accounted for in software
    //Serial.println(joint2.stepper.currentPosition());
    //joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() + joint1.stepper.distanceToGo()* movementLink);
    //joint2.stepper.moveTo(pos2 + joint1.stepper.distanceToGo() * movementLink);
    //joint2.stepper.moveTo(pos2);
    //Serial.println(joint1.steps2deg(joint1.stepper.distanceToGo()));
    //Serial.println(joint2.steps2deg(joint2.stepper.distanceToGo()));
    speed2 =  joint2.stepper.distanceToGo()/(double)joint1.stepper.distanceToGo(); //calculates ratio of speed between joint1 and joint2 
    joint2.setSpeed(round(joint1.stepper.speed() * abs(speed2)), acc); //uses ratio to set joint2s speed accordingly

    if (acc == 0) {
      while (joint1.stepper.distanceToGo() != 0 || joint2.stepper.distanceToGo() != 0) {
        joint1.stepper.runSpeedToPosition();
        joint2.stepper.runSpeedToPosition();
      }
    } else {
      while (joint1.stepper.distanceToGo() != 0 || joint2.stepper.distanceToGo() != 0) {
        joint1.stepper.runToPosition();
        joint2.stepper.runToPosition();
      }
    }
    //joint2.stepper.setCurrentPosition(pos2);
    joint1.stepper.stop();
    joint2.stepper.stop();

    joint1.stepper.setCurrentPosition(pos1);
    joint2.stepper.setCurrentPosition(pos2);
}

};
