
#pragma once //so that each header is only imported once
#include <Arduino.h>
#include <AccelStepper.h>
#include <jointClass.h>


class MultiJoint //Class takes 2 mechanically connected Joints and drives then together
{

public:
  float movementLink; //How the j2 should turn when j1 turns (ratio [-1,1]) e.g. -1 same length but different direction 
  Joint& joint1; //"Main" joint joint2 has to move with joint1 to not crash
  Joint& joint2; //"Assisting" joint this joint can move independent

  MultiJoint(Joint& j1, Joint& j2, float link):movementLink(link), joint1(j1), joint2(j2) {} // Joint& passes the object by reference meaning changes to the object WILL applied to the passed object 
    //joint1 = j1;
    //joint2 = j2; 
   // movementLink = link; // How the j2 should turn when j1 turns (ratio [-1,1]) e.g. -1 same length but different direction 
		
  void homing(bool returnToPos = false) {
    
    joint1.isHomed = false;
    //joint2.checkHomed(false); //Homes joint2 if necassary

    Serial.print("Now homing Joint ");
    Serial.println(joint1.jointNum);
    joint1.stepper.setCurrentPosition(0);
    joint2.stepper.setCurrentPosition(0);

    if (digitalRead(joint1.EstopPin) == joint1.EstopOnState) { //if the Endstop is already pressed the joint moves away to rehome and get an accurate reading

        move(15, 0, 500 * 2, 0, false);

        delay(500);
    }

    joint1.stepper.setSpeed(-500 * 2);
    joint2.stepper.setSpeed(-500 * 2 * movementLink);

    while (true) { //Normal homing routine
      if (digitalRead(joint1.EstopPin) == joint1.EstopOnState) { // Checks switch 2 times to counter interferance
        delayMicroseconds(500);
        if (digitalRead(joint1.EstopPin) == joint1.EstopOnState) {
          break;
        }
      }
      joint1.stepper.runSpeed();
      joint2.stepper.runSpeed();
    }
    joint2.stepper.setCurrentPosition(0);
    joint1.stepper.setCurrentPosition(0);
    delay(500);

    //joint1.setSpeed(500);
    //joint2.setSpeed(500);
    //oldPos = -1 * (joint1.stepper.currentPosition() - joint1.deg2steps(joint1.offset)); //Save currently negavtive pos and turn it into a positive

    //joint1.stepper.runToNewPosition(joint1.stepper.currentPosition() + joint1.deg2steps(joint1.offset)); //Move to offset

    //joint2.stepper.runToNewPosition(joint2.stepper.currentPosition() - joint2.deg2steps(joint1.offset));

    move(joint1.offset, 0, 500 * 2, 0, false); //Move to offset
    joint2.stepper.setCurrentPosition(0);
    joint1.stepper.setCurrentPosition(0);
    //oldPos = -1 * joint1.stepper.currentPosition(); //Save currently negavtive pos and turn it into a positive
    //joint1.stepper.setCurrentPosition(0); //Set real start point at offset
    //move(joint1.offset, 0);

    delay(500);

/*     if (returnToPos == true && joint1.steps2deg(oldPos) <= joint1.maxDeg) { //Drive to position before homing if in boundaries
      
      move(joint1.steps2deg(oldPos));

      Serial.print("Returned Joint ");
      Serial.print(joint1.jointNum);
      Serial.print(" back to now: ");
      Serial.print(joint1.steps2deg(oldPos));
      Serial.println("°");
    } else if (returnToPos == true && joint1.steps2deg(oldPos) > joint1.maxDeg) {
      Serial.print("Position outside boundaries: ");
      Serial.println(joint1.steps2deg(oldPos));
    } */

    joint1.isHomed = true;
    Serial.print("Homing of Joint ");
    Serial.print(joint1.jointNum);
    Serial.println(" finished!");

    joint1.setSpeed();
    joint2.setSpeed();

    delay(500);

    //joint2.homing();
  };

  void move(float deg1, float deg2 , int speed = 0, int acc = 0, bool safetyCheck = true) { //Moves joint to absolute Positon instantly with given speed. Positive deg are always away from the home positon 

    int pos1 = joint1.deg2steps(deg1);
    int pos2 = joint2.deg2steps(deg2);
    double speed2;

    if (joint1.isHomed == false && safetyCheck == true) {homing();}

    if (joint2.isHomed == false && safetyCheck == true) {joint2.homing();}

    if (safetyCheck) {
      if (joint1.validatePos(pos1) || joint2.validatePos(pos2)) {
        return;
      }
    }
    //if ((joint1.validatePos(pos1) || joint2.validatePos(pos2)) && safetyCheck == true) {return;}


    joint1.stepper.moveTo(pos1);
    joint1.setSpeed(speed, acc); //Sets target position for stepper1


    //Serial.println(joint2.stepper.currentPosition());
    //joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() + (joint1.stepper.currentPosition() - joint1.stepper.targetPosition()) * movementLink); //joint2 doesnt move in reality, this has to be accounted for in software
    //Serial.println(joint2.stepper.currentPosition());
    
    joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() - joint1.stepper.distanceToGo()* movementLink);
    //joint2.stepper.moveTo(pos2 + joint1.stepper.distanceToGo() * movementLink);
    joint2.stepper.moveTo(pos2);
    Serial.println(joint2.steps2deg(joint2.stepper.distanceToGo()));
    Serial.println(joint2.steps2deg(joint2.stepper.targetPosition()));
    speed2 =  joint2.stepper.distanceToGo()/(double)joint1.stepper.distanceToGo(); //calculates ratio of speed between joint1 and joint2 
    //if (speed2 > joint2.maxSpeed) {
    //  joint2.setSpeed(joint2.maxSpeed);
    //} else {
      joint2.setSpeed(round(joint1.stepper.speed() * abs(speed2)), acc); //uses ratio to set joint2s speed accordingly
    //}
    Serial.println(joint2.stepper.speed());


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
  }

  void moveTo(float deg1, float deg2 = 0.0, int speed = 0, int acc = 0) { //sets next absoulte position where the stepper will move to later

    int pos1 = joint1.deg2steps(deg1);
    int pos2 = joint2.deg2steps(deg2);
    double speed2;

    if (joint1.validatePos(pos1) || joint2.validatePos(pos2)) {return;}


    joint1.stepper.moveTo(joint1.deg2steps(deg1));
    joint1.setSpeed(speed, acc); //Sets target position for stepper1


    //Serial.println(joint2.stepper.currentPosition());
    joint2.stepper.setCurrentPosition(joint2.stepper.currentPosition() + (joint1.stepper.currentPosition() - joint1.stepper.targetPosition()) * movementLink); //joint2 doesnt move in reality, this has to be accounted for in software
    //Serial.println(joint2.stepper.currentPosition());
    joint2.stepper.moveTo(joint2.deg2steps(deg2));
    speed2 =  joint2.stepper.distanceToGo()/(double)joint1.stepper.distanceToGo(); //calculates ratio of speed between joint1 and joint2 
    joint2.setSpeed(round(joint1.stepper.speed() * abs(speed2)), acc); //uses ratio to set joint2s speed accordingly
  }
};