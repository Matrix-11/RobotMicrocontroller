#pragma once //so that each header is only imported once

#include <Arduino.h>
#include <jointClass.h>
#include <multiJointClass.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <differentialJoint.h>

#define numSteppers 5

float moveL1 = -1;
float moveL2 = 1;
float moveL3 = -1;

// void setJoints(Joint j1, Joint j2, Joint j3, Joint j4) {

//   jointArray[0] = j1;
//   jointArray[1] = j2;
//   jointArray[2] = j3;
//   jointArray[3] = j4;
// }

void move(float deg[numSteppers], int maxSpeed, Joint (&jointArray)[numSteppers], bool matchSpeed = false) {

  long pos[numSteppers];
  long togo[numSteppers];
  int speed[numSteppers];
  int oldTogoj3 = 0; // pure amount to go for calc of rotaton move for j3 and j4
  double maxWay = 0; // The maximum way a stepper has to rotate
  double maxTime = 0; // The maximum time a stepper has to rotate

  for (int i = 0; i < numSteppers; i++) {

    if (jointArray[i].isHomed == false) { //checks homing status
      Serial.print("Not all axis homed! ");
      Serial.print("Joint:");
      Serial.println(jointArray[i].jointNum);
      return;
    }
    //Serial.println(jointArray[i].steps2deg(jointArray[i].stepper.currentPosition()));
    pos[i] = jointArray[i].deg2steps(deg[i]); // converts degrees into steps for every stepper

    if (jointArray[i].validatePos(pos[i])) { // checks if position is in bounds
      Serial.println("Movement out of bounds");
      return;
    }
    togo[i] = pos[i] - jointArray[i].stepper.currentPosition();
  }

  for (int i = 0; i < numSteppers; i++) {
    if (jointArray[i].jointNum == 2) {
      togo[i] = togo[i] + togo[i-1] * moveL1; //accounts for mechanical linkage between j1 and j2
      //Serial.println("Applying offset to j2");
    }

    if (jointArray[i].jointNum == 3) {
      oldTogoj3 = togo[i];
      togo[i] = togo[i] + togo[i+1] * moveL2; //accounts for mechanical linkage between j3 and j4 (tilt)
      //Serial.println("Applying offset to j3");
    }

    if (jointArray[i].jointNum == 4) {
      togo[i] = oldTogoj3 + togo[i] * moveL3; //accounts for mechanical linkage between j3 and j4 (rotate)
      //Serial.println("Applying offset to j4");
    }
  }

  if (matchSpeed == false) {
    for (int i = 0; i < numSteppers; i++){
      jointArray[i].stepper.move(togo[i]); //applies positon and speed to every stepper
      jointArray[i].stepper.setSpeed(maxSpeed);
    }

  } else {
  
    for (int i = 0; i < numSteppers; i++) {

      if (abs(togo[i]) > maxWay) {
        maxWay = abs(togo[i]);
        //Serial.print("Found stepper with longest way: Joint ");
        //Serial.println(jointArray[i].jointNum);
        //Serial.println(maxWay);
      }

    }

    maxTime = maxWay / maxSpeed; //calculates the time every stepper has in order to get to his position
    //Serial.print("MaxTime is: ");
    //Serial.println(maxTime);
    for (int i = 0; i < numSteppers; i++) {

      speed[i] = round(abs(togo[i]) / maxTime); //calculates the speed for every stepper so they all reach their pos at the same time
      //Serial.print("Speed for stepper ");
      //Serial.println(jointArray[i].jointNum);
      //Serial.println(maxSpeed);
      jointArray[i].stepper.move(togo[i]); //applies positon and speed to every stepper
      jointArray[i].stepper.setSpeed(speed[i]);
      //Serial.print("Setting target positon for Stepper ");
      //Serial.println(jointArray[i].jointNum);
      //Serial.println(jointArray[i].steps2deg(togo[i]));
    }
  }

  //digitalWrite(16, HIGH);
  PORTH = PORTH | B00000010;
  while (jointArray[0].stepper.distanceToGo() != 0 || jointArray[1].stepper.distanceToGo() != 0 || jointArray[2].stepper.distanceToGo() != 0 || jointArray[3].stepper.distanceToGo() != 0 || jointArray[4].stepper.distanceToGo() != 0) {

    jointArray[0].stepper.runSpeedToPosition();
    jointArray[1].stepper.runSpeedToPosition();
    jointArray[2].stepper.runSpeedToPosition();
    jointArray[3].stepper.runSpeedToPosition();
    jointArray[4].stepper.runSpeedToPosition();
  }

  for (int i = 0; i < numSteppers; i++) {
    jointArray[i].stepper.stop();
    jointArray[i].stepper.setCurrentPosition(pos[i]);
  }
  //digitalWrite(16,LOW);
  PORTH = PORTH & B11111101;

  Serial.println("Finished move");
}