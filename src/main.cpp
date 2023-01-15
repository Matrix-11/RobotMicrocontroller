
#include <Arduino.h>
#include <jointClass.h>
#include <multiJointClass.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <differentialJoint.h>
#include <moveAll.h>
#include <moveElementClass.h>

#define Endstop4 15
#define Endstop3 14
#define Endstop2 2
#define Endstop1 3
#define Endstop0 18

#define NEMAturn 200 * 6 * 4 / 360 //Steps per degree 
#define NEMAgeared 14.925 * 4 //Steps per degree = 360 / Schrittwinkel / gearratio * microstepping mode
#define NEMAwrist  200 * 7.5 * 8 / 360 // Steps per degree = Standard Steps per Revolution * reduction ratio * microstepping mode / 360

#define maxSpeedturn 6000
#define maxSpeedgeared 10000 * 2//2300//3200 //for 1 joint 2800
#define maxSpeedwrist 6000 * 2
#define defaultSpeed 400

#define numSteppers 5
#define numChars 100
#define moveListLength 30


AccelStepper stepper0 = AccelStepper(1, 46, 48); //Ramps Z

AccelStepper stepper1 = AccelStepper(1, 26, 28); //Ramps E0
AccelStepper stepper2 = AccelStepper(1, 36, 34); //Ramps E1

AccelStepper stepper3 = AccelStepper(1, A0, A1); //Ramps X
AccelStepper stepper4 = AccelStepper(1, A6, A7); //Ramps Y

Joint joint0(stepper0, 0, NEMAturn, Endstop0, 0, 88, 400, true, maxSpeedturn, false);

Joint joint1(stepper1, 1, NEMAgeared, Endstop1, 1, 7, 180, false, maxSpeedgeared); 
Joint joint2(stepper2, 2, NEMAgeared, Endstop2, 1, 2, 180, false, maxSpeedgeared);

Joint joint3(stepper3, 3, NEMAwrist, Endstop3, 1, 13, 180, true, maxSpeedwrist);
Joint joint4(stepper4, 4, NEMAwrist, Endstop4, 0, 0, 360, false, maxSpeedwrist);

Joint  j[numSteppers] = {joint0, joint1, joint2, joint3, joint4};
Joint (&jointArray)[numSteppers] = j;


MultiJoint mJoint1(jointArray[1], jointArray[2], -1); //Movement of Axis 1 has to be counteracted by Axis 2 
DiffJoint mJoint3(jointArray[3], jointArray[4], 1); //Required by differential gearing for Axis 3 (tilt)
//DiffJoint mJoint4(joint4, joint3, -1); //Required by differential gearing for Axis 4 (rotate)

moveElement dummyElement;

char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

moveElement newMove;
int listFlag = 0; // state of moveList like paused, play, stop

bool start = false;
bool newData = false;

float currentPos[numSteppers];

void homeAll() {
  mJoint1.homing();
  mJoint3.homing();
  jointArray[2].homing(); //Becuase of funny pointer shit
  jointArray[0].homing();
}

void setup() {

 
  Serial.begin(250000);
   while (start == false) { // wait for serial port to connect. Else programm wont start
    if (Serial.available()) {
      if (Serial.read() == 'S') {
        start = true;
      }
    }
  } 

  pinMode(16, OUTPUT); //Output for Oszi
  pinMode(17, OUTPUT); //Output for Oszi
  pinMode(23, OUTPUT); //Output for Oszi
  digitalWrite(16, LOW);
  PORTH = PORTH & B11111110; //PIN17
  PORTA = PORTA & B11111101; //PIN23
	//digitalWrite(9, HIGH); //Turn on fan
  stepper1.setEnablePin(A8); //EnablePin 
  stepper1.disableOutputs(); //EnablePin to LOW

  stepper1.setEnablePin(24); //EnablePin 
  stepper1.disableOutputs(); //EnablePin to LOW
  
  stepper2.setEnablePin(30); //EnablePin
  stepper2.disableOutputs(); //EnablePin to LOW

  joint3.stepper.setEnablePin(38); //EnablePin
  joint3.stepper.disableOutputs(); //EnablePin to LOW
  
  joint4.stepper.setEnablePin(A2); //EnablePin
  joint4.stepper.disableOutputs(); //EnablePin to LOW

  //stepper1.setPinsInverted(true, false, false);
  //stepper2.setPinsInverted(true, false, false);
  joint4.stepper.setPinsInverted(true, false, false);

  delay(500);

  Serial.println("Hello");
  //Serial.println(joint1.isHomed);
  //mJoint3.move(90,90);
  //delay(500);
  //move(deg1, 500, jointArray);
  //move(deg2, 500, jointArray);
  //move(deg3, 500, jointArray);
  //move(deg4, 500, jointArray);
  homeAll();
}

void recvWithStartEndMarkers() { //receives from Serial Buffer and puts it in String
    static boolean recvInProgress = false;
    static byte n = 0; //
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rc;
 
  if (Serial.available() > 0) {
    PORTH = PORTH | B00000001; //PIN 17
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[n] = rc;
                n++;
                if (n >= numChars) { //
                    n = numChars - 1;
                }
            }
            else {
                receivedChars[n] = '\0'; // terminate the string
                recvInProgress = false;
                n = 0; //
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    PORTH = PORTH & B11111110;
  }
}

void processData(char input[numChars]) { // gets position out of send message formatted like: J1:54,J2:45,J4:85

  float receivedPos[numSteppers + 1];
  int recievedSpeed = defaultSpeed;
  char * strtokIndx; // this is used by strtok() as an index
  char output[numChars];
  char conversion[numChars];
  int jointnum;
  double checksum = 0;
  bool newPos = false;
  
  strtokIndx = strtok(input, ":"); //Strtok returns pointer

  while (strtokIndx != NULL) {

    strcpy(output, strtokIndx); // Chars are copied into array

    //Serial.print(output[0]);
    //Serial.println(output[1]);

    if (output[0] == 'J') { // J sets joint position
      strtokIndx = strtok(NULL, ",");
      jointnum = (int)(output[1]) - '0'; // Converts the 1 into the int of the ascii code if you then subtract the ascii code of 0 you get the real int
      //jointnum -= 1; // -1 because joint 0 doesnt exist yet

      receivedPos[jointnum] = atof(strtokIndx);
      newPos = true;

      checksum += receivedPos[jointnum];
    }

    if (output[0] == 'S') { // S sets speed of move
      strtokIndx = strtok(NULL, ",");
      recievedSpeed = atoi(strtokIndx);
      //checksum += recievedSpeed;
      newPos = true;
    }

    if (output[0] == 'C') { // C(ommand) for commands :P
      strtokIndx = strtok(NULL, ",");
      Serial.println(strtokIndx);
      strcpy(conversion, strtokIndx);
      //checksum += int(conversion[0]);
      switch (conversion[0]) {

        case 'H': // H(ome) homes all axis
          homeAll();
          break;
      }
    }
    strtokIndx = strtok(NULL, ":");
  } 

  if (newPos == true) {
    Serial.println("Recieved Position: ");
    for (int n = 0; n < numSteppers; n++) {
      Serial.print(receivedPos[n]);
      Serial.print(',');
    }
    Serial.println();
    Serial.print("Recieved Speed: ");
    Serial.println(recievedSpeed);

    newMove.set(receivedPos, recievedSpeed);
    newMove.written = true;
  }

  Serial.print("<C:");
  Serial.print(checksum, 2); // Prints checksum to PC, PC will intervene if something went wrong
  Serial.println(">");
}

void makeMove() {

  if (newMove.written) {
    move(newMove.position, newMove.speed, jointArray);
    newMove.written = false;
  }
}

//void serialEvent(){
// PORTA = PORTA | B00001000;
//}

void loop() {
  recvWithStartEndMarkers();
  PORTA = PORTA | B00000010; //PIN23
  //PORTA = PORTA & B11110111;
  if (newData == true) {
    //Serial.println("new Data!");
    //PORTA = PORTA | B00000010; //PIN23
    processData(receivedChars);
    newData = false;
    //PORTA = PORTA & B11111101;
  }
  makeMove();
  PORTA = PORTA & B11111101;
}