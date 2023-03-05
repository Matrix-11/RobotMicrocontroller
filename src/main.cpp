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

// Setup for all Steppermotors
AccelStepper stepper0 = AccelStepper(1, 46, 48); //Ramps Z

AccelStepper stepper1 = AccelStepper(1, 26, 28); //Ramps E0
AccelStepper stepper2 = AccelStepper(1, 36, 34); //Ramps E1

AccelStepper stepper3 = AccelStepper(1, A0, A1); //Ramps X
AccelStepper stepper4 = AccelStepper(1, A6, A7); //Ramps Y
//
Joint joint0(stepper0, 0, NEMAturn, Endstop0, 0, 88, 400, true, maxSpeedturn, false);

Joint joint1(stepper1, 1, NEMAgeared, Endstop1, 1, 7, 180, false, maxSpeedgeared); 
Joint joint2(stepper2, 2, NEMAgeared, Endstop2, 1, 0.75, 180, false, maxSpeedgeared); //Old offset 2

Joint joint3(stepper3, 3, NEMAwrist, Endstop3, 1, 15.5, 180, true, maxSpeedwrist); //Old offset 13
Joint joint4(stepper4, 4, NEMAwrist, Endstop4, 0, 0, 360, false, maxSpeedwrist);

Joint  j[numSteppers] = {joint0, joint1, joint2, joint3, joint4};
Joint (&jointArray)[numSteppers] = j;


MultiJoint mJoint1(jointArray[1], jointArray[2], -1); //Movement of Axis 1 has to be counteracted by Axis 2 
DiffJoint mJoint3(jointArray[3], jointArray[4], 1); //Required by differential gearing for Axis 3 (tilt)

const byte numBytes = 32;
byte receivedBytes[numBytes];
uint16_t receivedInts[7];
byte numReceived = 0;
bool newData = false;
//bool moveSuccess = true;

//uint16_t currentPos[numSteppers];
bool newMove = true;
bool start = false;

void homeAll() {
  mJoint1.homing();
  mJoint3.homing();
  jointArray[2].homing();
  jointArray[0].homing();
  Serial.println("<1>");
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
  pinMode(Endstop0, INPUT_PULLUP);
  pinMode(Endstop1, INPUT_PULLUP);
  pinMode(Endstop2, INPUT_PULLUP);
  pinMode(Endstop3, INPUT_PULLUP);
  pinMode(Endstop4, INPUT_PULLUP);

  pinMode(16, OUTPUT); //Output for Oszi
  pinMode(17, OUTPUT); //Output for Oszi
  pinMode(23, OUTPUT); //Output for Oszi
  pinMode(32, OUTPUT); //Output to switch Valve
  digitalWrite(32, HIGH);
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
  //joint2.setOffset();
  //mJoint3.setOffset();
  homeAll();
}

void recvBytes() { // recieves bytes and converts sets of 2 into ints
    byte ndx = 0;
    byte rb;
    byte waste;
    int j = 0;
    if (Serial.available() >= 14) {
      while (newData == false) {
        rb = Serial.read();
            if (ndx < 14) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
                //Serial.println(rb);
            }
            if (ndx >= 14) {
                waste = Serial.read();
                receivedBytes[ndx] = '\0'; // terminate the string
                numReceived = ndx;  // save the number for use when printing
                //ndx = 0;
                newData = true;
                Serial.println(1);
            }
        //Serial.println(ndx);
      }
    }

  if (newData == true) {
    for (int i = 0; i < 14; i = i + 2) { // converts 2 Bytes into one int
      receivedInts[j] = (receivedBytes[i] << 8) + receivedBytes[i+1];
      Serial.print(receivedInts[j]);
      Serial.print(',');
      j++;
    }
    Serial.println();
    //if (moveSuccess) {
    Serial.println("<1>");
    //}
    //else {
    //  Serial.println("<2>");
    //}
  }
}

void processData() {
  int newPos[5];
  bool matchSpeed = false;

  if (bitRead(receivedBytes[13], 0) == 1) { // if first bit of the int is high
    PORTC = PORTC & B11011111; // Turn on Valve (Pin32 low)
  } else {
    PORTC = PORTC | B00100000; // Turn off Valve (Pin32 high)
  }

  if (bitRead(receivedBytes[13], 1) == 1) { // if second bit of the int is high
    homeAll();
    return;
  }

  if (bitRead(receivedBytes[13], 2) == 1) { // if second bit of the int is high
    matchSpeed = true;
  }

  for (int i = 0; i < 5; i++) {

    //if (receivedInts[i] == currentPos[i]) { // checks if send position is the same as current postition
    //  return;
    //}
    newPos[i] = receivedInts[i];
  }

  move(newPos, receivedInts[5], jointArray, matchSpeed); // moves robot with given position and speed
}

void loop() {
  recvBytes();
  PORTA = PORTA | B00000010; //turn on PIN23
  //PORTA = PORTA & B11110111;
  if (newData == true) {
    //Serial.println("new Data!");
    //PORTA = PORTA | B00000010; //PIN23
    processData();
    newData = false;
    //PORTA = PORTA & B11111101;
  }
  PORTA = PORTA & B11111101; // turn off PIN23
}


void showNewData() {
    if (newData == true) {
        Serial.print("This just in... ");
        for (byte n = 0; n < numReceived/2; n++) {
            Serial.print(receivedInts[n]);
            Serial.print(' ');
        }
        Serial.println();
        newData = false;
    }
}