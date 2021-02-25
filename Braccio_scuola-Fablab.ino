/*
  See https://github.com/cgxeiji/CGx-InverseK for the implementation of the inverse kinematiks calculation

  See https://github.com/francesco-scar/arduino-robotic-arm-braccio for more informations about this project
*/

#define DEMO_MODE 2      // If > 0 the arm will loop between the n default positions 

#include <Braccio.h>
#include <Servo.h>
#include <Wire.h>
#include "nunchuk.h"
#include <InverseK.h>   // Inverse kinematics https://github.com/cgxeiji/CGx-InverseK


Link base_link, upperarm, forearm, hand;


float a0, a1, a2, a3;


int currentCoordinatePosition[3] = {0, 0, 449};
int currentPositionAngles[6] = {90, 135, 0, 0, 90, 0};
int offsetAngle[6] = {0, 5, 0, 8, 0, 0};
int maxAngleAllowed[6] = {180, 180, 180, 180, 180, 90};
int minAngleAllowed[6] = {0, 0, 0, 0, 0, 0};

int savedPositionsAngles[16][6] = { {0, 0, 0, 0, 0, 0},
                                    {90, 90, 90, 90, 90, 90}
                                  };
int nSavedPositions = 0;


bool incrementalMode = true;


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  Braccio.begin();
  Serial.begin(9600);
  Wire.begin();
  nunchuk_init();

  base_link.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(125, b2a(15.0), b2a(165.0));
  forearm.init(125, b2a(0.0), b2a(180.0));
  hand.init(200, b2a(0.0), b2a(180.0));

  // Attach the links to the inverse kinematic model
  InverseK.attach(base_link, upperarm, forearm, hand);

  Serial.println(F("Arduino Braccio robotic arm loaned by local Fablab.\n\nFor software implementation and other details look at https://github.com/francesco-scar/arduino-robotic-arm-braccio\n\nThis project was realized thanks to the help of many open source contributors, thank you.\n\n"));

  delay(4000);

  if (DEMO_MODE){
    nSavedPositions = DEMO_MODE;
    replicateSaved();
  }

  Serial.println("Start 10 readings to get valid controller result");
  for (int i = 0; i < 10; i++) {
    nunchuk_read();
    delay(100);
    Serial.print(i + 1);
    Serial.print("/10  -  ");
    nunchuk_print();
  }
}

void loop() {
  /*
    Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
    M1=base degrees. Allowed values from 0 to 180 degrees
    M2=shoulder degrees. Allowed values from 15 to 165 degrees
    M3=elbow degrees. Allowed values from 0 to 180 degrees
    M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
    M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
    M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed. [I will use 0-90 to be sure]
  */

  nunchuk_read();

  if (nunchuk_buttonZ()) {
    delay(1000);
    nunchuk_read();
    if (nunchuk_buttonC()) {
      replicateSaved();
    } else if (!nunchuk_buttonZ()) {
      saveCurrentPosition();
      delay(1000);
      return;
    } else {
      incrementalMode = !incrementalMode;
      Serial.print("Entering ");
      incrementalMode ? Serial.print("incremental") : Serial.print("cartesian");
      Serial.println(" mode.");
      delay(2000);
      return;
    }
  }

  if (incrementalMode) {
    readIncrementalNunchuk();
  } else {
    readCoordinateNunchuk();
  }

  //(step delay, M1, M2, M3, M4, M5, M6);
  Braccio.ServoMovement(20, currentPositionAngles[0] + offsetAngle[0],  currentPositionAngles[1] + offsetAngle[1], currentPositionAngles[2] + offsetAngle[2], currentPositionAngles[3] + offsetAngle[3], currentPositionAngles[4] + offsetAngle[4],  currentPositionAngles[5] + offsetAngle[5]);

}



void readCoordinateNunchuk () {
  int joystick_X_increment = map(abs(nunchuk_joystickX()) > 20 ? nunchuk_joystickX() : 0, -100, 100, -20, 20);
  int joystick_Y_increment = map(abs(nunchuk_joystickY()) > 20 ? nunchuk_joystickY() : 0, -100, 100, -20, 20);
  int delta_c0 = 0;
  int delta_c1 = 0;
  int delta_c2 = 0;

  if (nunchuk_buttonC()) {
    delta_c2 = joystick_Y_increment;
    currentPositionAngles[5] += joystick_X_increment;
  } else {
    delta_c1 = joystick_Y_increment;
    delta_c0 = joystick_X_increment;
  }

  int pitch_increment = map(abs(nunchuk_pitch()) > 0.35 ? nunchuk_pitch() : 0, -1.0, 1.0, -20, 20);
  int roll_increment = map(abs(nunchuk_roll()) > 0.35 ? nunchuk_roll() : 0, -1.0, 1.0, -20, 20);

  currentPositionAngles[4] -= roll_increment;

  if (delta_c0 || delta_c1 || delta_c2) {
    if (InverseK.solve(currentCoordinatePosition[0] + delta_c0, currentCoordinatePosition[1] + delta_c1, currentCoordinatePosition[2] + delta_c2, a0, a1, a2, a3)) {
      currentPositionAngles[0] = a2b(a0);
      currentPositionAngles[1] = a2b(a1);
      currentPositionAngles[2] = a2b(a2);
      currentPositionAngles[3] = a2b(a3);
      currentCoordinatePosition[0] += delta_c0;
      currentCoordinatePosition[1] += delta_c1;
      currentCoordinatePosition[2] += delta_c2;
    } else {
      Serial.println("Position not reachable");
    }
  }

  limitServoAngle();
  printCurrentPosition();


  Serial.print("  -  ");
  nunchuk_print();

  delay(30);
}


void readIncrementalNunchuk () {
  int joystick_X_increment = map(abs(nunchuk_joystickX()) > 20 ? nunchuk_joystickX() : 0, -100, 100, -3, 3);
  int joystick_Y_increment = map(abs(nunchuk_joystickY()) > 20 ? nunchuk_joystickY() : 0, -100, 100, -3, 3);

  if (nunchuk_buttonC()) {
    currentPositionAngles[3] -= joystick_Y_increment;
    currentPositionAngles[5] += joystick_X_increment;
  } else {
    currentPositionAngles[1] -= joystick_Y_increment;
    currentPositionAngles[0] += joystick_X_increment;
  }

  int pitch_increment = map(abs(nunchuk_pitch()) > 0.35 ? nunchuk_pitch() : 0, -1.0, 1.0, -3, 3);
  int roll_increment = map(abs(nunchuk_roll()) > 0.35 ? nunchuk_roll() : 0, -1.0, 1.0, -3, 3);

  currentPositionAngles[2] -= pitch_increment;
  currentPositionAngles[4] -= roll_increment;

  limitServoAngle();
  printCurrentPosition();


  Serial.print("  -  ");
  nunchuk_print();

  delay(30);
}


void limitServoAngle () {
  for (int i = 0; i < 6; i++) {
    if (currentPositionAngles[i] + offsetAngle[i] > maxAngleAllowed[i]) {
      currentPositionAngles[i] = maxAngleAllowed[i] - offsetAngle[i];
    } else if (currentPositionAngles[i] + offsetAngle[i] < minAngleAllowed[i]) {
      currentPositionAngles[i] = minAngleAllowed[i] - offsetAngle[i];
    }
  }
}


void saveCurrentPosition() {
  Serial.println("Save current position");
  if (nSavedPositions < 16) {
    for (int i = 0; i < 6; i++) {
      savedPositionsAngles[nSavedPositions][i] = currentPositionAngles[i];
    }
    nSavedPositions++;
  }
}


void replicateSaved() {
  Serial.println("Start saved movements");
  delay(2000);
  int currentSaved = -1;
  nunchuk_read();
  while (!(nunchuk_buttonC() && nunchuk_buttonZ()) || DEMO_MODE) {
    currentSaved = (currentSaved + 1) % nSavedPositions;
    Braccio.ServoMovement(20, savedPositionsAngles[currentSaved][0],  savedPositionsAngles[currentSaved][1], savedPositionsAngles[currentSaved][2], savedPositionsAngles[currentSaved][3], savedPositionsAngles[currentSaved][4],  savedPositionsAngles[currentSaved][5]);
    delay(500);
    nunchuk_read();
  }
  Serial.println("Exiting saved movements loop, entering default incremental mode");
  for (int i = 0; i < 6; i++) {
    currentPositionAngles[i] = savedPositionsAngles[currentSaved][i];
  }
  incrementalMode = true;
}

void printCurrentPosition() {
  for (int i = 0; i < 6; i++) {
    Serial.print(currentPositionAngles[i]);
    Serial.print(" ");
  }
  Serial.print("    -    ");
  for (int i = 0; i < 3; i++) {
    Serial.print(currentCoordinatePosition[i]);
    Serial.print(" ");
  }
}







float b2a(float b) {
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}
