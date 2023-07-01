/* @Brief: GOTO System
 * @Author: Bence Peter (ecneb2000@gmail.com)
 * @Copyright
 */
#include <Arduino.h>
#include <elapsedMillis.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

#define RA_AIn1 5
#define RA_AIn2 4
#define RA_BIn1 6
#define RA_BIn2 7

#define DE_AIn1 9
#define DE_AIn2 8
#define DE_BIn1 10
#define DE_BIn2 11

#define DRIVER_LEN 64
#define AXIS_RA 0
#define AXIS_DE 1

#define GOTO_RATE 800
#define SLEWRATE_RA 80
#define SLEWRATE_DE 80
#define TRACK_RATE 1.810499244

enum Command {
  GOTO = 65,       // "A RA DE"
  TRACK,          // "B"
  PARK,           // "C"
  SETPARKPOS,     // "D"
  GETAXISSTATUS,  // "E AXIS"
  HANDSHAKE,      // "F"
  SETTRACKRATE,   // "G TRACKRATE_RA TRACKRATE_DE"
  ABORT,          // "H"
  SETIDLE,        // "I"
  MOVE,           // "J AXIS"
  STOP,           // "K AXIS"
  SETSLEWRATE,    // "L AXIS RATE"
  ERROR = -1
};

enum MountStatus {
  GOTOING, // KEKW
  SLEWING,
  TRACKING,
  PARKING,
  PARKED,
  IDLE
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor1 = AFMS.getStepper(400, 1);
Adafruit_StepperMotor *myMotor2 = AFMS.getStepper(400, 2);

void forwardstep1() {
  myMotor1->onestep(FORWARD, SINGLE);
}

void backwardstep1() {
  myMotor1->onestep(BACKWARD, SINGLE);
}

void forwardstep2() {
  myMotor2->onestep(FORWARD, SINGLE);
}

void backwardstep2() {
  myMotor2->onestep(BACKWARD, SINGLE);
}

AccelStepper Axis_RA(forwardstep1, backwardstep1);
AccelStepper Axis_DE(forwardstep2, backwardstep2);

MountStatus mountStatus = PARKING;
MountStatus axisStatus[2] {IDLE, IDLE};
AccelStepper Axes[2] {Axis_RA, Axis_DE};

double gotoRate = GOTO_RATE;
double slewRateRA = SLEWRATE_RA;
double slewRateDE = SLEWRATE_DE;
double RATrackRate = TRACK_RATE;
double DETrackRate = 0;

char cmdSeparator = ' ';

char cmd[DRIVER_LEN]{ 0 };
bool cmdComplete = false;
char res[DRIVER_LEN]{ 0 };
String tmpCmd;

long RATarget = 0;
long DETarget = 0;
long RAHome = 0;
long DEHome = 0;

void sendError() {
  res[0] = ERROR;
  res[1] = '#';
  Serial.println(res);
  memset(res, 0, DRIVER_LEN);
}

void sendResponse() {
  Serial.println(res);
  memset(res, 0, DRIVER_LEN);
}

void track() {
  mountStatus = TRACKING;
  Axis_RA.enableOutputs();
  Axis_DE.enableOutputs();
  Axis_RA.setSpeed(RATrackRate);
  Axis_DE.setSpeed(DETrackRate);
}

void park() {
  mountStatus = PARKED;
  Axis_RA.disableOutputs();
  Axis_DE.disableOutputs();
}

void parseNums(String cmdStr, long nums[2], char sep) {
  int sidx = cmdStr.indexOf(sep, 2);
  String n = cmdStr.substring(2, sidx);
  nums[AXIS_RA] = n.toInt();
  n = cmdStr.substring(sidx + 1);
  nums[AXIS_DE] = n.toInt();
}

void parseNums(String cmdStr, float nums[2], char sep) {
  int sidx = cmdStr.indexOf(sep, 2);
  String n = cmdStr.substring(2, sidx);
  nums[AXIS_RA] = n.toFloat();
  n = cmdStr.substring(sidx + 1);
  nums[AXIS_DE] = n.toFloat();
}

bool parseCommand(char* cmd, char* res) {
  String cmdStr = String(cmd);
  String response;
  char cmdChar = cmd[0];
  if (cmd[0] == HANDSHAKE) { // Incoming handshake request, send back handshake cmd.
    res[0] = HANDSHAKE;
    res[1] = '#';
  }
  else if (cmd[0] == GOTO) { // Incoming goto request, enableoutputs, tokenize command, parse target positions, set target positions and set mountStatus
    mountStatus = GOTOING;
    Axis_DE.enableOutputs();
    Axis_RA.enableOutputs();
    long axisPositions[2] = { 0 };
    parseNums(cmdStr, axisPositions, cmdSeparator);
    RATarget = axisPositions[AXIS_RA];
    DETarget = axisPositions[AXIS_DE];
    Axis_RA.setMaxSpeed(gotoRate);
    Axis_DE.setMaxSpeed(gotoRate);
    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);
    Axis_RA.moveTo(axisPositions[AXIS_RA]);
    Axis_DE.moveTo(axisPositions[AXIS_DE]);
    res[0] = GOTO;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == TRACK) { // Start tracking with trackRate
    track();
    res[0] = TRACK;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == PARK) { // Park telescope to home positions
    mountStatus = PARKING;
    Axis_RA.enableOutputs();
    Axis_DE.enableOutputs();
    Axis_RA.setMaxSpeed(gotoRate);
    Axis_DE.setMaxSpeed(gotoRate);
    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);
    Axis_RA.moveTo(RAHome);
    Axis_DE.moveTo(DEHome);
    res[0] = PARK;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == SETTRACKRATE) {
    float axisRates[2] = { 0 };
    String cmdStr = String(cmd);
    char cmdChar = cmd[0];
    parseNums(cmdStr, axisRates, cmdSeparator);
    RATrackRate = axisRates[AXIS_RA];
    DETrackRate = axisRates[AXIS_DE];
    track();
    res[0] = SETTRACKRATE;
    res[1] = '#';    
    return true;
  }
  else if (cmd[0] == GETAXISSTATUS) { // Send back requested axis's position
    int axis;
    char cmdChar;
    String response;
    sscanf(cmd, "%c %d", &cmdChar, &axis);
    switch (axis) {
      case AXIS_RA:
        response = String(char(GETAXISSTATUS)) + String(" ") + String(Axis_RA.currentPosition()) + String(" ") + String(AXIS_RA) + String("#");
        strcpy(res, response.c_str());
        break;
      case AXIS_DE:
        response = String(char(GETAXISSTATUS)) + String(" ") + String(Axis_DE.currentPosition()) + String(" ") + String(AXIS_DE) + String("#");
        strcpy(res, response.c_str());
        break;
    }
    return true;
  }
  else if (cmd[0] == SETPARKPOS) {
    long axisParkpos[2];
    parseNums(cmdStr, axisParkpos, cmdSeparator);
    RAHome = axisParkpos[AXIS_RA];
    DEHome = axisParkpos[AXIS_DE];
    res[0] = SETPARKPOS;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == ABORT) {
    Axis_RA.setMaxSpeed(gotoRate);
    Axis_DE.setMaxSpeed(gotoRate);
    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);
    Axis_RA.stop();
    Axis_DE.stop();
    res[0] = ABORT;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == SETIDLE) {
    Axis_RA.setSpeed(0);
    Axis_DE.setSpeed(0);
    Axis_RA.disableOutputs();
    Axis_DE.disableOutputs();
    res[0] = SETIDLE;
    res[1] = '#';
    mountStatus = IDLE;
    return true;
  }
  else if (cmd[0] == MOVE) {
    Axis_RA.enableOutputs();
    Axis_DE.enableOutputs();
    Axis_RA.setSpeed(slewRateRA);
    Axis_DE.setSpeed(slewRateDE);
    int axis = -1;
    char cmdChar;
    sscanf(cmd, "%c %d", &cmdChar, &axis);
    Serial.println(axis);
    if (axis == AXIS_RA)
      axisStatus[AXIS_RA] = SLEWING;
    if (axis == AXIS_DE)
      axisStatus[AXIS_DE] = SLEWING;
    if (axisStatus[AXIS_RA] == SLEWING || axisStatus[AXIS_DE] == SLEWING)
      mountStatus = SLEWING;
    res[0] = MOVE;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == STOP) {
    int axis = -1;
    char cmdChar;
    sscanf(cmd, "%c %d", &cmdChar, &axis);
    axisStatus[axis] = IDLE;
    Axes[axis].disableOutputs();
    if (axisStatus[0] == IDLE && axisStatus[1] == IDLE) // if both axes are idle, set mount to idle
      mountStatus = IDLE;
    res[0] = STOP;
    res[1] = '#';
    return true;
  }
  else if (cmd[0] == SETSLEWRATE) {
    float axisRate[2] = { 0 };
    String cmdStr = String(cmd);
    char cmdChar = cmd[0];
    parseNums(cmdStr, axisRate, cmdSeparator);
    int axisNum = axisRate[0];
    if (axisNum == AXIS_RA)
      slewRateRA = axisRate[1];
    else if (axisNum == AXIS_DE)
      slewRateDE = axisRate[1];
    res[0] = SETSLEWRATE;
    res[1] = '#';    
    return true;
  }
  sendError();
  return false;
}

void setup() {
  // Create serial connection
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect
  }
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  // Set maximum stepper speed
  Axis_RA.setMaxSpeed(1000);
  Axis_DE.setMaxSpeed(1000);
  Axis_RA.disableOutputs();
  Axis_DE.disableOutputs();
}

void loop() {
  if (cmdComplete) {
    tmpCmd.toCharArray(cmd, DRIVER_LEN);
    parseCommand(cmd, res);
    sendResponse();
    //Serial.print(cmd);
    memset(cmd, 0, DRIVER_LEN);
    tmpCmd = "";
    cmdComplete = false;
  }
  switch (mountStatus) {
    case GOTOING:
      if (Axis_RA.currentPosition() != RATarget) Axis_RA.run(); // Run until target position reached      
      if (Axis_DE.currentPosition() != DETarget) Axis_DE.run(); // Run until target position reached
      if (Axis_RA.currentPosition() == RATarget && Axis_DE.currentPosition() == DETarget) { // If both axes reached their target position, start tracking
        track();
      }
      break;
    case SLEWING:
      if (axisStatus[AXIS_RA] == SLEWING)
        Axis_RA.runSpeed();
      if (axisStatus[AXIS_DE] == SLEWING)
        Axis_DE.runSpeed();
      break;
    case TRACKING:
      Axis_RA.runSpeed(); 
      Axis_DE.runSpeed();
      break;
    case PARKING:
      if (Axis_RA.currentPosition() != RAHome) Axis_RA.run(); // Run until home position reached      
      if (Axis_DE.currentPosition() != DEHome) Axis_DE.run(); // Run until home position reached
      if (Axis_RA.currentPosition() == RAHome && Axis_DE.currentPosition() == DEHome) { // If both axes reached their home position, disable outputs
        park();
      }
      break;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    tmpCmd += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      cmdComplete = true;
    }
  }
}
