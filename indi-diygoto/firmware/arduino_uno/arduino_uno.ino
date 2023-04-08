/* @Brief: GOTO System
 * @Author: Bence Peter (ecneb2000@gmail.com)
 * @Copyright
 */
#include <elapsedMillis.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

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

#define GOTO_RATE 500
#define TRACK_RATE 0.9

enum Command {
  GOTO = 'A',           // "A RA DE"
  TRACK = 'B',          // "B TRACKRATE"
  PARK = 'C',           // "C"
  SETPARKPOS = 'D',     // "D"
  GETAXISSTATUS = 'E',  // "E AXIS_NUM"
  HANDSHAKE = 'F',      // "F"
  SETTRACKRATE = 'G',   // "G TRACKRATE"
  ERROR = 'X'
};

enum MountStatus {
  SLEWING,
  TRACKING,
  PARKING,
  IDLE
};

AccelStepper Axis_RA(AccelStepper::FULL4WIRE, RA_AIn1, RA_AIn2, RA_BIn1, RA_BIn2);
AccelStepper Axis_DE(AccelStepper::FULL4WIRE, DE_AIn1, DE_AIn2, DE_BIn1, DE_BIn2);

MountStatus mountStatus = PARKING;

bool slewing = mountStatus == SLEWING;
bool tracking = mountStatus == TRACKING;

double slewRate = GOTO_RATE;
double trackRate = TRACK_RATE;

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
  res[0] = 'X';
  res[1] = '\0';
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
  Axis_RA.setSpeed(trackRate);
  Axis_DE.setSpeed(0);
}

void park() {
  mountStatus = PARKING;
  Axis_RA.disableOutputs();
  Axis_DE.disableOutputs();
}

void parseCommand(char* cmd, char* res) {
  /* @brief Parse incoming command and send response to client.
   * Implemented commands: HANDSHAKE, GOTO, TRACK, GETAXISSTATUS, PARK
   * @arguments
   *  cmd (char*): Command cstring, that is being parsed.
   *  res (char*): Response cstring, that will be sent back if everything is ok.
   */
  char* pch; // helper pointer for command tokenisation
  char sep = ' '; // command separator character
  if (strcmp(cmd[0], char(HANDSHAKE)) == 0) { // Incoming handshake request, send back handshake cmd.
    res[0] = HANDSHAKE;
    res[1] = '\0';
  }
  if (strcmp(cmd[0], char(GOTO)) == 0) { // Incoming goto request, enableoutputs, tokenize command, parse target positions, set target positions and set mountStatus
    mountStatus = SLEWING;
    Axis_DE.enableOutputs();
    Axis_RA.enableOutputs();

    long axisPositions[2] = { 0 };
    String cmdStr = String(cmd);
    char cmdChar = cmd[0];

    int sidx = cmdStr.indexOf(cmdSeparator, 2);
    String n = cmdStr.substring(2, sidx);
    axisPositions[AXIS_RA] = n.toInt();
    n = cmdStr.substring(sidx + 1);
    axisPositions[AXIS_DE] = n.toInt();

    RATarget = axisPositions[AXIS_RA];
    DETarget = axisPositions[AXIS_DE];

    Axis_RA.setMaxSpeed(slewRate);
    Axis_DE.setMaxSpeed(slewRate);

    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);

    Axis_RA.moveTo(axisPositions[AXIS_RA]);
    Axis_DE.moveTo(axisPositions[AXIS_DE]);

    res[0] = GOTO;
    res[1] = '\0';
  }
  if (strcmp(cmd[0], char(TRACK)) == 0) { // Start tracking with trackRate
    track();
  }
  if (strcmp(cmd[0], char(GETAXISSTATUS)) == 0) { // Send back requested axis's position
    int axis;
    char cmdChar;
    String response;
    sscanf(cmd, "%c %d", &cmdChar, &axis);
    switch (axis) {
      case AXIS_RA:
        response = String(GETAXISSTATUS) + String(" ") + String(Axis_RA.currentPosition()) + String(" ") + String(AXIS_RA);
        strcpy(res, response.c_str());
        break;
      case AXIS_DE:
        response = String(GETAXISSTATUS) + String(" ") + String(Axis_DE.currentPosition()) + String(" ") + String(AXIS_DE);
        strcpy(res, response.c_str());
        break;
    }
  }
  if (strcmp(cmd[0], char(PARK)) == 0) { // Park telescope to home positions
    mountStatus = PARKING;
    Axis_RA.setMaxSpeed(slewRate);
    Axis_RA.setMaxSpeed(slewRate);
    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);
    Axis_RA.moveTo(RAHome);
    Axis_DE.moveTo(DEHome);
    res[0] = PARK;
    res[1] = '\0';
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect
  }
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
    case SLEWING:
      if (Axis_RA.currentPosition() != RATarget) Axis_RA.run(); // Run until target position reached      
      if (Axis_DE.currentPosition() != DETarget) Axis_DE.run(); // Run until target position reached
      if (Axis_RA.currentPosition() == RATarget && Axis_DE.currentPosition() == DETarget) { // If both axes reached their target position, start tracking
        track();
      }
      break;
    case TRACKING:
      tracking = Axis_RA.runSpeed() && Axis_DE.runSpeed();
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
    if (inChar == '\n') {
      cmdComplete = true;
    }
  }
}
