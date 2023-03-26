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
#define AXIS_DEC 1

enum Command {
  GOTO = 'A',         // command structure "A:RA:DE:SLEWSPEED"
  TRACK = 'B',
  PARK = 'C',
  SETPARKPOS = 'D',
  GETAXISSTATUS = 'E',
  HANDSHAKE = 'F'
};

enum MountStatus {
  SLEWING, TRACKING, PARKING
};

AccelStepper Axis_RA(AccelStepper::FULL4WIRE, RA_AIn1, RA_AIn2, RA_BIn1, RA_BIn2);
AccelStepper Axis_DE(AccelStepper::FULL4WIRE, DE_AIn1, DE_AIn2, DE_BIn1, DE_BIn2);

MountStatus mountStatus = PARKING;

bool slewing = mountStatus == PARKING;
bool tracking = mountStatus == TRACKING;

char cmd[DRIVER_LEN] {0};
bool cmdComplete = false;
char res[DRIVER_LEN] {0};

void sendError() {
  res[0] = 2;
  res[1] = '\0';  
  Serial.write(res);
  res[0] = '\0';
}

void parseCommand(char* cmd, char* res) {
  if (strstr(cmd, char(GOTO)) != NULL) {
    mountStatus = SLEWING;
    long TargetRASteps;
    long TargetDESteps;
    float SlewSpeed;
    sscanf(cmd, "A:%l:%l:%f", TargetRASteps, TargetDESteps, SlewSpeed);
    Axis_RA.setMaxSpeed(SlewSpeed);
    Axis_DE.setMaxSpeed(SlewSpeed);
    Axis_RA.setAcceleration(5);
    Axis_DE.setAcceleration(5);
    Axis_RA.moveTo(TargetRASteps);
    Axis_DE.moveTo(TargetDESteps);
    res[0] = 1;
    res[1] = '\0';
    Serial.write(res);
    res[0] = '\0';
  }
  if (strstr(cmd, char(HANDSHAKE)) != NULL) {
    res[0] = 1;
    res[1] = '\0';
    Serial.write(res);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Axis_RA.setMaxSpeed(1000);
  Axis_DE.setMaxSpeed(1000);
  Axis_RA.disableOutputs();
  Axis_DE.disableOutputs();
}

void loop() {
  if ( Serial.available() > 0 ) {
    if (mountStatus == SLEWING) {
      slewing = Axis_RA.run() && Axis_DE.run();
      if (!slewing) mountStatus = TRACKING;
    } else if (mountStatus == TRACKING && !slewing) {
      tracking = Axis_RA.runSpeed() && Axis_DE.runSpeed();
    } else {
      Axis_RA.disableOutputs();
      Axis_DE.disableOutputs();
    }
  }
}

void SerialEvent() {
  while (Serial.available()) {
    size_t cmdLen = Serial.readBytesUntil('\0', cmd, DRIVER_LEN);
    if (cmdLen <= 0) cmd[0] = '\0';
    parseCommand(cmd, res);
  }
}
