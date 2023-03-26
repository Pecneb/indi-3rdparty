#include <elapsedMillis.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

const int AIn1 = 5;
const int AIn2 = 4;
const int BIn1 = 6;
const int BIn2 = 7;

const int AIn1_2 = 9;
const int AIn2_2 = 8;
const int BIn1_2 = 10;
const int BIn2_2 = 11;


AccelStepper Axis_RA(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2);
AccelStepper Axis_DE(AccelStepper::FULL4WIRE, AIn1_2, AIn2_2, BIn1_2, BIn2_2);

MultiStepper Mount();

elapsedMillis printTime;
elapsedMillis runTime;

void setup() {
  Serial.begin(9600);

  Mount.addStepper(Axis_RA);
  Mount.addStepper(Axis_DE);

  long initialTargets[] = {10000, 10000};
  Mount.moveTo(initialTargets);
}

void loop() {
  if (printTime >= 1000) {
    printTime = 0;
 
    float mSpeed = Axis_RA.speed();
    float mSpeed_2 = Axis_DE.speed();
 
    String raStat = String("RA Speed: ") + String(mSpeed);
    String deStat = String("DE Speed: ") + String(mSpeed_2);
 
    Serial.print(raStat);
    Serial.print("   ");
    Serial.println(Axis_RA.currentPosition());
 
    Serial.print(deStat);
    Serial.print("   ");
    Serial.println(Axis_DE.currentPosition());
 
    Serial.println();
  }

  if (!Mount.run()) {
    long nextTargets[] = {-Axis_RA.currentPosition(), -Axis_DE.currentPosition()};
    Mount.moveTo(nextTargets);
  }

  if (runTime >= 60000) {
    long endTargets[] = {0,0};
    Mount.moveTo(endTargets);

    Axis_RA.disableOutputs();
    Axis_RA.disableOutputs();
  }
}