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

elapsedMillis printTime;
elapsedMillis runTime;

bool track = false;
bool raRun = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Axis_RA.setMaxSpeed(1000);
  Axis_RA.setAcceleration(5);
  //Axis_RA.moveTo(86164);
  Axis_RA.moveTo(1000);

  Axis_DE.setMaxSpeed(1000);
  Axis_DE.setAcceleration(5);
  //Axis_DE.moveTo(86164);
  Axis_DE.moveTo(1000);
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

  /*
  if (runTime >= 60000) {
      Axis_RA.disableOutputs();

      Axis_DE.disableOutputs();
  }*/
 
  if (!track) {
    raRun = Axis_RA.run();
  } /*else {
    raRun = Axis_RA.runSpeed();
    Axis_RA.move(1);
  }*/
  if (!raRun) {
    //Axis_RA.setSpeed(20);//0.905308);
    //track = true;
    //Axis_RA.moveTo(-Axis_RA.currentPosition());
    Axis_RA.disableOutputs();
  }
  if (!Axis_DE.run()) {
    Axis_DE.disableOutputs();
  }
 
  /*if (!Axis_RA.run()) {
    Axis_RA.moveTo(-Axis_RA.currentPosition());
  }
  if (!Axis_DE.run()) {
    Axis_DE.moveTo(-Axis_DE.currentPosition());
  }
  */
  
}