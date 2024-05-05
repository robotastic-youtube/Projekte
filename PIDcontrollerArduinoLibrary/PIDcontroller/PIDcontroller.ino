#include "PIDcontroller.h"

#define SYS_OUTPUT_PIN A0
#define SYS_INPUT_PIN DD3

int switchPin = 2;

PIDcontroller pid;
float setPoint1 = 100;
float setPoint2 = 0;

int dtPlot = 200;
int lastPlotTime = 0;

void setup() {
  
  Serial.begin(9600);
  analogWrite(SYS_INPUT_PIN, 0);
  pinMode(switchPin, INPUT_PULLUP);

  pid.setSampleTime(100);
  pid.setPIDgain(1.5, 5, 0.001);
  pid.setSetpoint(0.0);
  pid.setMaxOutput(255);
  pid.setMinOutput(0);
  
  pid.setSetpoint(setPoint1);
}

void loop() {
  unsigned long curTime = millis();

  if ((digitalRead(switchPin) == 0) && (pid.getSetpoint() != setPoint1)){

    pid.setSetpoint(setPoint1);
    // pid.turnOff();

  }
  else if((digitalRead(switchPin) == 1) && (pid.getSetpoint() != setPoint2)){

    pid.setSetpoint(setPoint2);
    // pid.turnOn(map(analogRead(SYS_OUTPUT_PIN), 0, 1024, 0, 255));

  }

  float output = map(analogRead(SYS_OUTPUT_PIN), 0, 1024, 0, 255);
  output = pid.smoothSignal(output, 5);
  float input = pid.runPID(output);
  analogWrite(SYS_INPUT_PIN, input);

  if ((curTime - lastPlotTime) > dtPlot)
  {
    Serial.print(0);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.println(pid.getSetpoint());

    lastPlotTime = curTime;
  }

}