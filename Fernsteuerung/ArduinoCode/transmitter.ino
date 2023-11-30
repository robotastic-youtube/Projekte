#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int rightJoystickX = 5;
const int rightJoystickY = 6;
const int leftJoystickX = 4;
const int leftJoystickY = 3;
const int leftJoystickButton = 8;
const int rightJoystickButton = 7;
const int switch1 = 6;
const int switch2 = 5;
const int switch3 = 4;

struct TransmitterData{
  byte lJXval; // left joystick x value
  byte lJYval; // left joystick y value
  byte rJXval; // right joystick x value
  byte rJYval; // right joystick y value
  byte lJbuttonVal; // left joystick button
  byte rJbuttonVal; // right joystick button
  byte switch1Val; // switch 1
  byte switch2Val; // switch 2
  byte switch3Val; // switch 3
};

TransmitterData transmitterData;

RF24 radio(9,10);// CE/CSN
const byte address[6] = "00001";


void setup() {
  // setup radio connection
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  // setup serial connection
  Serial.begin(9600);

  // set initial values
  transmitterData.lJXval = 127;
  transmitterData.lJYval = 127;
  transmitterData.rJXval = 127;
  transmitterData.rJYval = 127;
  transmitterData.lJbuttonVal = 0;
  transmitterData.rJbuttonVal = 0;
  transmitterData.switch1Val = 0;
  transmitterData.switch2Val = 0;
  transmitterData.switch3Val = 0;

  // set digital inputs
  pinMode(leftJoystickButton, INPUT_PULLUP);
  pinMode(rightJoystickButton, INPUT_PULLUP);
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  pinMode(switch3, INPUT_PULLUP);
}

void loop() {

  // read analog pins and map value from 0...1023 to 0...255 (255 max value that can be stored in one byte)
  transmitterData.rJXval = map(analogRead(rightJoystickX), 0, 1023, 0, 255);
  transmitterData.rJYval = map((1023 - analogRead(rightJoystickY)), 0, 1023, 0, 255);
  transmitterData.lJXval = map((1023 - analogRead(leftJoystickX)), 0, 1023, 0, 255);
  transmitterData.lJYval = map(analogRead(leftJoystickY), 0, 1023, 0, 255);

  // read digital pins
  transmitterData.lJbuttonVal = digitalRead(leftJoystickButton);
  transmitterData.rJbuttonVal = digitalRead(rightJoystickButton);
  transmitterData.switch1Val = digitalRead(switch1);
  transmitterData.switch2Val = digitalRead(switch2);
  transmitterData.switch3Val = digitalRead(switch3);

  // output data
  /*Serial.print(transmitterData.rJXval);
  Serial.print(",");
  Serial.print(transmitterData.rJYval);
  Serial.print(",");
  Serial.print(transmitterData.lJXval);
  Serial.print(",");
  Serial.print(transmitterData.lJYval);
  Serial.print(",");
  Serial.print(transmitterData.lJbuttonVal);
  Serial.print(",");
  Serial.print(transmitterData.rJbuttonVal);
  Serial.print(",");
  Serial.print(transmitterData.switch1Val);
  Serial.print(",");
  Serial.print(transmitterData.switch2Val);
  Serial.print(",");
  Serial.println(transmitterData.switch3Val);*/

  radio.write(&transmitterData, sizeof(TransmitterData));
}

