
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); //CE, CSN
const byte address[6] = "00001";

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

void setup() {
  // put your setup code here, to run once:

  while(!Serial);
    Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //radio.setPayloadSize(32);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (radio.available()){
    //Serial.println("DATA AVAILABLE");
    radio.read(&transmitterData, sizeof(TransmitterData));
    Serial.print(transmitterData.rJXval);
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
    Serial.println(transmitterData.switch3Val);
    
  }

}