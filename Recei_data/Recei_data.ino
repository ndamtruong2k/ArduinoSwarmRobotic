/*
* Arduino Wireless Communication Tutorial
*     Example 2 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define button 4
#define ledred 2

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

void setup() {
  Serial.begin(9600);
  pinMode(4, INPUT);
  pinMode(ledred, OUTPUT);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}


void loop() {
    buttonState = digitalRead(button);
    if (buttonState == HIGH){
      digitalWrite(ledred, HIGH);
      } else {
        digitalWrite(ledred,LOW);
        }
    radio.write(&buttonState, sizeof(buttonState));
    delay(5);
}
