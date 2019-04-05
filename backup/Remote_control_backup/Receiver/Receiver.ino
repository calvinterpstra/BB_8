#include <SoftwareSerial.h>

SoftwareSerial HC12(9, 0); // HC-12 TX Pin, HC-12 RX Pin

#define setPin 2

byte incomingByte;
String readBuffer = "";

void setup() {
  Serial.begin(9600);             // Open serial port to computer
  HC12.begin(9600);               // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);     // HC-12 normal mode
}

unsigned long oldTime,newTime;

void loop() {
  // ==== Storing the incoming data into a String variable
//  newTime = millis();
  //Serial.println(newTime-oldTime);
  //oldTime = newTime;
  
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();
    Serial.print(char(incomingByte));
  }

  while (Serial.available()) {
    HC12.write(Serial.read());
  }
}
