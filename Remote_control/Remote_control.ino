#include <SoftwareSerial.h>

#define setPin 6

SoftwareSerial HC12(10, 11);         // HC-12 TX Pin, HC-12 RX Pin

byte incomingByte;
String readBuffer = "";

void setup() {
  Serial.begin(9600);                   // Open serial port to computer
  HC12.begin(9600);                     // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);           // HC-12 normal, transparent mode
}

void loop() {
  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    readBuffer += char(incomingByte);    // Add each byte to ReadBuffer string variable
  }
  
  delay(100);
  // ==== Sending data from one HC-12 to another via the Serial Monitor
  while (Serial.available()) {
    HC12.write(Serial.read());
  }
}
