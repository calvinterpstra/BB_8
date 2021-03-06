#include <SoftwareSerial.h>

#define setPin 6 //digital
#define X_Pin  0 //analog
#define Y_Pin  1 //analog
#define Sw_Pin 2 //digital

SoftwareSerial HC12(10, 11);         // HC-12 TX Pin, HC-12 RX Pin

byte incomingByte;
String readBuffer = "";
int x = 0;
int y = 0;

void setup() {
  Serial.begin(9600);                   // Open serial port to computer
  HC12.begin(9600);                     // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);           // HC-12 normal, transparent mode
}

void loop() {
  x = analogRead(X_Pin);
  y = analogRead(Y_Pin);
//  Serial.println(x);

  // Storing the incoming data into a String variable
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    readBuffer += char(incomingByte);    // Add each byte to ReadBuffer string variable
    Serial.print(readBuffer);
  }
  readBuffer = "";
  delay(100);
  // Sending data from one HC-12 to another via the Serial Monitor
  while (Serial.available()) {
    HC12.write(Serial.read());
  }
  
    if (x <= 700 && x >= 300 && y <= 700 && y >= 300) {
      delay(200);
      ///Serial.println("S");
      HC12.print("S");
    }
    if (x <= 300 or x >= 700) {
      delay(200); 
      //Serial.println("F");
      HC12.print("F");
    }
    if (y <= 300) {
      delay(200);
      //Serial.println("R");
      HC12.print("R");
    }
    if (y >= 700) {
      delay(200);
      //Serial.println("L");
      HC12.print("L");
    }
}
