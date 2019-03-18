#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

char incomingByte;
String readBuffer = "";
int currentAngle = 0;



void setup() {
  Serial.begin(9600);             // Open serial port to computer
  HC12.begin(9600);               // Open serial port to HC12
}
void loop() {
  readBuffer = "";
  boolean start = false;
  // Reads the incoming angle
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    delay(5);
    // Reads the data between the start "s" and end marker "e"
    if (start == true) {
      if (incomingByte != 'e') {
        readBuffer += char(incomingByte);    // Add each byte to ReadBuffer string variable
      }
      else {
        start = false;
      }
    }
    // Checks whether the received message statrs with the start marker "s"
    else if ( incomingByte == 's') {
      start = true; // If true start reading the message
    }
  }
  // Converts the string into integer
  currentAngle = readBuffer.toInt();
  // Makes sure it uses angles between 0 and 180
  if (currentAngle > 0 && currentAngle < 180) {
    // Convert angle value to steps (depending on the selected step resolution)
    // A cycle = 200 steps, 180deg = 100 steps ; Resolution: Sixteenth step x16
    currentAngle = map(currentAngle, 0, 180, 0, 1600); 
    Serial.println(currentAngle); // Prints the angle on the serial monitor
  }
}
