#include "microphoneListener.h"
#include "transmitterHC.h"



void setup() {
  Serial.begin(9600);                   // Open serial port to computer
  setupTransmitter();
  setupMicrophone();
}

void loop() {
  transmitJoystick(); 
  
  if (listenMicrophone())  {
    Serial.write("Signaal ontvangen,klaar met luisteren");
  }
}


