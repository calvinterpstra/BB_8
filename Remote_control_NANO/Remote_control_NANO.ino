#include <SoftwareSerial.h>
#include "microphoneListener.h"
#include <SoftwareSerial.h>

#define setPin 6 //digital
#define X_Pin  0 //analog
#define Y_Pin  1 //analog
#define Sw_Pin 2 //digital
#define redPin 3 //digital 
#define greenPin 5 //digital
#define bluePin 9 //digital

#define OFF         'o'
#define BALANCE     'b'
#define FORWARD     'f'
#define LEFT        'l'
#define RIGHT       'r'

SoftwareSerial HC12(10, 11);         // HC-12 TX Pin, HC-12 RX Pin


byte incomingByte;
int x = 0;
int y = 0;
char currentState = OFF;

void setup() {
  Serial.begin(9600);                   // Open serial port to computer

  //setup HC12
  HC12.begin(9600);                     // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);           // HC-12 normal, transparent mode


  setupMicrophone();
  pinMode(redPin, OUTPUT);              // Configuring the LED pins
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

}

void loop() {
  readHC12feedback();
  char Joystick_state = readJoystick()); 
  readKeyboard();
  if (currentState==OFF){
      if (listenMicrophone()){
        Serial.print("Signaal ontvangen");
        detachInterrupt(digitalPinToInterrupt(MICROPHONE_PIN)); //done listening
        switchState(BALANCE);
      }
  }
  else if ((Joystick_state!=currentState)&(Joystick_state!=BALANCE)) { //running
    switchState(Joystick_state);
  }
  delay(100);
}

void transmitHC12(char newState){
  if (incomingByte!=10){ HC12.print(newState);}
}

char readJoystick(){
  x = analogRead(X_Pin);
  y = analogRead(Y_Pin);
//  Serial.println(x);
  
    if (x <= 700 && x >= 300 && y <= 700 && y >= 300) { return BALANCE;}
    else if (x <= 300 or x >= 700) { return FORWARD;  }
    else if (y <= 300) { return RIGHT;  }
    else if (y >= 700) { return LEFT;   }
}

void setColor(int red, int green, int blue){
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

void switchState(char newState){
  currentState = newState;
  transmitHC12(newState);
}

void readKeyboard(){
  int incomingByte = 0;
  while (Serial.available()) {
    int incomingByte = Serial.read()); 
    if (incomingByte==63){transmitHC12(incomingByte);}  //get current logs
    else if (incomingByte!=10){switchState(incomingByte);}
  }
}

void readHC12feedback(){
  while (HC12.available()) {             // If HC-12 has data
   Serial.print(HC12.read());          
  }
}