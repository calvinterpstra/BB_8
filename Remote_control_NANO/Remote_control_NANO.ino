#include <SoftwareSerial.h>
#include "microphoneListener.h"

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

void readHC12feedback(){
  while (HC12.available()) {             // If HC-12 has data
   Serial.print(char(HC12.read()));          
  }
}

void setup() {
  Serial.begin(2000000);                   // Open serial port to computer

  //setup HC12
  HC12.begin(9600);                     // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);           // HC-12 normal, transparent mode

  //Microphone setup
  setupMicrophone();

  //LED Setup
  pinMode(redPin, OUTPUT);              // Configuring the LED pins
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);


  pinMode(X_Pin, INPUT);
  pinMode(Y_Pin, INPUT);

  
  setColor(OFF);
}

// States
void off(){
  if (listenMicrophone()){
//    Serial.println("Signaal ontvangen");
    detachInterrupt(digitalPinToInterrupt(MICROPHONE_PIN)); //done listening
    switchState(BALANCE);
  }
}

void balance(){
  setColor(BALANCE);
//  Serial.print("b, ");
  char Joystick_state = readJoystick();
  switchState(Joystick_state);
}

void forward(){
  setColor(FORWARD);
//  Serial.print("f, ");
  char Joystick_state = readJoystick();
  switchState(Joystick_state);
}

void left(){
  setColor(LEFT);
//  Serial.print("l, ");
  char Joystick_state = readJoystick();
  switchState(Joystick_state);
}

void right(){
  setColor(RIGHT);
//  Serial.print("r, ");
  char Joystick_state = readJoystick();
  switchState(Joystick_state);
}

void loop() {
//  Serial.print("state: ");
//  Serial.println(currentState);
  readHC12feedback();
  switch (currentState){
    case(OFF):
      off();
      break;
    case(BALANCE):
      balance();
      break;
    case(FORWARD):
      forward();
      break;
    case(LEFT):
      left();
      break;
    case(RIGHT):
      right();
      break;
    default:
      break;
  }
  getNextState();
}

void transmitHC12(char newState){
  if (incomingByte!=10){ HC12.print(newState);}
}

char readJoystick(){
  x = analogRead(X_Pin);
  y = analogRead(Y_Pin);
//  Serial.print(x);
//  Serial.print(", ");
//  Serial.println(y);
//  if (x <= 700 && x >= 300 && y <= 700 && y >= 300) { return BALANCE;}
  if (x >= 700) { return FORWARD;  }
  else if (y <= 300) { return RIGHT;  }
  else if (y >= 700) { return LEFT;   }
  else if (x<=300) { transmitHC12('m'); return BALANCE; }
  else {return BALANCE;}
}

void switchState(char newState){
  if(!(newState == currentState)){
    setColor(newState);
    currentState = newState;
    transmitHC12(newState);
  }
}

void getNextState(){
  int incomingByte = 0;
  while (Serial.available()) {
    int incomingByte = Serial.read(); 
//    if (incomingByte==63){transmitHC12(incomingByte);}  //get current logs
    if (incomingByte==OFF){
        attachInterrupt(digitalPinToInterrupt(MICROPHONE_PIN), count , FALLING); //Start listening
        switchState(incomingByte);
    }
    else{
      HC12.print(char(incomingByte));
    }
  }
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

void setColor(char state){
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  int red;
  int green;
  int blue;
  switch(state){
    case(OFF):
      red = 255;
      green = 0;
      blue = 0;
      break;
    case(BALANCE):
      red = 0;
      green = 255;
      blue = 0;
      break;
    case(FORWARD):
      red = 0;
      green = 0;
      blue = 255;
      break;
    case(LEFT):
      red = 255;
      green = 255;
      blue = 0;
      break;
    case(RIGHT):
      red = 255;
      green = 0;
      blue = 255;
      break;
    default:
      break;
  }
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
