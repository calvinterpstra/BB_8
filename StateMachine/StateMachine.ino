#define REST 'r'
#define SOUND 's'
#define BALANCE 'b'
#define DRIVE 'd'
#include "microphoneListener.h"
 

unsigned long timeOffest;

unsigned long getTime(){
    return millis() - timeOffest;
}

void resetTimer(){
    timeOffest = millis();
}



void rest(){
}


void balance(){
}

void drive(){
}

void setup() {
  Serial.begin(9600);
  setupMicrophone();
}

char state = REST;
void loop() {
    switch (state) {
        case REST:
            rest();
            break;
        case SOUND:
            byte signal_received = listenMicrophone();
            if (signal_received){ updateState(BALANCE);}
            break;
        case BALANCE:
            balance();
            break;
        case DRIVE:
            drive();
            break;
       knmbkekvnb default:
            break;
    }
    //Serial.write(state);
    readKeyboard();
}

void readKeyboard(){
    int incomingByte = 0;
    if (Serial.available() > 0) {
        int incomingByte = Serial.read();
        if (incomingByte==63){
          Serial.print("Current state:");
          Serial.println(state);
        }
        else if (incomingByte != 10){    updateState(incomingByte); }
    }
}


void updateState(char nextState){
    state = nextState;
    Serial.print("new state:");
    Serial.println(state);
}