#define REST 'r'
#define SOUND 's'
#define BALANCE 'b'
#define DRIVE 'd'
#include "microphoneListener.h"
#define microphone_pin 2

///SCRIPT FOR NANO

unsigned long timeOffest;

unsigned long getTime()
{
    return millis() - timeOffest;
}

void resetTimer()
{
    timeOffest = millis();
}

void rest()
{
}

void balance()
{
    digitalWrite(3,HIGH);
    digitalWrite(5,HIGH);
    digitalWrite(7,HIGH);
  
}

void drive()
{
    digitalWrite(3,LOW);
    digitalWrite(5,LOW);
    digitalWrite(7,LOW);
}

void setup()
{
    Serial.begin(9600);
    setupMicrophone();
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(7, OUTPUT);

}

char state = REST;
void updateState(char nextState)
{
    state = nextState;
    Serial.print("new state:");
    Serial.println(state);
}

void loop()
{
    switch (state)
    {
    case REST:
        rest();
        break;
    case SOUND:
        //byte signal_received = ;
        if (listenMicrophone())
        {
            updateState(BALANCE);
            detachInterrupt(digitalPinToInterrupt(microphone_pin)); //done listening
        }
        break;
    case BALANCE:
        balance();
        break;
    case DRIVE:
        drive();
        break;
    case default:
        break;
    }
    //Serial.write(state);
    readKeyboard();
}

void readKeyboard()
{
    int incomingByte = 0;
    if (Serial.available() > 0)
    {
        int incomingByte = Serial.read();
        if (incomingByte == 63)
        {
            Serial.print("Current state:");
            Serial.println(state);
        }
        else if (incomingByte != 10)
        {
            updateState(incomingByte);
        }
    }
}
