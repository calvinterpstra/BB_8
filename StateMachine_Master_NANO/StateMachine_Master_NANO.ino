#define OFF 'o'
#define OFF_BWM 0
#define SOUND 's'
#define BALANCE 'b'
#define BALANCE_BWM 32
#define FORWARD 'f'
#define FORWARD_BWM 96
#define LEFT 'l'
#define LEFT_BWM 160
#define RIGHT 'r'
#define RIGHT_BWM 224

#include "microphoneListener.h"
#define microphone_pin 2
#define pin_to_uno 3





///SCRIPT FOR NANO

//0   1-63: balance, 64-127:forward, 128-191:left rotate, 192:254: right rotate
//off      32           96             160                    224      


//let op::: bij analogWrite: maximum 254 ipv 255

unsigned long timeOffest;

unsigned long getTime()
{
  return millis() - timeOffest;
}

void resetTimer()
{
  timeOffest = millis();
}

void off(){    analogWrite(pin_to_uno,OFF_BWM);}
void balance() analogWrite(pin_to_uno, BALANCE_BWM); }
void forward_drive(){analogWrite(pin_to_uno, FORWARD_BWM);}
void left_rotate() {analogWrite(pin_to_uno, LEFT_BWM);}
void right_rotate() {analogWrite(pin_to_uno, RIGHT_BWM);}


void setup()
{
  Serial.begin(9600);
  setupMicrophone();
  pinMode(pin_to_uno, OUTPUT);

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
    case OFF:
      off();
      break;
    case SOUND:
      //byte signal_received = ;
      if (listenMicrophone())  {
        updateState(BALANCE);
        detachInterrupt(digitalPinToInterrupt(microphone_pin)); //done listening
      }
      break;
    case BALANCE:
      balance();
      break;
    case LEFT: 
      left_rotate();
      break;
    case RIGHT:
      right_rotate();
      break;
    case FORWARD:
      forward_drive();
      break;
    default:
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