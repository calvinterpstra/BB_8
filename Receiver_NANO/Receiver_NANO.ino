#define OFF         'o'
#define OFF_BWM     0
#define BALANCE     'b'
#define BALANCE_BWM 32
#define FORWARD     'f'
#define FORWARD_BWM 96
#define LEFT        'l'
#define LEFT_BWM    160
#define RIGHT       'r'
#define RIGHT_BWM   224

#define  microphone_pin 2
#define  pin_to_uno  3
#define  HC12_setPin 6


#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin





///SCRIPT FOR NANO (Receiver)

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


//states definitions
void off          () { analogWrite(pin_to_uno, OFF_BWM    ); }
void balance      () { analogWrite(pin_to_uno, BALANCE_BWM); }
void forward_drive() { analogWrite(pin_to_uno, FORWARD_BWM); }
void left_rotate  () { analogWrite(pin_to_uno, LEFT_BWM   ); }
void right_rotate () { analogWrite(pin_to_uno, RIGHT_BWM  ); }


void setup()
{
  Serial.begin(9600);             // Open serial port to computer
  HC12.begin(9600);               // Open serial port to HC12
  pinMode(HC12_setPin, OUTPUT);
  pinMode(pin_to_uno, OUTPUT);
  digitalWrite(HC12_setPin, HIGH);     // HC-12 normal mode

}

char state = OFF;
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
    case OFF    : { off(); break;}
    case BALANCE: { balance(); break;}
    case LEFT   : { left_rotate(); break;}
    case RIGHT  : { right_rotate(); break;}
    case FORWARD: { forward_drive(); break;}
    default:{      break;}
  }
  //Serial.write(state);
  readKeyboardAndHC12();
}

void writeCurrentState(){
  Serial.print("Current state:");
  Serial.println(state);
}

void readKeyboardAndHC12()
{
  int incomingByte = 0;
  if (Serial.available() > 0)
  {
    int incomingByte = Serial.read();
  }
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();
    int readBuffer = char(incomingByte);    // Add each byte to ReadBuffer string variable 
  }
  if (incomingByte == 63)      { writeCurrentState(            ); } //get current state when typed '?'
  else if (incomingByte != 10) { updateState      (incomingByte); } //update state if char is not empty

}
