#define REST 'r'
#define SOUND 's'
#define BALANCE 'b'
#define DRIVE 'd'

//microphone variables
int val = 0;
int freq = 0;
int counter = 0;
unsigned long timer = 0;
unsigned long start_time = 0;
int STARTBAND_LOW = 1700;
int STARTBAND_HIGH= 1900;  //band 1700-1900 Hz (group 111)
void count() {counter = counter + 1;}

unsigned long timeOffest;

unsigned long getTime(){
    return millis() - timeOffest;
}

void resetTimer(){
    timeOffest = millis();
}

char getNextState(){
    // int incomingByte = 0;
    if (Serial.available() > 0) {
        int incomingByte = Serial.read();
        return char(incomingByte);
    }
}

char updateState(char state){
    char nextState = getNextState();
    return nextState;
}

void rest(){
}

void sound(){
  timer = millis();
  if((timer - start_time) > 100){
    freq        = counter * 10;
    start_time  = millis();
    Serial.println(freq);
    if ((freq>=STARTBAND_LOW)&&(freq<=STARTBAND_HIGH)}{
      updateState(BALANCE);
    }
    counter     = 0;
}




void balance(){
}

void drive(){
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), count , RISING); //Microphone pin
}

char state = REST;
void loop() {
    switch (state) {
        case REST:
            rest();
            break;
        case SOUND:
            sound();
            break;
        case BALANCE:
            balance();
            break;
        case DRIVE:
            drive();
            break;
        default:
            Serial.print("default: ");
            Serial.println(state);
            break;
    }
    state = updateState(state);
}
