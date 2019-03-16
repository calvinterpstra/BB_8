#define REST 'r'
#define SOUND 's'
#define BALANCE 'b'
#define DRIVE 'd'

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
}

void balance(){
}

void drive(){
}

void setup() {
  Serial.begin(9600);
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