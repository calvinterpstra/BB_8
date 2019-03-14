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
    int incomingByte = 0;
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        return char(incomingByte);
    }
}

char updateState(char state){
    char nextState = getNextState();
    switch (state){
        case REST:
            if (nextState == SOUND || nextState == BALANCE){
                return nextState;
            } else { return state; }
            break;
        case SOUND:
            if (nextState == 'F' || nextState == REST){
                return nextState;
            } else { return state; }
            break;
        case BALANCE:
            if (nextState == REST || nextState == 'W'){
                return nextState;
            } else { return state; }
            break;
        case DRIVE:
            if (nextState == SOUND){
                return nextState;
            } else { return state; }
            break;
        default:
            return state; 
            break;
    }
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