#define REST 'r'
#define SOUND 's'
#define BALANCE 'b'
#define DRIVE 'd'

auto state = rest;
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

auto updateState(auto state){
    char nextState = getNextState();
    
}

auto rest = [] () {
    Serial.println("rest");
};

auto sound = [] () {
    Serial.println("sound");
};

auto balance = [] () {
    Serial.println("balance");
};

auto drive = [] () {
    Serial.println("drive");
};

void setup() {
  Serial.begin(9600);
}

void loop() {
    state();
    // state = updateState(state);
}
