//microphone variables
int val = 0;
int freq = 0;
int counter = 0;
unsigned long timer = 0;
unsigned long start_time = 0;
int STARTBAND_LOW = 1700;
int STARTBAND_HIGH= 1900;  //band 1700-1900 Hz (group 111)
void count() {counter = counter + 1;}
#define microphone_pin 13



void setupMicrophone(){
      attachInterrupt(digitalPinToInterrupt(microphone_pin), count , RISING); //Microphone pin
}


byte listenMicrophone(){
    timer = millis();
    if((timer - start_time) > 100){
        freq        = counter * 10;
        start_time  = millis();
        Serial.println(freq);
        if ((freq>=STARTBAND_LOW)&&(freq<=STARTBAND_HIGH)){
          detachInterrupt(digitalPinToInterrupt(microphone_pin)); //done listening
          return 1;
          
        }
        counter     = 0;
        return 0;
}
return 0;
}
