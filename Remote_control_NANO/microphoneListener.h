//microphone variables
int val = 0;
int freq = 0;
int counter = 0;
unsigned long timer = 0;
unsigned long start_time = 0;
#define F_MAX 1850 //Max. frequency [Hz]
#define F_MIN 1750 //Min. frequency [Hz]
#define MICROPHONE_PIN 2 //Pin connected to Microphone (MUST BE INTERRUPT PIN!)


void count() {counter = counter + 1;}

void setupMicrophone(){
      attachInterrupt(digitalPinToInterrupt(MICROPHONE_PIN), count , RISING); //Microphone pin
}


byte listenMicrophone(){
  timer = millis();
  if((timer - start_time) > 100){
    freq        = counter * 10;
    start_time  = millis();
    Serial.println(freq);
    counter     = 0;
    if(freq>=F_MIN && freq<=F_MAX){
      return 1;
    }
  }
  return 0;
}
