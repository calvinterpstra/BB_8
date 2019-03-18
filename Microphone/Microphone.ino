int val = 0;
int freq = 0;
int counter = 0;
unsigned long timer = 0;
unsigned long start_time = 0;
int STARTBAND_LOW = 1700;
int STARTBAND_HIGH= 1900;
//band 1700-1900 Hz (group 111)

void setup(){
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), count , RISING);
}

void loop(){
  timer = millis();
  if((timer - start_time) > 100){
    freq        = counter * 10;
    start_time  = millis();
   // Serial.println(freq);
    if ((freq>=STARTBAND_LOW)&&(freq<=STARTBAND_HIGH)}{
      //switchState() 
    }
    counter     = 0;
  }
}

void count() {
  counter = counter + 1;
}
