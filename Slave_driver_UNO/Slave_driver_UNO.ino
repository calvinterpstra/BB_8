#define pin_to_NANO 2 //digital


volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile int val = 0;


void balance(){

}

void drive_forward(){

}

void rotate_left(){

}

void rotate_right(){

}

void setup() {
  Serial.begin(115200);
  // when pin D2 goes high, call the rising function
  attachInterrupt(pin_to_NANO, rising , RISING);
  attachInterrupt(pin_to_NANO, falling, FALLING);
  }

void loop() {
  val = pwm_value/20.32;
  if(val <= 5){
    Serial.print("nothing, ");
  } else if(val <= 25){
    Serial.print("balance, ");
    balance();
  } else if(val <= 50){
    Serial.print("forward, ");
    drive_forward();
  } else if(val <= 75){
    Serial.print("left, ");
    rotate_left();
  } else {
    Serial.print("right, ");
    rotate_right();
  }
  Serial.println(val);
  delay(500);
}

void rising(){ prev_time = micros();}
void falling(){ pwm_value = micros() - prev_time; }