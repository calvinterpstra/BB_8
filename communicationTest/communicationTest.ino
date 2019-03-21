volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile int val = 0;

void setup() {
  Serial.begin(115200);
  // when pin D2 goes high, call the rising function
  attachInterrupt(0, rising, RISING);
}

void loop() {
  val = pwm_value/20.32;
  if(val <= 5){
    Serial.print("nothing, ");
  } else if(val <= 25){
    Serial.print("stop, ");
  } else if(val <= 50){
    Serial.print("forward, ");
  } else if(val <= 75){
    Serial.print("left, ");
  } else {
    Serial.print("right, ");
  }
  Serial.println(val);
  delay(500);
}

void rising() {
  attachInterrupt(0, falling, FALLING);
  prev_time = micros();
}

void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros() - prev_time;
}
