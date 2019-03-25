// #define OFF         'o'
// #define OFF_BWM     0
// #define BALANCE     'b'
// #define BALANCE_BWM 32
// #define FORWARD     'f'
// #define FORWARD_BWM 96
// #define LEFT        'l'
// #define LEFT_BWM    160
// #define RIGHT       'r'
// #define RIGHT_BWM   224


volatile int pwm_value = 0;
volatile int prev_time = 0;
volatile int val = 0;


void setup() {
  Serial.begin(115200);
  // when pin D2 goes high, call the rising function
  attachInterrupt(0, rising, RISING);
  attachInterrupt(0, falling, FALLING);
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

void rising()  { prev_time = micros();}
void falling() { pwm_value = micros() - prev_time; }
