#include <Wire.h>
#include <TimerOne.h>
#include <KalmanFilter.h>


#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define Motor1_PWM_Pin  6
#define Motor1_in1_Pin  7
#define Motor1_in2_Pin  8
#define Motor2_PWM_Pin  5
#define Motor2_in1_Pin  4
#define Motor2_in2_Pin  3
#define Motor3_PWM_Pin  11
#define Motor3_in1_Pin  12
#define Motor3_in2_Pin  10

//PID gains:
double kp = 25;
double ki = 0.0001;
double kd = 90;
double pitch_Offset = 1.3;

//motor_layout:
int motor1Output = 0;
int motor2Output = 0;
int motor3Output = 0;

void setMotors(int motor1Power, int motor2Power, int motor3Power) {
  if (motor1Power >= 0) {
    analogWrite(Motor1_PWM_Pin, motor1Power);
    digitalWrite(Motor1_in1_Pin, LOW);
    digitalWrite(Motor1_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor1_PWM_Pin, -motor1Power);
    digitalWrite(Motor1_in1_Pin, HIGH);
    digitalWrite(Motor1_in2_Pin, LOW);

  }
  if (motor2Power >= 0) {
    analogWrite(Motor2_PWM_Pin, motor2Power);
    digitalWrite(Motor2_in1_Pin, HIGH);
    digitalWrite(Motor2_in2_Pin, LOW);
  }
  else {
    analogWrite(Motor2_PWM_Pin, -motor2Power);
    digitalWrite(Motor2_in1_Pin, LOW);
    digitalWrite(Motor2_in2_Pin, HIGH);
  }
  if (motor3Power >= 0) {
    analogWrite(Motor3_PWM_Pin, motor3Power);
    digitalWrite(Motor3_in1_Pin, HIGH);
    digitalWrite(Motor3_in2_Pin, LOW);
  }
  else {
    analogWrite(Motor3_PWM_Pin, -motor3Power);
    digitalWrite(Motor3_in1_Pin, LOW);
    digitalWrite(Motor3_in2_Pin, HIGH);
  }
}

//MPU:
long int ti;
volatile bool intFlag = false;
double pitch_input, pitch_output, a1_output, a1_input, a2_output, a2_input;
long int cpt = 0;

//KalmanFilter kalmanX(0.001, 0.003, 0.03);
//KalmanFilter kalmanY(0.001, 0.003, 0.03);
//
//float accPitch = 0;
//float accRoll = 0;
//
//float kalPitch = 0;
//float kalRoll = 0;

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  
  while (Wire.available())
    Data[index++] = Wire.read();
}


void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void callback()
{
  intFlag = true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void setup()
{ 
  Wire.begin();
  Serial.begin(115200);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt


  // Store initial time
  ti = millis();

  //motor pin layout:
  pinMode(Motor1_PWM_Pin, OUTPUT);
  pinMode(Motor1_in1_Pin, OUTPUT);
  pinMode(Motor1_in2_Pin, OUTPUT);
  pinMode(Motor2_PWM_Pin, OUTPUT);
  pinMode(Motor2_in1_Pin, OUTPUT);
  pinMode(Motor2_in2_Pin, OUTPUT);
  pinMode(Motor3_PWM_Pin, OUTPUT);
  pinMode(Motor3_in1_Pin, OUTPUT);
  pinMode(Motor3_in2_Pin, OUTPUT);
}

static inline double offset (double motor_value){
    if (motor1Output <= 0){
      return motor_value-60;
    }
    return motor_value+60;
  }

void loop(){


  while (!intFlag);
  intFlag = false;

  //  // Display time
  //  Serial.print (millis()-ti,DEC);
  //  Serial.print ("\t");
  //
  //
  // _______________
  // ::: Counter :::

  // Display data counter
  //  Serial.print (cpt++,DEC);
  //  Serial.print ("\t");



  // ____________________________________
  // :::  accelerometer and gyroscope :::

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]); //pitch
  int16_t ay = -(Buf[2] << 8 | Buf[3]); //roll
  int16_t az = Buf[4] << 8 | Buf[5];
  //int16_t alin = sqrt(ay^2+ az^2);
  // Gyroscope
  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];

  //accPitch = -(atan2(ax, alin)*180.0)/M_PI;
  //accRoll  = (atan2(ay, az)*180.0)/M_PI;

  // Kalman filter
  //kalPitch = kalmanY.update(accPitch, gy);
  //kalRoll = kalmanX.update(accRoll, gx);

  // Display values

  // Accelerometer
  //Serial.print (ax, DEC);
  //Serial.print ("\t");
  //Serial.print (ay,DEC);
  //  Serial.print ("\t");
  //  Serial.print (az,DEC);
  //  Serial.print ("\t");
  //
  //  // Gyroscope
  //  Serial.print (gx,DEC);
  //  Serial.print ("\t");
  //  Serial.print (gy,DEC);
  //  Serial.print ("\t");
  //  Serial.print (gz,DEC);
  //  Serial.print ("\t");

  // End of line
  //Serial.println("");
  //  delay(100);

  //PID loop
//double roll_Setpoint = 0; 
//  roll_input = map(-(Buf[2] << 8 | Buf[3]), -8500, 8500, -1023, 1023);
//  //double roll_offset = -179.32;
//  //roll_input = map(kalRoll - roll_offset, -800, 800, -1023, 1023);
//  roll_output = map(computePID_roll(roll_input, roll_Setpoint), -10000, 10000, -255, 255);

  double pitch_Setpoint = 0; 
  pitch_input = map(-(Buf[0] << 8 | Buf[1]), -8500, 8500, -1023, 1023);
  //double pitch_offset = 66.3;
  //pitch_input = map(kalPitch - pitch_offset, -800, 800, -1023, 1023);              
  pitch_output = map(computePID_pitch(pitch_input, pitch_Setpoint), -10000, 10000, -255, 255);
  
  double a1_Setpoint = 0;
  int16_t a1 = (sqrt(3.)/2)*ay+(0.5)*ax;
  a1_input = map(a1, -8500, 8500, -1023, 1023);
  a1_output = map(computePID_a1(a1_input, a1_Setpoint), -10000, 10000, -255, 255);

  double a2_Setpoint = 0;
  int16_t a2 = (sqrt(3.)/2)*ay-(0.5)*ax;
  a2_input = map(a2, -8500, 8500, -1023, 1023);
  a2_output = map(computePID_a2(a2_input, a2_Setpoint), -10000, 10000, -255, 255);

  //MotorControl
  motor1Output = pitch_output*pitch_Offset + a2_output;
  motor2Output = -pitch_output*pitch_Offset + a1_output;
  motor3Output = (-a1_output - a2_output)/1.379;

//  motor1Output = offset(motor1Output);
//  motor2Output = offset(motor2Output);
//  motor3Output = offset(motor3Output);

//  if(abs(ax) >= abs(a1) && abs(ax) >= abs(a2)){
//    a1_output = 0;
//    a2_output = 0;
//  }
  
  if(abs(motor1Output) <= abs(motor2Output) && abs(motor1Output) <= abs(motor3Output)){
    motor1Output = 0;
    if(motor2Output <= 0){
      motor2Output = motor2Output-60;
    }
    if(motor2Output >= 0){
      motor2Output = motor2Output+60;
    }
    if(motor3Output <= 0){
      motor3Output = motor3Output-60;
    }
    if(motor3Output >= 0){
      motor3Output = motor3Output+60;
    }
  }
  else if(abs(motor2Output) <= abs(motor1Output) && abs(motor2Output) <= abs(motor3Output)){
    motor2Output = 0;
    if(motor1Output <= 0){
      motor1Output = motor1Output-60;
    }
    if(motor1Output >= 0){
      motor1Output = motor1Output+60;
    }
    if(motor3Output <= 0){
      motor3Output = motor3Output-60;
    }
    if(motor3Output >= 0){
      motor3Output = motor3Output+60;
    }
  }
  else if(abs(motor3Output) <= abs(motor1Output) && abs(motor3Output) <= abs(motor2Output)){
    motor3Output = 0;
    if(motor1Output <= 0){
    motor1Output = motor1Output-60;
    }
    if(motor1Output >= 0){
      motor1Output = motor1Output+60;
    }
    if(motor2Output <= 0){
      motor2Output = motor2Output-60;
    }
    if(motor2Output >= 0){
      motor2Output = motor2Output+60;
    }
  }

  if (motor1Output >= 255) {
  motor1Output = 255;
  }
  if (motor1Output <= -255) {
  motor1Output = -255;
  }

  if(motor2Output >= 255){
  motor2Output = 255;
  }
  if(motor2Output <= -255){
  motor2Output = -255;
  }

  if(motor3Output >= 255){
  motor3Output = 255;
  }
  if(motor3Output <= -255){
  motor3Output = -255;
  }

  
  //setMotors(200, 200, 200);
  setMotors(motor1Output, motor2Output, motor3Output);
  Serial.print(motor1Output);
  Serial.print("\t");
  Serial.print(motor2Output);
  Serial.print("\t");
  Serial.print(motor3Output);
  Serial.println("");
}

//PID for pitch control:
double computePID_pitch(double inp, double Setpoint) {
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error;
  double lastError;
  double cumError, rateError;
  
  currentTime = millis();                                      //get current time
  elapsedTime = (double)(currentTime - previousTime);          //compute time elapsed from previous computation

  error = Setpoint - inp;                                      // determine error
  cumError = error * elapsedTime;                             // compute integral
  rateError = (error - lastError) / elapsedTime;               // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;

  lastError = error;                                           //remember current error
  previousTime = currentTime;                                  //remember current time

  return out;                                                  //have function return the PID output
}

//PID for axis 1 control:
double computePID_a1(double inp_a1, double Setpoint_a1) {
  unsigned long currentTime_a1, previousTime_a1;
  double elapsedTime_a1;
  double error_a1;
  double lastError_a1;
  double cumError_a1, rateError_a1;
  
  currentTime_a1  = millis();                                      //get current time
  elapsedTime_a1 = (double)(currentTime_a1 - previousTime_a1);          //compute time elapsed from previous computation

  error_a1 = Setpoint_a1 - inp_a1;                                      // determine error
  cumError_a1 = error_a1 * elapsedTime_a1;                             // compute integral
  rateError_a1 = (error_a1 - lastError_a1) / elapsedTime_a1;               // compute derivative

  double out_a1 = kp * error_a1 + ki * cumError_a1 + kd * rateError_a1;

  lastError_a1 = error_a1;                                           //remember current error
  previousTime_a1 = currentTime_a1;                                  //remember current time

  return out_a1;                                                  //have function return the PID output
}

//PID for axis 2 control:
double computePID_a2(double inp_a2, double Setpoint_a2) {
  unsigned long currentTime_a2, previousTime_a2;
  double elapsedTime_a2;
  double error_a2;
  double lastError_a2;
  double cumError_a2, rateError_a2;
  
  currentTime_a2  = millis();                                      //get current time
  elapsedTime_a2 = (double)(currentTime_a2 - previousTime_a2);          //compute time elapsed from previous computation

  error_a2 = Setpoint_a2 - inp_a2;                                      // determine error
  cumError_a2 = error_a2 * elapsedTime_a2;                             // compute integral
  rateError_a2 = (error_a2 - lastError_a2) / elapsedTime_a2;               // compute derivative

  double out_a2 = kp * error_a2 + ki * cumError_a2 + kd * rateError_a2;

  lastError_a2 = error_a2;                                           //remember current error
  previousTime_a2 = currentTime_a2;                                  //remember current time

  return out_a2;                                                  //have function return the PID output
}






