#include <Wire.h>
#include <TimerOne.h>
//#include <KalmanFilter.h>


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
#define Motor3_in1_Pin  10
#define Motor3_in2_Pin  12

//PID gains:
double kp = 0.075;
double ki = 0.00000003;
double kd = 0.25;
double pitch_Offset = 1.3;
int    motor_start_moving = 60;

//motor_layout:
int motor1Output = 0;
int motor2Output = 0;
int motor3Output = 0;

unsigned long currentTime, previousTime;
double elapsedTime;

void setMotor(int motorPower,int motor_PWM_Pin,int motor_in1_pin, int motor_in2_pin){
  if (motorPower>=0){
    digitalWrite(motor_in1_pin, LOW);
    digitalWrite(motor_in2_pin, HIGH);
  }
  else{
    digitalWrite(motor_in1_pin, HIGH);
    digitalWrite(motor_in2_pin, LOW);
  }

  motorPower = abs(motorPower);
  if (motorPower>255) {motorPower =255; }
  if (motorPower>10){
      analogWrite(motor_PWM_Pin, motorPower+motor_start_moving);
  }

}

void setMotors(int motor1Power, int motor2Power, int motor3Power) {
  int minimum_power = min(abs(motor1Power), min(abs(motor2Power),abs(motor3Power)));
  if (abs(motor1Power) == abs(minimum_power)){motor1Power = 0;}
  else if (abs(motor2Power) == abs(minimum_power)){motor2Power = 0;}
  else if (abs(motor3Power) == abs(minimum_power)){motor3Power = 0;}

  setMotor(motor1Power,Motor1_PWM_Pin,Motor1_in1_Pin,Motor1_in2_Pin);
  setMotor(motor2Power,Motor2_PWM_Pin,Motor2_in1_Pin,Motor2_in2_Pin);
  setMotor(motor3Power,Motor3_PWM_Pin,Motor3_in1_Pin,Motor3_in2_Pin);
}

//MPU:
volatile bool intFlag = false;
double pitch_input, aP_output, a1_output, a1_input, a2_output, a2_input;
long int cpt = 0;

//KalmanFilter kalmanX(0.001, 0.003, 0.03);
//KalmanFilter kalmanY(0.001, 0.003, 0.03);
//
//float accPitch = 0;
//float accRoll = 0;
//
//float kalPitch = 0;
//float kalRoll = 0;

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  
  while (Wire.available())
    Data[index++] = Wire.read();
}


void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void callback() {
  intFlag = true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void setup()   { 
  Wire.begin();
  Serial.begin(115200);

  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);                      // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);                      // Set gyroscope low pass filter at 5Hz

  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);        // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);                    // Set by pass mode for the magnetometers

  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);                        // Request continuous magnetometer measurements in 16 bits

  pinMode(13, OUTPUT);
  Timer1.initialize(10000);                                     // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);                             // attaches callback() as a timer overflow interrupt

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

#define GYROSCOPE_SENSITIVITY 120 //60


float corrected_pitch;
float corrected_roll;

#define Acc_percentage 0.01

void ComplementaryFilter(int16_t gx,int16_t gy,int16_t ax,int16_t ay, int16_t az)
{   //http://www.pieter-jan.com/node/11
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    
    if (abs(gx)>0) corrected_pitch += ((gx-27) / GYROSCOPE_SENSITIVITY) * elapsedTime; // Angle around the X-axis
    if (abs(gy)>0) corrected_roll  -= ((gy-27) / GYROSCOPE_SENSITIVITY) * elapsedTime;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
   // Serial.println(forceMagnitudeApprox);
    //if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    if (true)
    {
	// Turning around the X axis results in a vector on the Y-axis

        corrected_pitch = corrected_pitch * (1-Acc_percentage) + ax * Acc_percentage;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = small_angle_atan2(ax, az) * 180 / 3.14159274;
        corrected_roll = corrected_roll * (1-Acc_percentage) + rollAcc * Acc_percentage;
    }
} 

void loop(){
  while (!intFlag);
  intFlag = false;

  // ::: Counter :::
  // Display data counter
  //  Serial.print (cpt++,DEC);
  //  Serial.print ("\t");

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]); //pitch
  int16_t ay = -(Buf[2] << 8 | Buf[3]); //roll




  //Accelerometer filtered
   int16_t az = Buf[4] << 8 | Buf[5];  
  // int16_t alin = sqrt(ay^2+ az^2);
  // ax = -(small_angle_atan2(ax, alin)*180.0)/M_PI;
  // ay  = (small_angle_atan2(ay, az)*180.0)/M_PI;


  // Gyroscope
  int16_t gy = -(Buf[8] << 8 | Buf[9]);
  int16_t gx = -(Buf[10] << 8 | Buf[11]);

  // Kalman filter (use ax in combination with accelerometer filter)
  //kalPitch = kalmanY.update(accPitch, gy);
  //kalRoll = kalmanX.update(accRoll, gx);

  ComplementaryFilter(gx,gy, ax,ay,az);

  Serial.print(gx);
  Serial.print(",");
  Serial.print(ax);
  Serial.print(",");
  Serial.println(corrected_pitch);

  //CHOOSE MEASUREMENTS
  ax = ax; //use gyo
  ay = ay; //use gyro


// Define new Axes
  int16_t aP = ax;
  int16_t a1 = (sqrt(3.)/2)*ay+(0.5)*ax;
  int16_t a2 = (sqrt(3.)/2)*ay-(0.5)*ax;



  // Display values

  //Serial.println(ax);
  //Serial.print ("\t");
  //Serial.print (ay,DEC);
  //  Serial.print ("\t");
  //Serial.print (a1,DEC);
  //  Serial.print ("\t");
  //Serial.print (a2,DEC);
  //  Serial.print ("\t");


  currentTime = millis();                                      //get current time
  elapsedTime = (double)(currentTime - previousTime);          //compute time elapsed from previous computation

  double pitch_Setpoint = 0; 
  double a1_Setpoint = 0;
  double a2_Setpoint = 0;

// OLD PID
//double roll_Setpoint = 0; 
//  roll_input = map(-(Buf[2] << 8 | Buf[3]), -8500, 8500, -1023, 1023);
//  //double roll_offset = -179.32;
//  //roll_input = map(kalRoll - roll_offset, -800, 800, -1023, 1023);
//  roll_output = map(computePID_roll(roll_input, roll_Setpoint), -10000, 10000, -255, 255);

  //PID's
  aP_output = computePID_pitch(aP, pitch_Setpoint);
  a1_output = computePID_a1(a1, a1_Setpoint);
  a2_output = computePID_a2(a2, a2_Setpoint);

  a1_output = 0;
  a2_output = 0;

  previousTime = currentTime;                                  //remember current time

  //Sum PID's and process in Motor Controller
  motor1Output = aP_output + a2_output;
  motor2Output = -aP_output + a1_output;
  motor3Output = (-a1_output - a2_output);
  
  //setMotors(200, 200, 200);
  setMotors(motor1Output, motor2Output, motor3Output);

  //monitor output motors
   //Serial.println(motor1Output);
   //Serial.println(motor2Output);
  // Serial.print(motor3Output);
  // Serial.println("");
}

//PID for pitch control:
double computePID_pitch(double inp, double Setpoint) {
  double error;
  double lastError;
  double cumError, rateError;
  
  error = Setpoint - inp;                                      // determine error
  cumError = error * elapsedTime;                             // compute integral
  rateError = (error - lastError) / elapsedTime;               // compute derivative
  double out = kp * error + ki * cumError + kd * rateError;

  lastError = error;                                           //remember current error
  return out;                                                  //have function return the PID output
}

//PID for axis 1 control:
double computePID_a1(double inp_a1, double Setpoint_a1) {
  double error_a1;
  double lastError_a1;
  double cumError_a1, rateError_a1;
  
  error_a1 = Setpoint_a1 - inp_a1;                                      // determine error
  cumError_a1 = error_a1 * elapsedTime;                             // compute integral
  rateError_a1 = (error_a1 - lastError_a1) / elapsedTime;               // compute derivative
  double out_a1 = kp * error_a1 + ki * cumError_a1 + kd * rateError_a1;

  lastError_a1 = error_a1;                                           //remember current error
  return out_a1;                                                  //have function return the PID output
}

//PID for axis 2 control:
double computePID_a2(double inp_a2, double Setpoint_a2) {
  double error_a2;
  double lastError_a2;
  double cumError_a2, rateError_a2;
  
  error_a2 = Setpoint_a2 - inp_a2;                                      // determine error
  cumError_a2 = error_a2 * elapsedTime;                             // compute integral
  rateError_a2 = (error_a2 - lastError_a2) / elapsedTime;               // compute derivative
  double out_a2 = kp * error_a2 + ki * cumError_a2 + kd * rateError_a2;

  lastError_a2 = error_a2;                                           //remember current error
  return out_a2;                                                  //have function return the PID output
}
