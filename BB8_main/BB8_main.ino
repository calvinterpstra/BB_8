#include <Wire.h>
#include <TimerOne.h>

#include <SoftwareSerial.h>

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
#define Motor2_in1_Pin  13
#define Motor2_in2_Pin  4
#define Motor3_PWM_Pin  11
#define Motor3_in1_Pin  12
#define Motor3_in2_Pin  10

#define MANUAL 2
#define DEMO 1
#define NORMAL 0

#define wortel3_2 0.866025
#define setPin 2


//#define motor_offset 100;
#define motor_offset 18
#define MOTOR_MAX 200
//#define motor_offset 30;
//PID gains:


double kp = 0.31;
double kd = 4.2;

//double kp = 0.014;
//double kd = 0.8;

//motor_layout:
int motor1Output = 0;
int motor2Output = 0;
int motor3Output = 0;

uint8_t Buf[14];
unsigned long currentTime, previousTime;
double elapsedTime = 2.462;

int modus = NORMAL;

SoftwareSerial HC12(3, 9); // HC-12 TX Pin, HC-12 RX Pin

char incomingByte;
String readBuffer = "";

unsigned long oldTime,newTime;


void setMotor(int motorPower,int motor_PWM_Pin,int motor_in1_pin, int motor_in2_pin){
  if (motorPower>=0){
    digitalWrite(motor_in1_pin, LOW);
    digitalWrite(motor_in2_pin, HIGH);
  }
  else{
    digitalWrite(motor_in1_pin, HIGH);
    digitalWrite(motor_in2_pin, LOW);
  }

  int motor_power = abs(motorPower);
  if (motor_power>5){ 
    if (motor_PWM_Pin==Motor1_PWM_Pin) {motor_power +=       150 -motor_offset;}  //offset
    else if (motor_PWM_Pin==Motor2_PWM_Pin){ motor_power +=  150 - motor_offset;}
    else if (motor_PWM_Pin==Motor3_PWM_Pin) {motor_power +=  140 - motor_offset;}
  }
  if (motor_power>MOTOR_MAX) {motor_power = MOTOR_MAX; }

  analogWrite(motor_PWM_Pin, motor_power);
}

void setMotors(int motor1Power, int motor2Power, int motor3Power) {
  if (modus!=MANUAL){
    int minimum_power = min(abs(motor1Power), min(abs(motor2Power),abs(motor3Power)));
    if (abs(motor1Power) == abs(minimum_power)){motor1Power = 0;}
    else if (abs(motor2Power) == abs(minimum_power)){motor2Power = 0;}
    else if (abs(motor3Power) == abs(minimum_power)){motor3Power = 0;}
  }

  setMotor(motor1Power,Motor1_PWM_Pin,Motor1_in1_Pin,Motor1_in2_Pin);
  setMotor(motor2Power,Motor2_PWM_Pin,Motor2_in1_Pin,Motor2_in2_Pin);
  setMotor(motor3Power,Motor3_PWM_Pin,Motor3_in1_Pin,Motor3_in2_Pin);

    //monitor output motors
 // Serial.print(motor1Power);
 // Serial.print(',');
 // Serial.print(motor2Power);
 // Serial.print(',');
 // Serial.println(motor3Power);
}

//MPU:
//volatile bool intFlag = false;
double pitch_Setpoint = 0; 
double a1_Setpoint = 0;
double a2_Setpoint = 0;
double aP_output, a1_output, a2_output;
//long int cpt = 0;

float corrected_pitch;
float corrected_roll;

int16_t ax,ay,az,gx,gy, calc1,calc2,aP,a1,a2;

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

void setup()   { 
  Wire.begin();
  Serial.begin(2000000);
  HC12.begin(9600);               // Open serial port to HC12
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);     // HC-12 normal mode

  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);                      // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);                      // Set gyroscope low pass filter at 5Hz

  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);        // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);                    // Set by pass mode for the magnetometers

  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);                        // Request continuous magnetometer measurements in 16 bits

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

  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  if (modus==DEMO){
    elapsedTime += 15;
  }
  //set initial values
  ax = -(Buf[0] << 8 | Buf[1]); //pitch
  ay = -(Buf[2] << 8 | Buf[3]); //roll
  
  aP = corrected_pitch;
  calc1 = wortel3_2*corrected_roll; //is used twice, only calculated once
  calc2 = 0.5*corrected_pitch;
  a1 = calc1+calc2;
  a2 = calc1-calc2;
  
  pitch_Setpoint = ax; //pitch
  a1_Setpoint = a1;
  a2_Setpoint = a2;

  corrected_pitch = ax;
  corrected_roll = ay; //roll
  
}

#define GYROSCOPE_SENSITIVITY 180 //           goeie = 130
#define Acc_percentage 0.005                  ///goeie = 0.04
#define gyro_offsetx    27
#define gyro_offsety    49

float gyr_percentage = 1-Acc_percentage;

void ComplementaryFilter(int16_t gx,int16_t gy,int16_t ax,int16_t ay, int16_t az)
{   //http://www.pieter-jan.com/node/11
         
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    corrected_pitch += ((gx-gyro_offsetx) / GYROSCOPE_SENSITIVITY) * elapsedTime; // Angle around the X-axis
    corrected_roll  -= ((gy-gyro_offsety) / GYROSCOPE_SENSITIVITY) * elapsedTime;    // Angle around the Y-axis
 
    corrected_pitch = corrected_pitch * gyr_percentage + ax * Acc_percentage;
    corrected_roll = corrected_roll * gyr_percentage + ay * Acc_percentage;
    
} 

void loop(){

  //send and receive HC12
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = char(HC12.read());
    Serial.print(incomingByte);
    if (incomingByte == 'd'){
      modus = DEMO;
    }
    else if (incomingByte == 'm'){
      if (modus==MANUAL)      modus = NORMAL;
      else modus = MANUAL;
      setMotors(0,0,0);
    }
    else if (modus==MANUAL){
      if (incomingByte == 'b') setMotors(0,0,0);
      else if (incomingByte == 'f') setMotors(MOTOR_MAX,-MOTOR_MAX,0);
      else if (incomingByte == 'r') setMotors(-100,-100,-100);
      else if (incomingByte == 'l') setMotors(100,100,100);
    }
  }

//  while (Serial.available()) {
//    HC12.write(Serial.read());
//  }

  // Read accelerometer and gyroscope

  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Accelerometer
  ax = -(Buf[0] << 8 | Buf[1]); //pitch
  ay = -(Buf[2] << 8 | Buf[3]); //roll
  az = Buf[4] << 8 | Buf[5];  

  // Gyroscope
  gy = -(Buf[8] << 8 | Buf[9]);
  gx = -(Buf[10] << 8 | Buf[11]);

///Filter axis
  ComplementaryFilter(gx,gy, ax,ay,az);

// Define new Axes
  aP = corrected_pitch;
  calc1 = wortel3_2*corrected_roll; //is used twice, only calculated once
  calc2 = 0.5*corrected_pitch;
  a1 = calc1+calc2;
  a2 = calc1-calc2;


  // Display values

  //Serial.print(ax);
  //Serial.print(ay);
  //Serial.print (a1);
  //Serial.println(a2);


//  currentTime = millis();                                      //get current time
//  elapsedTime = (double)(currentTime - previousTime);          //compute time elapsed from previous computation
//  previousTime = currentTime;                                  //remember current time
//  Serial.println(elapsedTime);
//  

  //PID's
  aP_output = computePID_pitch(aP, pitch_Setpoint);
  a1_output = computePID_a1(a1, a1_Setpoint);
  a2_output = computePID_a2(a2, a2_Setpoint);

//  aP_output = aP_output*abs(aP_output);
//  a1_output = a1_output*abs(a1_output);
//  a2_output = a2_output*abs(a2_output);
 // Serial.write(',');
 // Serial.println(aP_output);



  //Sum PID's and process in Motor Controller
  motor1Output = aP_output + a2_output;
  motor2Output = -aP_output + a1_output;
  motor3Output = (-a1_output - a2_output);

//print angles
  if (modus==DEMO){
    Serial.print(gx); //Serial.print(gx);
    Serial.print(",");
    Serial.print(ax);   ///ax
    Serial.print(",");
    Serial.println(corrected_pitch);
    delay(15);
  }
  else if (modus==NORMAL)
  {
    setMotors(motor1Output, motor2Output, motor3Output);
  }

                          //motor = keyboard output;
//  String readString;
//  while (Serial.available()) {
//    char c = Serial.read();  //gets one byte from serial buffer
//    readString += c; //makes the string readString
//    delay(2);  //slow looping to allow buffer to fill with next character
//  }
//
//  if (readString.length() >0) {
//      Serial.println(readString);//so you can see the captured string 
//      int motor_set = readString.toInt();  //convert readString into a number
//      Serial.println(motor_set);
//      setMotors(motor_set, -motor_set,0);
//  }

}

//PID for pitch control:
double error,error_a1,error_a2;
double lastError,lastError_a1,lastError_a2;
double rateErrorAvg, rateErrorAvg1,rateErrorAvg2;
double rateError, rateError_a1,rateError_a2;

double computePID_pitch(double inp, double Setpoint) {
  error = Setpoint - inp;                                      // determine error
  //cumError = error * elapsedTime;                             // compute integral
  rateError = (error - lastError) / elapsedTime;               // compute derivative
  rateErrorAvg = (0.1*rateErrorAvg+0.9*rateError);
  
  lastError = error;                                           //remember current error
  return (kp * error + kd * rateErrorAvg);         //ki * cumError + 
}

//PID for axis 1 control:
double computePID_a1(double inp_a1, double Setpoint_a1) { 
  error_a1 = Setpoint_a1 - inp_a1;                                      // determine error
//  cumError_a1 = error_a1 * elapsedTime;                             // compute integral
  rateError_a1 = (error_a1 - lastError_a1) / elapsedTime;               // compute derivative
  rateErrorAvg1 = (0.1*rateErrorAvg1+0.9*rateError_a1);
   
  lastError_a1 = error_a1;                                           //remember current error
  return (kp * error_a1  + kd * rateErrorAvg1);                   // + ki * cumError_a1
}
//PID for axis 2 control:
double computePID_a2(double inp_a2, double Setpoint_a2) {
  error_a2 = Setpoint_a2 - inp_a2;                                      // determine error
//  cumError_a2 = error_a2 * elapsedTime;                             // compute integral
  rateError_a2 = (error_a2 - lastError_a2) / elapsedTime;               // compute derivative
  rateErrorAvg2 = (0.1*rateErrorAvg2+0.9*rateError_a2);                //running average for small looptime

  lastError_a2 = error_a2;                                           //remember current error
  return (kp * error_a2 + kd * rateErrorAvg2);                    //ki * cumError_a2 
}
