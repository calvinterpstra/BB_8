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
#define Motor2_in1_Pin  3
#define Motor2_in2_Pin  4
#define Motor3_PWM_Pin  11
#define Motor3_in1_Pin  12
#define Motor3_in2_Pin  10

#define  HC12_setPin 99999
SoftwareSerial HC12(1, 0); // HC-12 TX Pin, HC-12 RX Pin


//PID gains:

//double kp = 0.1;
//double ki = 0;// .00000002;
//double kd = 2.8;

//bigger legs
//double kp = 0.11;
//double kd = 4.5;

double kp = 0.3;
double kd = 5.2;

//motor_layout:
int motor1Output = 0;
int motor2Output = 0;
int motor3Output = 0;

uint8_t Buf[14];
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

  int motor_power = abs(motorPower);
  if (motor_power>5){ 
    if (motor_PWM_Pin==Motor1_PWM_Pin) motor_power += 155;  //offset
    else if (motor_PWM_Pin==Motor2_PWM_Pin) motor_power +=  145;
    else if (motor_PWM_Pin==Motor3_PWM_Pin) motor_power +=  145;   
  }
  if (motor_power>253) {motor_power =254; }

  analogWrite(motor_PWM_Pin, motor_power);
}

void setMotors(int motor1Power, int motor2Power, int motor3Power) {
//  int minimum_power = min(abs(motor1Power), min(abs(motor2Power),abs(motor3Power)));
//  if (abs(motor1Power) == abs(minimum_power)){motor1Power = 0;}
//  else if (abs(motor2Power) == abs(minimum_power)){motor2Power = 0;}
//  else if (abs(motor3Power) == abs(minimum_power)){motor3Power = 0;}

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

//void callback() {
//  intFlag = true;
////  digitalWrite(13, digitalRead(13) ^ 1);
//}

//void readHC12(){
//  while (HC12.available()) {             // If HC-12 has data
//    int incomingByte = HC12.read();
//    readBuffer = char(incomingByte);    // Add each byte to ReadBuffer string variable 
//  }
//}

void setup()   { 
  Wire.begin();
  // HC12.begin(9600);               // Open serial port to HC12
  Serial.begin(2000000);
  //pinMode(HC12_setPin, OUTPUT);
 // digitalWrite(HC12_setPin, HIGH);     // HC-12 normal mode

  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);                      // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);                      // Set gyroscope low pass filter at 5Hz

  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);        // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);                    // Set by pass mode for the magnetometers

  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);                        // Request continuous magnetometer measurements in 16 bits

//  pinMode(13, OUTPUT);
//  Timer1.initialize(10000);                                     // initialize timer1, and set a 1/2 second period
//  Timer1.attachInterrupt(callback);                             // attaches callback() as a timer overflow interrupt


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

  //set initial values
  corrected_pitch = -(Buf[0] << 8 | Buf[1]); //pitch
  corrected_roll = -(Buf[2] << 8 | Buf[3]); //roll
}

#define GYROSCOPE_SENSITIVITY 220 //           goeie = 130


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
//  while (!intFlag);
//  intFlag = false;

  // readHC12();
  // ::: Counter :::
  // Display data counter
  //  Serial.print (cpt++,DEC);
  //  Serial.print ("\t");

  // Read accelerometer and gyroscope

  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]); //pitch
  int16_t ay = -(Buf[2] << 8 | Buf[3]); //roll
  int16_t az = Buf[4] << 8 | Buf[5];  

  // Gyroscope
  int16_t gy = -(Buf[8] << 8 | Buf[9]);
  int16_t gx = -(Buf[10] << 8 | Buf[11]);

///Filter axis
  ComplementaryFilter(gx,gy, ax,ay,az);

//print angles
//  Serial.print(gx);
//  Serial.print(",");
//  Serial.print(ax);
//  Serial.print(",");
//  Serial.println(corrected_pitch);
//  delay(10);

#define wortel3_2 0.866025
// Define new Axes
  int16_t aP = corrected_pitch;
  int16_t calc1 = wortel3_2*corrected_roll; //is used twice, only calculated once
  int16_t calc2 = 0.5*corrected_pitch;
  int16_t a1 = calc1+calc2;
  int16_t a2 = calc1-calc2;


  // Display values

  //Serial.print(ax);
  //Serial.print(ay);
  //Serial.print (a1);
  //Serial.println(a2);


  currentTime = millis();                                      //get current time
  elapsedTime = (double)(currentTime - previousTime);          //compute time elapsed from previous computation
  previousTime = currentTime;                                  //remember current time
  //Serial.println(elapsedTime);
  

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

 // Serial.write(',');
 // Serial.println(aP_output);



  //Sum PID's and process in Motor Controller
  motor1Output = aP_output + a2_output;
  motor2Output = -aP_output + a1_output;
  motor3Output = (-a1_output - a2_output);

  setMotors(motor1Output, motor2Output, motor3Output);

  //motor = keyboard output
//  String readString;
//  while (Serial.available()) {
//    char c = Serial.read();  //gets one byte from serial buffer
//    readString += c; //makes the string readString
//    delay(2);  //slow looping to allow buffer to fill with next character
//  }
//
//  if (readString.length() >0) {
//      Serial.println(readString);//so you can see the captured string 
//    int motor_set = readString.toInt();  //convert readString into a number
//   Serial.println(motor_set);
//    setMotors(0, motor_set, motor_set);
//  }



}

//PID for pitch control:
double error,error_a1,error_a2;
double lastError,lastError_a1,lastError_a2;
//double cumError, cumError_a1,cumError_a2;
double rateError, rateError_a1,rateError_a2;

double computePID_pitch(double inp, double Setpoint) {
  error = Setpoint - inp;                                      // determine error
  //cumError = error * elapsedTime;                             // compute integral
  rateError = (error - lastError) / elapsedTime;               // compute derivative
  
  lastError = error;                                           //remember current error
  return (kp * error + kd * rateError);         //ki * cumError + 
}

//PID for axis 1 control:
double computePID_a1(double inp_a1, double Setpoint_a1) { 
  error_a1 = Setpoint_a1 - inp_a1;                                      // determine error
//  cumError_a1 = error_a1 * elapsedTime;                             // compute integral
  rateError_a1 = (error_a1 - lastError_a1) / elapsedTime;               // compute derivative
   
  lastError_a1 = error_a1;                                           //remember current error
  return (kp * error_a1  + kd * rateError_a1);                   // + ki * cumError_a1
}
//PID for axis 2 control:
double computePID_a2(double inp_a2, double Setpoint_a2) {
  error_a2 = Setpoint_a2 - inp_a2;                                      // determine error
//  cumError_a2 = error_a2 * elapsedTime;                             // compute integral
  rateError_a2 = (error_a2 - lastError_a2) / elapsedTime;               // compute derivative

  lastError_a2 = error_a2;                                           //remember current error
  return (kp * error_a2 + kd * rateError_a2);                    //ki * cumError_a2 
}
