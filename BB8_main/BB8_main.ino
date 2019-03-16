#include <Wire.h>
#include <TimerOne.h>

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

#define Motor1_PWM_Pin  5
#define Motor1_in1_Pin  6
#define Motor1_in2_Pin  7
#define Motor2_PWM_Pin  3
#define Motor2_in1_Pin  8
#define Motor2_in2_Pin  9
#define Motor3_PWM_Pin  11
#define Motor3_in1_Pin  10
#define Motor3_in2_Pin  12



// ---------------------------------------LAYOUT----------------------------------------------:

// volatile int motorPower1;
// volatile int motorPower2;
// volatile int motorPower3;
// volatile byte count=0;

void setMotors(int motor1Power, int motor2Power, int motor3Power) {
  if(motor1Power >= 0) {
    analogWrite(Motor1_PWM_Pin, motor1Power);
    digitalWrite(Motor1_in1_Pin, LOW);
    digitalWrite(Motor1_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor1_PWM_Pin, -motor1Power);
    digitalWrite(Motor1_in1_Pin, HIGH);
    digitalWrite(Motor1_in2_Pin, LOW);

  }
  if(motor1Power >= 0) {
  analogWrite(Motor2_PWM_Pin, motor2Power);
    digitalWrite(Motor2_in1_Pin, LOW);
   digitalWrite(Motor2_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor2_PWM_Pin, -motor2Power);
    digitalWrite(Motor2_in1_Pin, LOW);
    digitalWrite(Motor2_in2_Pin, HIGH);
  }
  if(motor1Power >= 0) {
    analogWrite(Motor3_PWM_Pin, motor3Power);
    digitalWrite(Motor3_in1_Pin, HIGH);
    digitalWrite(Motor3_in2_Pin, LOW);
  }
  else {
    analogWrite(Motor3_PWM_Pin, -motor3Power);
    digitalWrite(Motor3_in1_Pin, HIGH);
    digitalWrite(Motor3_in2_Pin, LOW);
  }
}

// ---------------------------------------PID----------------------------------------------:
double kp = 0.1;
double ki = 0.5;
double kd = 0.2;
double Setpoint = 0;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, doutput, setPoint;
double cumError, rateError;

// ------------------------------------------MPU----------------------------------------------:

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}



// Initial time
long int ti;
volatile bool intFlag=false;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();

  // ---------------------------------------LAYOUT----------------------------------------------:
  
  // set the motor control and PWM pins to output mode
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



// Counter
long int cpt=0;

void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
void loop()
{

  
  while (!intFlag);
  intFlag=false;
  
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
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
    // Display values
  
  // Accelerometer
  //Serial.print (ax); 
//  Serial.print ("\t");
//  Serial.print (ay,DEC);
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
  input = map(-(Buf[0]<<8 | Buf[1]), -8500, 8500, -1023, 1023);                //read from rotary encoder connected to A0
  output = map(computePID(input), -1000000, 1000000, -255, 255);
  
  if(output >= 255){
    output = 255;
  }
  if(output <= -255){
    output = -255;
  }

  //MotorControl
  setMotors(output, output, output);
  Serial.print(input);
  Serial.print("\t");
  Serial.print(output);
  Serial.println("");
}
 
double computePID(double inp){     
        currentTime = millis();                                      //get current time
        elapsedTime = (double)(currentTime - previousTime);          //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                      // determine error
        cumError += error * elapsedTime;                             // compute integral
        rateError = (error - lastError)/elapsedTime;                 // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError; 
        
        lastError = error;                                           //remember current error
        previousTime = currentTime;                                  //remember current time
 
        return out;                                                  //have function return the PID output
}






