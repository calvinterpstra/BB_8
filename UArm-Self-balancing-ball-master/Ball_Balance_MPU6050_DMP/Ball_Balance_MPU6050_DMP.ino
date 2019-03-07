#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define Motor1_PWM_Pin  2
#define Motor1_in1_Pin  12
#define Motor1_in2_Pin  13
#define Motor2_PWM_Pin  5
#define Motor2_in1_Pin  4
#define Motor2_in2_Pin  7
#define Motor3_PWM_Pin  3
#define Motor3_in1_Pin  6
#define Motor3_in2_Pin  2

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle 0

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile int motorPower1;
volatile int motorPower2;
volatile int motorPower3;
volatile float currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

void setMotors(int Motor1_Speed, int Motor2_Speed, int Motor3_Speed) {
  if(Motor1_Speed >= 0) {
    analogWrite(Motor1_PWM_Pin, Motor1_Speed);
    digitalWrite(Motor1_in1_Pin, LOW);
    digitalWrite(Motor1_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor1_PWM_Pin, 255 + Motor1_Speed);
    digitalWrite(Motor1_in1_Pin, LOW);
    digitalWrite(Motor1_in2_Pin, HIGH);
  }
  if(Motor2_Speed >= 0) {
    analogWrite(Motor2_PWM_Pin, Motor2_Speed);
    digitalWrite(Motor2_in1_Pin, LOW);
    digitalWrite(Motor2_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor2_PWM_Pin, 255 + Motor2_Speed);
    digitalWrite(Motor2_in1_Pin, LOW);
    digitalWrite(Motor2_in2_Pin, HIGH);
  }
  if(Motor1_Speed >= 0) {
    analogWrite(Motor3_PWM_Pin, Motor3_Speed);
    digitalWrite(Motor3_in1_Pin, LOW);
    digitalWrite(Motor3_in2_Pin, HIGH);
  }
  else {
    analogWrite(Motor3_PWM_Pin, 255 + Motor3_Speed);
    digitalWrite(Motor3_in1_Pin, LOW);
    digitalWrite(Motor3_in2_Pin, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  //MPU
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
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

  // initialize PID sampling loop
  init_PID();
}

void loop() {
  //MPU
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
    }

  // set motor power after constraining it
  motorPower1 = constrain(motorPower1, -255, 255);
  motorPower2 = constrain(motorPower1, -255, 255);
  motorPower3 = constrain(motorPower1, -255, 255);
  setMotors(motorPower1, motorPower2, motorPower3);

  //PID loop
  currentAngle = ypr[0];
//  error = currentAngle - targetAngle;
//  errorSum = errorSum + error;  
//  errorSum = constrain(errorSum, -300, 300);
//  //calculate output from P, I and D values
//  motorPower1 = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
//  prevAngle = currentAngle;
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{

  // calculate the angle of inclination
  //currentAngle = ypr[0];
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower1 = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
}




