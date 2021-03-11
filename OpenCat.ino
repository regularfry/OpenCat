/* Main Arduino sketch for OpenCat, the bionic quadruped walking robot.
   Updates should be posted on GitHub: https://github.com/PetoiCamp/OpenCat

   Rongzhong Li
   Jan.3rd, 2021
   Copyright (c) 2021 Petoi LLC.

   This sketch may also includes others' codes under MIT or other open source liscence.
   Check those liscences in corresponding module test folders.
   Feel free to contact us if you find any missing references.

   The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include "protothreads.h"
#define MAIN_SKETCH
#include "WriteInstinct/OpenCat.h"
#include "interpreter.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128

//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using (1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may be broken
// and the reading will be unpredictable. it should be replaced with previous reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLag[HISTORY][3];

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[PACKET_SIZE]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

int16_t motorCommand[DOF];

#define ULTRA_SOUND
#define ULTRA_VCC 8
#define TRIGGER 9
#define ECHO 10
#define LONGEST_DISTANCE 200 //cm; this sets the max delay for the ultrasound

float ultraDistance;

// Serial readuntil timeout. Set to -1 to disable the timeout and
// hand loop control entirely over to the host.
#define SERIAL_TIMEOUT_MS -1

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
#include <IRremote.h>
/*-----( Declare objects )-----*/
IRrecv irrecv(IR_RECIEVER);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'

void getFIFO() {//get FIFO only without further processing
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
}

void getYPR() {//get YPR angles from FIFO data, takes time
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) ;
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    while(fifoCount < packetSize) { fifoCount = mpu.getFIFOCount(); }
    while(fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    mpu.resetFIFO();
    /*
    PTL("a");
    int result = -1;
    while(1 != result){
      PTL(result);
      result = mpu.GetCurrentFIFOPacket(fifoBuffer, packetSize);
    }
    PTL("b");
    mpu.resetFIFO();
    PTL("c");*/
#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#ifdef MPU_YAW180
      ypr[2] = -ypr[2];
      ypr[1] = -ypr[1];

#endif
#endif
      for (byte g = 1; g < 3; g++)
        ypr[g] *= degPerRad;        //ypr converted to degree

      // overflow is detected after the ypr is read. it's necessary to keep a lag record of previous reading.  -- RzLi --
#ifdef FIX_OVERFLOW
      for (byte g = 1; g < 3; g++) {
        yprLag[lag][g] = ypr[g];
        ypr[g] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] ;
      }
      lag = (lag + 1) % HISTORY;
#endif

#ifdef DEVELOPER
      PT(ypr[0]);
      PTF("\t");
      PT(ypr[1]);
      PTF("\t");
      PTL(ypr[2]);
#endif

  }
}


void ultraSetup(){
 pinMode(ULTRA_VCC, OUTPUT);
 digitalWrite(ULTRA_VCC, HIGH);
 pinMode(TRIGGER, OUTPUT);
 pinMode(ECHO, INPUT);
}

pt ptUltra;
int ultraThread(struct pt* pt) {
    static int duration = 0;
    static float distance = 0.0;
    static float farTime =  LONGEST_DISTANCE*2/0.034;


    PT_BEGIN(pt);

    // Loop forever
    for(;;) {
        digitalWrite(TRIGGER, LOW);   // turn the LED on (HIGH is the voltage level)
        PT_SLEEP(pt, 2e-3);
        digitalWrite(TRIGGER, HIGH);    // turn the LED off by making the voltage LOW
        PT_SLEEP(pt, 10e-3);
        digitalWrite(TRIGGER, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(ECHO, HIGH, farTime);

        // Calculating the distance
        ultraDistance = duration * 0.034 / 2; // 10^-6 * 34000 cm/s
        PT_SLEEP(pt, 100);
    }

    PT_END(pt);
}

pt ptGyro;
int gyroThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;) {
    getYPR();
    PT_SLEEP(pt, 50);
  }
  PT_END(pt);
}

void c_setMotorAngle(){
  //(n n -- ) -- "1 2 m" will set motor 2's angle to 1 degree.
  // These angles are read in as int16_t's, but they end up passed to calibratedPWM as floats.
  // This means that we've got a lot of wasted resolution at the moment: we can only move in
  // increments of a degree at a time, and only values -90..90 are
  // useful.
  // Passing values outside -128..127 will shut the motor down.
  int16_t index = stackPopToNative();
  int16_t angle = stackPopToNative();
  if(index < DOF){
    motorCommand[index] = angle;
  } else {
    PT("e No such motor: "); PTL(index);
  }
}

void c_setLegAngles(){
  int16_t index = stackPopToNative();

  stackPush((item)(index + 12));
  c_setMotorAngle();

  stackPush((item)(index + 8));
  c_setMotorAngle();
}

void c_pose(){
  for(int16_t i = 15; i >= 0; i--){
    stackPush((item)i);
    c_setMotorAngle();
  }
}

void c_legPose(){
  for(int16_t i = 15; i >= 8; i--){
    stackPush((item)i);
    c_setMotorAngle();
  }
}

void c_pushMotorAngle(){
  int16_t index = stackPopToNative();
  stackPush((item)motorCommand[index]);
}

void c_failsafe(){
  shutServos();
  for(int i = 0; i < DOF; i++){
    motorCommand[i] = 256;
  }
}

void shutServo(int i){
  pwm.setPWM(i, 0, 4096);
}

void c_shutServo() {
  int16_t index = stackPopToNative();
  shutServo(index);
}

void c_beep() {
  int16_t durationMS = stackPopToNative();
  int16_t note = stackPopToNative();
  beep(note, durationMS);
}

void c_sleep(){
  int16_t durationMS = stackPopToNative();
  delay(durationMS);
}

void c_printState(item*){
  printTimestamp();
  printVoltage(analogRead(BATT));
  printGravity();
  printIR();
  printUltrasound();
  printMotors();
  printHeader();
}



void initCommands(){
  addNativeFun("m", c_setMotorAngle);
  addNativeFun("d", c_failsafe);
  addNativeFun("g", c_pushMotorAngle);
  addNativeFun("s", c_shutServo);
  addNativeFun("p", c_printState);
  addNativeFun("beep", c_beep);
  addNativeFun("sleep", c_sleep);
  addNativeFun("leg", c_setLegAngles);
  addNativeFun("pose", c_pose);
  addNativeFun("legpose", c_legPose);
}

void printHeader(){
  PT("-- ");
}

void printVoltage(float voltage){
  PT("b ");
  PT(voltage);
  PTL();
}

void printGravity(){
  PT("g ");
  PT(gravity.x);
  PT(" ");
  PT(gravity.y);
  PT(" ");
  PT(gravity.z);
  PTL();
}

void printIR(){
  //Rather than try to decode on-device, just plonk the IR code directly out on the serial line.
  if (irrecv.decode(&results)){
    PT("i ");
    PT(results.value);
    PTL();
    irrecv.resume();
  }
}

void printUltrasound(){
  PT("u ");
  PT(ultraDistance);
  PTL();
}

void printTime(unsigned long ms){
  PT("t ");
  PT(ms);
  PTL();
}

void printTimestamp(){
  printTime(millis());
}

void printMotors(){
  PT("m ");
  for(int i = 0; i < DOF; i++){
    PT(motorCommand[i]);
    PT(" ");
  }
  PTL();
}

void prompt(){
  PT("> ");
}

void readSerial(){
  char *serial_c_str;
  size_t len;

  do {
    String line = Serial.readStringUntil('\n');
    PTL();
    serial_c_str = line.c_str();
    len = line.length();
    if(len > 0){
      interpret(serial_c_str, len);
      PTL();
      prompt();
    }
  } while (Serial.available() > 0);
}

void driveMotors(){
  for (int i = 0; i < DOF; i++){
    int16_t value = motorCommand[i];
    if (value < -128 || value >= 128){
      shutServo(i);
    } else {
      calibratedPWM(i, value);
    }
  }
}

pt ptSerial;
int serialThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;){
    if (Serial.available() > 0) {
      readSerial();
      driveMotors();
    }
    PT_SLEEP(pt, 1);
  }
  PT_END(pt);
}

void setup() {
  pinMode(BUZZER, OUTPUT);
  ultraSetup();
#ifdef PIXEL_PIN
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(3, pixels.Color(0, 0, LIT_ON));

  pixels.show();   // Send the updated pixel colors to the hardware.
#endif
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  PTLF("\n* Start *");
  PTLF("Initialize I2C");
  PTLF("Connect MPU6050");
  mpu.initialize();
  delay(500);
  // verify connection
  PTLF("Test connection");
  PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.

  // load and configure the DMP
  do {
    PTLF("Initialize DMP");
    devStatus = mpu.dmpInitialize();
    delay(500);
    // supply your own gyro offsets here, scaled for min sensitivity

    for (byte i = 0; i < 4; i++) {
      PT(EEPROMReadInt(MPUCALIB + 4 + i * 2));
      PT(" ");
    }
    PTL();
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      PTLF("Enable DMP");
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      PTLF("Enable interrupt");
      attachInterrupt(INTERRUPT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      PTLF("DMP ready!");
      //dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);

  //opening music
#if WALKING_DOF == 8
  playMelody(MELODY);
#endif

  //IR
  {
    //PTLF("IR Receiver Button Decode");
    irrecv.enableIRIn(); // Start the receiver
  }

  assignSkillAddressToOnboardEeprom();
  PTL();

  // servo
  { pwm.begin();

    pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
    delay(200);

    motion.loadBySkillName("rest");
    for (int8_t i = DOF - 1; i >= 0; i--) {
      pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      //PTL(SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i] * rotationDirection(i) );
      calibratedPWM(i, motion.dutyAngles[i]);
      delay(20);
    }
    randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
    c_failsafe();
  }
  beep(30);

  pinMode(BATT, INPUT);
  pinMode(BUZZER, OUTPUT);
  initCommands();
  PT_INIT(&ptUltra);
  PT_INIT(&ptGyro);
  PT_INIT(&ptSerial);
  prompt();
}

void loop() {
  PT_SCHEDULE(ultraThread(&ptUltra));
  PT_SCHEDULE(gyroThread(&ptGyro));
  PT_SCHEDULE(serialThread(&ptSerial));
}
