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
#define MAIN_SKETCH
#include "WriteInstinct/OpenCat.h"
#include "commands.h"

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

char token;
char lastToken;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
byte hold = 0;
int8_t offsetLR = 0;
bool checkGyro = true;
int8_t skipGyro = 2;

#define COUNT_DOWN 60

uint8_t timer = 0;
//#define SKIP 1
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstMotionJoint;
byte jointIdx = 0;

//bool Xconfig = false;


unsigned long usedTime = 0;
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

int16_t motorCommand[DOF];

void setMotorAngle(){
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

void pushMotorAngle(){
  int16_t index = stackPopToNative();
  stackPush((item)motorCommand[index]);
}

void failsafe(){
  shutServos();
  for(int i = 0; i < DOF; i++){
    motorCommand[i] = 256;
  }
}

void shutServo(int i){
  pwm.setPWM(i, 0, 4096);
}

void shutServoCommand() {
  int16_t index = stackPopToNative();
  shutServo(index);
}

void setup() {
  pinMode(BUZZER, OUTPUT);
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
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  PTLF("\n* Start *");
  PTLF("Initialize I2C");
  PTLF("Connect MPU6050");
  mpu.initialize();
  //do
  {
    delay(500);
    // verify connection
    PTLF("Test connection");
    PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.
  } //while (!mpu.testConnection());

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

    //meow();
    strcpy(lastCmd, "rest");
    motion.loadBySkillName(lastCmd);
    for (int8_t i = DOF - 1; i >= 0; i--) {
      pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      //PTL(SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i] * rotationDirection(i) );
      calibratedPWM(i, motion.dutyAngles[i]);
      failsafe();
      delay(20);
    }
    randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
    shutServos();
    token = T_REST;
  }
  beep(30);

  pinMode(BATT, INPUT);
  pinMode(BUZZER, OUTPUT);
  //meow();
  initCommands();
}


void initCommands(){
  addNativeFun("m", setMotorAngle);
  addNativeFun("d", failsafe);
  addNativeFun("g", pushMotorAngle);
  addNativeFun("s", shutServoCommand);
}

void printHeader(){
  PTL("-- ");
}

void printVoltage(float voltage){
  PT("b ");
  PTL(voltage);
}

void printGravity(){
  PT("g ");
  PT(gravity.x);
  PT(" ");
  PT(gravity.y);
  PT(" ");
  PTL(gravity.z);
}

void printIR(){
  //Rather than try to decode on-device, just plonk the IR code directly out on the serial line.
  if (irrecv.decode(&results)){
    PT("i ");
    PTL(results.value);
    irrecv.resume();
  }
}

void printUltrasound(){
  //Nothing yet.  Need to figure out how we do this; a 50ms resolution on the TTL signal from the
  //hc-sr04 gives an 8m range, which isn't useful.
}

void printStartTime(unsigned long ms){
  PT("t ");
  PTL(ms);
}

void readSerial(){
  while (Serial.available() > 0){
    String line = Serial.readStringUntil('\n');
    PTL(line);
    inbuf = line.c_str();
    bufLen = line.length();
    interpret();
  }
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

void driveBeeper(long beepTimeMS) {
  if (beepTimeMS < 0) { return; }
  delay(beepTimeMS);
}

//We aim to go round the loop() once every 50ms.  That means we're sending 20 updates per second, unless
//we decide that's too bonkers and intentionally skip some data on some passes.
//Quick maths: 57600bps / 20 = 2880 byte budget.
#define TICK_MS 50
void loop() {
  unsigned long loopStart = millis();
  printHeader();
  printStartTime(loopStart);

  printVoltage(analogRead(BATT));

  getYPR();
  printGravity();
  printIR();
  printUltrasound();

  readSerial();
  driveMotors();

  long toDelayMS = TICK_MS - (millis() - loopStart);
  driveBeeper(toDelayMS);
}
