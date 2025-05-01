// Self-Balancing Robot with Emotional Expressions (2x 8x8 MAX7219)
// Uses MPU6050 for motion sensing, PID for balancing, and LED matrices for eyes

#include <Wire.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LMotorController.h"
#include <LedControl.h>

// ===================== LED Setup =====================
LedControl lc = LedControl(12, 11, 10, 2); // DIN, CLK, CS, # of devices

void showEyes(byte leftEye[8], byte rightEye[8]) {
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, leftEye[i]);   // Left eye = Matrix 0
    lc.setRow(1, i, rightEye[i]);  // Right eye = Matrix 1
  }
}

// ========== Eye Expressions ==========
byte happyLeft[8] =     {B00100100,B00100100,B00000000,B00000000,B00000000,B01000010,B00111100,B00000000};
byte happyRight[8] =    {B00100100,B00100100,B00000000,B00000000,B00000000,B01000010,B00111100,B00000000};

byte angryLeft[8] =     {B01000000,B00100000,B00000000,B00000000,B00000000,B00111100,B01000010,B00000000};
byte angryRight[8] =    {B00000010,B00000100,B00000000,B00000000,B00000000,B00111100,B01000010,B00000000};

byte sleepLeft[8] =     {B00000000,B00000000,B00000000,B01111110,B01111110,B00000000,B00000000,B00000000};
byte sleepRight[8] =    {B00000000,B00000000,B00000000,B01111110,B01111110,B00000000,B00000000,B00000000};

byte surpriseLeft[8] =  {B00000000,B00111100,B01000010,B10000001,B10000001,B01000010,B00111100,B00000000};
byte surpriseRight[8] = {B00000000,B00111100,B01000010,B10000001,B10000001,B01000010,B00111100,B00000000};

byte loveLeft[8] =      {B00000000,B01100110,B11111111,B11111111,B11111111,B01111110,B00111100,B00011000};
byte loveRight[8] =     {B00000000,B01100110,B11111111,B11111111,B11111111,B01111110,B00111100,B00011000};

// ===================== MPU and PID Setup =====================
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double input, output;
double setpoint = 180;
double Kp = 22, Ki = 140, Kd = 1.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor controller
double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.6;
int ENA = 5, IN1 = 6, IN2 = 7, IN3 = 9, IN4 = 8, ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ===================== Expression Logic =====================
unsigned long startupTime;
bool wasFallen = false;
unsigned long hugStartTime = 0;
bool isHugging = false;

void updateEyesByMPU(float input, double output) {
  static float lastInput = 180;
  float delta = abs(input - lastInput);
  lastInput = input;

  // 1. LOVE — Bootup
  if (millis() - startupTime < 3000) {
    showEyes(loveLeft, loveRight);
    return;
  }

  // 2. SLEEPY — Laying flat
  if (input < 120 || input > 240) {
    showEyes(sleepLeft, sleepRight);
    wasFallen = true;
    isHugging = false;
    return;
  }

  // 3. LOVE — Picked up from fall
  if (wasFallen && abs(input - 180) < 5 && delta < 1) {
    showEyes(loveLeft, loveRight);
    wasFallen = false;
    return;
  }

  // 4. LOVE — Hug detected (still + upright)
  if (abs(input - 180) < 3 && abs(output) < 5 && delta < 0.3) {
    if (!isHugging) {
      hugStartTime = millis();
      isHugging = true;
    }
    if (millis() - hugStartTime > 2000) {
      showEyes(loveLeft, loveRight);
      return;
    }
  } else {
    isHugging = false;
  }

  // 5. SURPRISED — Fast motion
  if (delta > 15) {
    showEyes(surpriseLeft, surpriseRight);
    return;
  }

  // 6. ANGRY — Moderate shaking
  if (delta > 5) {
    showEyes(angryLeft, angryRight);
    return;
  }

  // 7. HAPPY — Moving and upright
  if (abs(output) > 20 && abs(input - 180) < 5) {
    showEyes(happyLeft, happyRight);
    return;
  }

  // Default fallback
  showEyes(happyLeft, happyRight);
}

// ===================== Arduino Setup =====================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  startupTime = millis();

  for (int i = 0; i < 2; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, 8);
    lc.clearDisplay(i);
  }

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ===================== Arduino Loop =====================
void loop() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();
    motorController.move(output, 20);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;

    pid.Compute();
    motorController.move(output, 20);
    updateEyesByMPU(input, output);
  }
}
