#include <Wire.h>
#include <LedControl.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

LedControl lc = LedControl(12, 11, 10, 2); // DIN, CLK, CS, 2 devices
MPU6050 mpu;

// MPU variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float input;

// Expression flags
unsigned long startupTime;
bool wasFallen = false;
unsigned long hugStartTime = 0;
bool isHugging = false;

// Emotion timers
bool justFell = false;
unsigned long fallSadSince = 0;
bool showLoveNow = false;
unsigned long loveStartTime = 0;

// Eye patterns
//Happy

byte happyLeft[8] = {
  B00111100,
  B01111110,
  B11100111,
  B11100111,
  B11100111,
  B11111111,
  B01111110,
  B00111100
};

byte happyRight[8] = {
  B00111100,
  B01111110,
  B11100111,
  B11100111,
  B11100111,
  B11111111,
  B01111110,
  B00111100
};

//angry
byte angryLeft[8] = {
  B00001110,
  B00010001,
  B00100001,
  B01011001,
  B10011001,
  B10000001,
  B01000010,
  B00111100
};

byte angryRight[8] = {
  B00001110,
  B00010001,
  B00100001,
  B01011001,
  B10011001,
  B10000001,
  B01000010,
  B00111100
};

// Love 
byte loveLeft[8] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000
};

byte loveRight[8] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000
};

//sleep
byte sleepLeft[8] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B10000001,
  B01000010,
  B00111100,
  B00000000
};

byte sleepRight[8] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B10000001,
  B01000010,
  B00111100,
  B00000000
};

//sad

byte sadLeft[8] = {
  B01100000,
  B01001000,
  B01000100,
  B01011010,
  B00101010,
  B00010010,
  B00001010,
  B00000100
};

byte sadRight[8] = {
  B01100000,
  B01001000,
  B01000100,
  B01011010,
  B00101010,
  B00010010,
  B00001010,
  B00000100
};

void showEyes(byte left[8], byte right[8]) {
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, left[i]);
    lc.setRow(1, i, right[i]);
  }
}

void updateEyesByMPU(float input) {
  static float lastInput = 180;
  float delta = abs(input - lastInput);
  lastInput = input;

  // Show startup love for 3s
  if (millis() - startupTime < 3000) {
    showEyes(loveLeft, loveRight);
    return;
  }

  // Detect a fall
  if (delta > 25) {
    justFell = true;
    fallSadSince = millis();
    wasFallen = true;
    showLoveNow = false;
  }

  // If robot was sad but now upright and stable → show love
  if (wasFallen && abs(input - 180) < 5 && delta < 1) {
    if (!showLoveNow) {
      loveStartTime = millis();
      showLoveNow = true;
      justFell = false;
      wasFallen = false;
    }
  }

  // Show love for 4s after recovery
  if (showLoveNow && millis() - loveStartTime < 4000) {
    showEyes(loveLeft, loveRight);
    return;
  } else if (showLoveNow && millis() - loveStartTime >= 4000) {
    showLoveNow = false;
  }

  // Stay sad until robot is picked up or enough time has passed for sleep
  if (justFell && millis() - fallSadSince < 7000 && (input < 120 || input > 240)) {
    showEyes(sadLeft, sadRight);
    return;
  }

  // After 5s flat on the ground → sleep
  if ((input < 120 || input > 240) && millis() - fallSadSince >= 5000) {
    showEyes(sleepLeft, sleepRight);
    isHugging = false;
    return;
  }

  // Hug detection
  if (abs(input - 180) < 3 && delta < 0.3) {
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

  if (delta > 5) {
    showEyes(angryLeft, angryRight);
    return;
  }

  if (abs(input - 180) < 5) {
    showEyes(happyLeft, happyRight);
    return;
  }

  showEyes(happyLeft, happyRight);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
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
    attachInterrupt(0, [](){ mpuInterrupt = true; }, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("MPU init failed");
  }
}

volatile bool mpuInterrupt = false;

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
    updateEyesByMPU(input);
  }
}
