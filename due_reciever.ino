/*
This code reads the time multiple PPM signals are HIGH and passes it through to input
*/
#include <Servo.h>
//TODO: download Timer1 library
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define SIGNAL_NEUTRAL 1400
//#define THROTTLE_INT_PORT 0 //unneeded because of PChangeInt

//Reciever input
#define THROTTLE_IN_PORT 12
#define RUDDER_IN_PORT 9
#define ELEVATOR_IN_PORT 10
#define AILERON_IN_PORT 11
#define AUX1_IN_PORT 7
#define AUX2_IN_PORT 8

//Arduino Output
#define THROTTLE_OUT_PORT 3
#define RUDDER_OUT_PORT 2
#define ELEVATOR_OUT_PORT 4
#define AILERON_OUT_PORT 5
#define AUX1_OUT_PORT 6

//interrupt flags, bitmask
#define THROTTLE_FLAG 1
#define RUDDER_FLAG 2
#define ELEVATOR_FLAG 4
#define AILERON_FLAG 8
#define AUX1_FLAG 16
#define AUX2_FLAG 32

//base multiplier for active correction
#define MULTIPLIER 5

//interrupt pin for gyro
#define MPU6050INTERRUPT_PIN 13

//shared vars for input and flags
volatile uint8_t sharedFlags = 0;
volatile uint16_t sharedThrottle = SIGNAL_NEUTRAL;
volatile uint16_t sharedRudder = SIGNAL_NEUTRAL;
volatile uint16_t sharedElevator = SIGNAL_NEUTRAL;
volatile uint16_t sharedAileron = SIGNAL_NEUTRAL;
volatile uint16_t sharedAux1 = SIGNAL_NEUTRAL;
volatile uint16_t sharedAux2 = SIGNAL_NEUTRAL;

//volatile vars for keeping track of time
volatile uint32_t throttleStart;
volatile uint32_t rudderStart;
volatile uint32_t elevatorStart;
volatile uint32_t aileronStart;
volatile uint32_t aux1Start;
volatile uint32_t aux2Start;

//target ypr
int16_t yawTarget;
int16_t pitchTarget;
int16_t rollTarget;

//error ypr
int16_t yawError = 0;
int16_t pitchError = 0;
int16_t rollError = 0;

//ypr input
int16_t yawInput = 0;
int16_t pitchInput = 0;
int16_t rollInput = 0;

//the outputs
Servo throttle, rudder, elevator, aileron, aux1;

//the gyro
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//I gains for active correction
double throttleIGain = 1;
double rudderIGain = 1;
double elevatorIGain = 1;
double aileronIGain = 1;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(THROTTLE_IN_PORT, calcThrottle, CHANGE);
  attachInterrupt(RUDDER_IN_PORT, calcRudder, CHANGE);
  attachInterrupt(ELEVATOR_IN_PORT, calcElevator, CHANGE);
  attachInterrupt(AILERON_IN_PORT, calcAileron, CHANGE);
  attachInterrupt(AUX1_IN_PORT, calcAux1, CHANGE);
  attachInterrupt(AUX2_IN_PORT, calcAux2, CHANGE);

  throttle.attach(THROTTLE_OUT_PORT);
  rudder.attach(RUDDER_OUT_PORT);
  elevator.attach(ELEVATOR_OUT_PORT);
  aileron.attach(AILERON_OUT_PORT);
  aux1.attach(AUX1_OUT_PORT);

  Wire.begin();
  Serial.begin(115200);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again*/

  delay(3000);

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
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 13)..."));
    attachInterrupt(MPU6050INTERRUPT_PIN, dmpDataReady, RISING);
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

  if (!dmpReady) return;
  getAccel();
  yawTarget = ypr[0];
  pitchTarget = ypr[1];
  rollTarget = ypr[2];
}

void loop() {
  // put your main code here, to run repeatedly:
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  static uint16_t throttleIn;
  static uint16_t rudderIn;
  static uint16_t elevatorIn;
  static uint16_t aileronIn;
  static uint16_t aux1In;
  static uint16_t aux2In;

  static uint16_t throttleProcessed;
  static uint16_t rudderProcessed;
  static uint16_t elevatorProcessed;
  static uint16_t aileronProcessed;
  static uint16_t aux1Processed;
  static uint16_t aux2Processed;

  static uint8_t flags;

  if (sharedFlags) {
    noInterrupts();
    flags = sharedFlags;

    if (flags & THROTTLE_FLAG) {
      throttleIn = sharedThrottle;
      //throttleProcessed = sharedThrottle;
    }

    if (flags & RUDDER_FLAG) {
      rudderIn = sharedRudder;
      //rudderProcessed = sharedRudder;
    }

    if (flags & ELEVATOR_FLAG) {
      elevatorIn = sharedElevator;
      //elevatorProcessed = sharedElevator;
    }

    if (flags & AILERON_FLAG) {
      aileronIn = sharedAileron;
      //aileronProcessed = sharedAileron;
    }

    if (flags & AUX1_FLAG) {
      aux1In = sharedAux1;
      //aux1Processed = sharedAux1;
    }

    if (flags & AUX2_FLAG) {
      aux2In = sharedAux2;
      //aux2Processed = sharedAux2;
    }

    sharedFlags = 0;
    interrupts();
  }

  getAccel();
  //PROCESSING
  if (aux2In < SIGNAL_NEUTRAL - 200) { //if processing...

    if (flags & THROTTLE_FLAG) {
      throttleProcessed = SIGNAL_NEUTRAL;
    }

    if (flags & RUDDER_FLAG) {
      rudderProcessed = SIGNAL_NEUTRAL;
      yawInput = map(rudderIn, 1096, 1900, -1 * MULTIPLIER, MULTIPLIER);
      yawTarget += yawInput;
      yawTarget = normalize(yawTarget);
      yawError = ypr[0] - yawTarget;
      rudderProcessed = ((-1 * MULTIPLIER * yawError) + rudderProcessed) * rudderIGain;
      if (abs(yawError) > 20) {
        rudderIGain += 0.05;
      } else {
        rudderIGain = 1;
      }
    }

    if (flags & ELEVATOR_FLAG) {
      elevatorProcessed = SIGNAL_NEUTRAL;
      pitchInput = map(elevatorIn, 1096, 1900, -1 * MULTIPLIER, MULTIPLIER);
      pitchTarget += pitchInput;
      pitchTarget = normalize(pitchTarget);
    }

    if (flags & AILERON_FLAG) {//unneeded
      rollInput = map(aileronIn, 1096, 1900, -1 * MULTIPLIER, MULTIPLIER);
      rollTarget = 0;
    }

    if (flags & AUX1_FLAG) {//unneeded
      if (false) {

      }
    }
  } else {
    throttleIGain = 1;
    rudderIGain = 1;
    elevatorIGain = 1;
    aileronIGain = 1;
  }
  //WRITING
  if (aux2In < SIGNAL_NEUTRAL - 200) { //if processing...
    if (flags & THROTTLE_FLAG) {
      if (throttle.readMicroseconds() != throttleProcessed) {
        throttle.writeMicroseconds(throttleProcessed);
      }
    }

    if (flags & RUDDER_FLAG) {
      if (rudder.readMicroseconds() != rudderProcessed) {
        rudder.writeMicroseconds(rudderProcessed);
      }
    }

    if (flags & ELEVATOR_FLAG) {
      if (elevator.readMicroseconds() != elevatorProcessed) {
        elevator.writeMicroseconds(elevatorProcessed);
      }
    }

    if (flags & AILERON_FLAG) {
      if (aileron.readMicroseconds() != aileronProcessed) {
        aileron.writeMicroseconds(aileronProcessed);
      }
    }

    if (flags & AUX1_FLAG) {
      if (aux1.readMicroseconds() != aux1Processed) {
        aux1.writeMicroseconds(aux1Processed);
      }
    }
  } else {//pass values through
    if (flags & THROTTLE_FLAG) {
      if (throttle.readMicroseconds() != throttleIn) {
        throttle.writeMicroseconds(throttleIn);
      }
    }

    if (flags & RUDDER_FLAG) {
      if (rudder.readMicroseconds() != rudderIn) {
        rudder.writeMicroseconds(rudderIn);
      }
    }

    if (flags & ELEVATOR_FLAG) {
      if (elevator.readMicroseconds() != elevatorIn) {
        elevator.writeMicroseconds(elevatorIn);
      }
    }

    if (flags & AILERON_FLAG) {
      if (aileron.readMicroseconds() != aileronIn) {
        aileron.writeMicroseconds(aileronIn);
      }
    }

    if (flags & AUX1_FLAG) {
      if (aux1.readMicroseconds() != aux1In) {
        aux1.writeMicroseconds(aux1In);
      }
    }
  }

}

void calcThrottle() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(THROTTLE_IN_PORT) == HIGH) {
    throttleStart = micros();
  }
  else {
    sharedThrottle = (uint32_t)(micros() - throttleStart);
    sharedFlags |= THROTTLE_FLAG;
  }
}

void calcRudder() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(RUDDER_IN_PORT) == HIGH) {
    rudderStart = micros();
  }
  else {
    sharedRudder = (uint32_t)(micros() - rudderStart);
    sharedFlags |= RUDDER_FLAG;
  }
}

void calcElevator() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(ELEVATOR_IN_PORT) == HIGH) {
    elevatorStart = micros();
  }
  else {
    sharedElevator = (uint32_t)(micros() - elevatorStart);
    sharedFlags |= ELEVATOR_FLAG;
  }
}

void calcAileron() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(AILERON_IN_PORT) == HIGH) {
    aileronStart = micros();
  }
  else {
    sharedAileron = (uint32_t)(micros() - aileronStart);
    sharedFlags |= AILERON_FLAG;
  }
}

void calcAux1() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(AUX1_IN_PORT) == HIGH) {
    aux1Start = micros();
  }
  else {
    sharedAux1 = (uint32_t)(micros() - aux1Start);
    sharedFlags |= AUX1_FLAG;
  }
}

void calcAux2() {
  //calculate the length of the HIGH PPM wave
  if (digitalRead(AUX2_IN_PORT) == HIGH) {
    aux2Start = micros();
  }
  else {
    sharedAux2 = (uint32_t)(micros() - aux2Start);
    sharedFlags |= AUX2_FLAG;
  }
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void getAccel() {
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

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
  }
}

int16_t normalize(int16_t norm) {
  if (norm > 360) {
    while (norm > 360) {
      norm -= 360;
    }
  } else if (norm < 0) {
    while (norm < 0) {
      norm += 360;
    }
  }

  return norm;
}

/*void getMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}

void calibrateMPU6050() {
  getMPU6050();
  AcXCal = AcX;
  AcYCal = AcY;
  AcZCal = AcZ;
  TmpCal = Tmp;
  GyXCal = GyX;
  GyYCal = GyY;
  GyZCal = GyZ;
  delay(100);
  getMPU6050();
  AcXCal = (AcX + AcXCal) / 2;
  AcYCal = (AcY + AcYCal) / 2;
  AcZCal = (AcZ + AcZCal) / 2;
  TmpCal = (Tmp + TmpCal) / 2;
  GyXCal = (GyX + GyXCal) / 2;
  GyYCal = (GyY + GyYCal) / 2;
  GyZCal = (GyZ + GyZCal) / 2;
  delay(100);
  getMPU6050();
  AcXCal = (AcX + AcXCal) / 2;
  AcYCal = (AcY + AcYCal) / 2;
  AcZCal = (AcZ + AcZCal) / 2;
  TmpCal = (Tmp + TmpCal) / 2;
  GyXCal = (GyX + GyXCal) / 2;
  GyYCal = (GyY + GyYCal) / 2;
  GyZCal = (GyZ + GyZCal) / 2;
  Serial.print("AcXCal = "); Serial.print(AcXCal);
  Serial.print(" | AcYCal = "); Serial.print(AcYCal);
  Serial.print(" | AcZCal = "); Serial.print(AcZCal);
  Serial.print(" | TmpCal = "); Serial.print(TmpCal / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyXCal = "); Serial.print(GyXCal);
  Serial.print(" | GyYCal = "); Serial.print(GyYCal);
  Serial.print(" | GyZCal = "); Serial.println(GyZCal);
}*/
