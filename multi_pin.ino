/*
This code reads the time multiple PPM signals are HIGH and outputs it to Serial
*/
#include <Servo.h>
//TODO: download Timer1 library
//TODO: use Due for interrupts
#include <Wire.h>


#define SIGNAL_NEUTRAL 1400
//#define THROTTLE_INT_PORT 0 //unneeded because of PChangeInt

#define THROTTLE_IN_PORT 2
#define RUDDER_IN_PORT 3
#define ELEVATOR_IN_PORT 4
#define AILERON_IN_PORT 5
#define AUX1_IN_PORT 6
#define AUX2_IN_PORT 7

#define THROTTLE_OUT_PORT 8
#define RUDDER_OUT_PORT 9
#define ELEVATOR_OUT_PORT 10
#define AILERON_OUT_PORT 11
#define AUX1_OUT_PORT 12

#define THROTTLE_FLAG 1
#define RUDDER_FLAG 2
#define ELEVATOR_FLAG 4
#define AILERON_FLAG 8
#define AUX1_FLAG 16
#define AUX2_FLAG 32

#define MULTIPLIER 50

volatile uint8_t sharedFlags = 0;
volatile uint16_t sharedThrottle = SIGNAL_NEUTRAL;
volatile uint16_t sharedRudder = SIGNAL_NEUTRAL;
volatile uint16_t sharedElevator = SIGNAL_NEUTRAL;
volatile uint16_t sharedAileron = SIGNAL_NEUTRAL;
volatile uint16_t sharedAux1 = SIGNAL_NEUTRAL;
volatile uint16_t sharedAux2 = SIGNAL_NEUTRAL;

volatile uint32_t throttleStart;
volatile uint32_t rudderStart;
volatile uint32_t elevatorStart;
volatile uint32_t aileronStart;
volatile uint32_t aux1Start;
volatile uint32_t aux2Start;

Servo throttle, rudder, elevator, aileron, aux1;

const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t AcXCal, AcYCal, AcZCal, TmpCal, GyXCal, GyYCal, GyZCal;

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
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(9600);

  calibrateMPU6050();

  Serial.println("Accel and Gyro Calibrated");
}

void loop() {
  // put your main code here, to run repeatedly:
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

  //PROCESSING
  if (aux2In > SIGNAL_NEUTRAL) {//if processing...
    getMPU6050();
    if (flags & THROTTLE_FLAG) {
      if (AcZ > AcZCal + 200 && MULTIPLIER * throttleIGain < 500) {//if FALLING
        throttleProcessed = SIGNAL_NEUTRAL + MULTIPLIER * throttleIGain;
        throttleIGain += 0.1;
      } else if (AcZ < AcZCal - 200 && MULTIPLIER * throttleIGain < 500) {//if RISING
        throttleProcessed = SIGNAL_NEUTRAL - MULTIPLIER * throttleIGain;
        throttleIGain += 0.1;
      } else {//if STABLE
        throttleIGain = 1;
      }
    }

    if (flags & RUDDER_FLAG) {
      if (false) {

      }
    }

    if (flags & ELEVATOR_FLAG) {
      if (false) {

      }
    }

    if (flags & AILERON_FLAG) {
      if (false) {

      }
    }

    if (flags & AUX1_FLAG) {
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
  if (aux2In > SIGNAL_NEUTRAL) {//if processing...
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

void getMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
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
}
