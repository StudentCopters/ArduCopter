/*
This code reads the time multiple PPM signals are HIGH and outputs it to Serial
*/
#include <Servo.h>
//TODO: download Timer1 library
#include <PinChangeInt.h>

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

void setup() {
  // put your setup code here, to run once:
  PCintPort::attachInterrupt(THROTTLE_IN_PORT, calcThrottle, CHANGE);
  PCintPort::attachInterrupt(RUDDER_IN_PORT, calcRudder, CHANGE);
  PCintPort::attachInterrupt(ELEVATOR_IN_PORT, calcElevator, CHANGE);
  PCintPort::attachInterrupt(AILERON_IN_PORT, calcAileron, CHANGE);
  PCintPort::attachInterrupt(AUX1_IN_PORT, calcAux1, CHANGE);
  PCintPort::attachInterrupt(AUX2_IN_PORT, calcAux2, CHANGE);
  
  throttle.attach(THROTTLE_OUT_PORT);
  rudder.attach(RUDDER_OUT_PORT);
  elevator.attach(ELEVATOR_OUT_PORT);
  aileron.attach(AILERON_OUT_PORT);
  aux1.attach(AUX1_OUT_PORT);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t throttleIn;
  static uint16_t rudderIn;
  static uint16_t elevatorIn;
  static uint16_t aileronIn;
  static uint16_t aux1In;
  static uint16_t aux2In;
  static uint8_t flags;
  
  if(sharedFlags){
    noInterrupts();
    flags = sharedFlags;
    
    if(flags & THROTTLE_FLAG){
      throttleIn = sharedThrottle;
    }
    
    if(flags & RUDDER_FLAG){
      rudderIn = sharedRudder;
    }
    
    if(flags & ELEVATOR_FLAG){
      elevatorIn = sharedElevator;
    }
    
    if(flags & AILERON_FLAG){
      aileronIn = sharedAileron;
    }
    
    if(flags & AUX1_FLAG){
      aux1In = sharedAux1;
    }
    
    if(flags & AUX2_FLAG){
      aux2In = sharedAux2;
    }
    
    sharedFlags = 0;
    interrupts();
  }
  
  //PROCESSING
  
  if(flags & THROTTLE_FLAG){
    if(throttle.readMicroseconds() != throttleIn){
      throttle.writeMicroseconds(throttleIn);
    }
  }
  
  if(flags & RUDDER_FLAG){
    if(rudder.readMicroseconds() != rudderIn){
      rudder.writeMicroseconds(rudderIn);
    }
  }
  
  if(flags & ELEVATOR_FLAG){
    if(elevator.readMicroseconds() != elevatorIn){
      elevator.writeMicroseconds(elevatorIn);
    }
  }
  
  if(flags & AILERON_FLAG){
    if(aileron.readMicroseconds() != aileronIn){
      aileron.writeMicroseconds(aileronIn);
    }
  }
  
  if(flags & AUX1_FLAG){
    if(aux1.readMicroseconds() != aux1In){
      aux1.writeMicroseconds(aux1In);
    }
  }
}

