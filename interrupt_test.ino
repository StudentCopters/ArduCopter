/*
This code reads the time a PPM signal is HIGH and outputs it to Serial
*/
#import <Servo.h>
//TODO: download Timer1 library
//TODO: download PCint lib

#define SIGNAL_NEUTRAL 1400
#define SIGNAL_INT_PORT 0
#define SIGNAL_PORT 2

volatile boolean isNewSignal = false;
volatile int signalIn = SIGNAL_NEUTRAL;
volatile unsigned long signalStart = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(SIGNAL_INT_PORT, calcInput, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(isNewSignal){
    Serial.print(signalIn);
    isNewSignal = false;
  }
}

void calcInput(){
  //calculate the length of the HIGH PPM wave
  if(digitalRead(SIGNAL_PORT) == HIGH){
    signalStart = micros();
  }
  else{
    if(signalStart && (isNewSignal == false)){
      signalIn = (int)(micros()-signalStart);
      isNewSignal = true;
    }
  }
}
