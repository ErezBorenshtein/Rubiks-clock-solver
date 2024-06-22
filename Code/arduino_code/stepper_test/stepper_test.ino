#include <AccelStepper.h>
#include <MultiStepper.h>

// Define the pins for the stepper motor

//UR
//#define STEP_PIN 2
//#define DIR_PIN 3

//DR
//#define STEP_PIN 4
//#define DIR_PIN 5

//DL
#define STEP_PIN 6
#define DIR_PIN 7

//UL
//#define DIR_PIN 8
//#define STEP_PIN 9

#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

// Create an instance of the AccelStepper class
AccelStepper stepper(1, STEP_PIN, DIR_PIN);
MultiStepper steppers;

void powerEnabler(){
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do{

    mode = digitalRead(LIMIT_PIN);

  }while(mode==1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}


void setup() {

    pinMode(LIMIT_PIN, INPUT_PULLUP);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
    powerEnabler();

    Serial.begin(115200);
    // Set the maximum speed and acceleration of the stepper motor
    stepper.setMaxSpeed(3900);
    stepper.setAcceleration(1000);
    stepper.setSpeed(100); 

    // Set the initial position of the stepper motor
    stepper.setCurrentPosition(0);

    steppers.addStepper(stepper);
}

void loop() {
    /*int steps = 200*2;
    // Rotate the stepper motor one revolution in one direction
    
    stepper.move(steps);
    long x =millis();
    stepper.runToPosition();
    Serial.println(millis()-x);
    // Pause for 1 second
    delay(1000);

    // Rotate the stepper motor one revolution in the opposite direction
    stepper.setCurrentPosition(0);
    

    // Pause for 1 second
    delay(1000);*/
    steppers.moveTo(new long[1]{1000});
    while(steppers.run()){
    }
    stepper.setCurrentPosition(0);
    delay(1000);

}


