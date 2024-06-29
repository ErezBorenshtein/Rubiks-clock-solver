/*void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Wait for serial port to open
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  Serial.println("ready");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data
    String inputString = Serial.readStringUntil('\n');
    
    // Print the received string
    Serial.println("Received: " + inputString);
  }
}*/

//"sketch": "Code\\arduino_code\\optimaized_code\\optimaized_code.ino"

#include <AccelStepper.h>
#include <MultiStepper.h>

// Define motor interface type
#define motorInterfaceType 1

// Create individual stepper instances
//AccelStepper stepper1(motorInterfaceType, 48, 49);
//AccelStepper stepper2(motorInterfaceType, 42, 43);

// Define limit and motor enable pins
#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

void powerEnabler() {
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do {
    mode = digitalRead(LIMIT_PIN);
  } while (mode == 1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void setup() {
  // Initialize limit and motor enable pins
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);
  
  // Attach interrupt to limit pin
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();

  // Initialize serial communication
  Serial.begin(115200);

  // Set maximum speed and acceleration for each stepper
  //stepper1.setMaxSpeed(13000);
  //stepper1.setAcceleration(5000000);
  //stepper1.setCurrentPosition(0);
  //stepper1.setMinPulseWidth(3);

  //stepper2.setMaxSpeed(13000);
  //stepper2.setAcceleration(5000000);
  //stepper2.setCurrentPosition(0);
}

/*void moveSteppers(long positions[]) {
  // Move each stepper to the target position
  stepper1.moveTo(positions[0]);
  //stepper2.moveTo(positions[1]);

  // Run both steppers until they reach their target positions
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    delay(1);
    //stepper2.run();
  }
}*/

const long turn_steps = 400 * 2;
const int step_dir[4] = {43, 49, 11, 7};
const int step_pul[4] = {42, 48, 12, 8};
//const int step_dir[4] = {11, 9, 14, 7};
//const int step_pul[4] = {12, 10, 15, 8};

void move_motor(long spd, long steps[]) {

  long maxSteps = 0;
  for (int i = 0; i < 4; i++) {
    bool hl = true;
    if (steps[i] > 0) hl = false;
    digitalWrite(step_dir[i], hl);
    steps[i] = abs(steps[i]);
    if(steps[i] > maxSteps) maxSteps = steps[i];
  }

  long avgTime = 1000000 * 60 / turn_steps / spd;
  long maxTime = 375;
  long slope = 10;
  long accel = min(maxSteps, max(0, (maxTime - avgTime) / slope));
  bool motorPulse = false;
  for (int i = 0; i < maxSteps; i++) {
    motorPulse = !motorPulse;
    
    for (int j = 0; j < 4; j++) {
      if (i < steps[j]) {
        digitalWrite(step_pul[j], motorPulse);
      }
    }
    
    long delay_time = avgTime;
    if (i < accel) delay_time = maxTime - slope * i;
    delayMicroseconds(delay_time);
  }
}

class ClockSteppers{
  public:
    const int speed = 800;
    const long turnSteps = 400 * 2;
    const int stepPul[4] = {42, 44, 46, 48};
    const int stepDir[4] = {43, 45, 47, 49};
    
    long currentPositions[4] = {0,0,0,0};

    void moveMotor(long steps[]) {

      long maxSteps = 0;
      for (int i = 0; i < 4; i++) {
        bool hl = true;
        if (steps[i] > 0) hl = false;
        digitalWrite(stepDir[i], hl);
        steps[i] = abs(steps[i]);
        if(steps[i] > maxSteps) maxSteps = steps[i];
        Serial.println("steps: "+String(steps[i]));
      }

      long avgTime = 1000000 * 60 / turnSteps / speed;
      long maxTime = 375;
      long slope = 10;
      long accel = min(maxSteps, max(0, (maxTime - avgTime) / slope));
      bool motorPulse = false;
      for (int i = 0; i < maxSteps; i++) {
        motorPulse = !motorPulse;
        
        for (int j = 0; j < 4; j++) {
          if (i < steps[j]) {
            
            digitalWrite(stepPul[j], motorPulse);
          }
        }
        
        long delay_time = avgTime;
        if (i < accel) delay_time = maxTime - slope * i;
        delayMicroseconds(delay_time);
      }
    }

    void moveTo(long toPositions[]){
      long steps[4];
      for(int i =0; i<4;i++){
        steps[i] = toPositions[i] - currentPositions[i];
        Serial.println("steps: "+String(steps[i]));
      }
      
      moveMotor(steps);

      for(int i =0; i<4;i++){
        currentPositions[i] = toPositions[i];
      }
    }
};

ClockSteppers clockSteppers;

void loop() {

  delay(1000);
//
  //moveSteppers(new long[2]{108, 0});
//
  //delay(1000);
//
  //moveSteppers(new long[2]{0, 0});
  //int x = millis();
  //move_motor(800,new long[4]{200, 200, 200, 200});
  //delay(10);
  //move_motor(800, new long[4]{-200, -200, -200, -200});
  //Serial.println(millis() - x);

  clockSteppers.moveTo(new long[4]{200, 200, 200, 200});
  delay(1000);
  clockSteppers.moveTo(new long[4]{-200, -200, -200, -200});
  

}
