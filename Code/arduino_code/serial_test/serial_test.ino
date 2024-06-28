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
const int step_dir[4] = {43, 14, 11, 7};
const int step_pul[4] = {42, 15, 12, 8};
//const int step_dir[4] = {11, 9, 14, 7};
//const int step_pul[4] = {12, 10, 15, 8};

void move_motor(long spd, long deg1, long deg2, int motor0, int motor1, int motor2, int motor3) {
  bool hl1 = true;
  bool hl2 = true;
  if (deg1 > 0) hl1 = false;
  if (deg2 > 0) hl2 = false;
  if (deg1) {
    //Serial.println("hl1: " +(String)hl1);
    if (motor0) digitalWrite(step_dir[0], hl1);
    if (motor1) digitalWrite(step_dir[1], hl1);
    if (motor2) digitalWrite(step_dir[2], hl1);
    if (motor3) digitalWrite(step_dir[3], hl1);
  }
  if (deg2) {
    if (!motor0) digitalWrite(step_dir[0], hl2);
    if (!motor1) digitalWrite(step_dir[1], hl2);
    if (!motor2) digitalWrite(step_dir[2], hl2);
    if (!motor3) digitalWrite(step_dir[3], hl2);
  }
  long steps1 = abs(deg1) * turn_steps / 360;
  long steps2 = abs(deg2) * turn_steps / 360;
  long avg_time = 1000000 * 60 / turn_steps / spd;
  long max_time = 375;
  long slope = 10;
  long accel = min(max(steps1, steps2), max(0, (max_time - avg_time) / slope));
  bool motor_hl = false;
  for (int i = 0; i < max(steps1, steps2); i++) {
    motor_hl = !motor_hl;
    if (i < steps1) {
      if (motor0) digitalWrite(step_pul[0], motor_hl);
      if (motor1) digitalWrite(step_pul[1], motor_hl);
      if (motor2) digitalWrite(step_pul[2], motor_hl);
      if (motor3) digitalWrite(step_pul[3], motor_hl);
    }
    if (i < steps2) {
      if (!motor0) digitalWrite(step_pul[0], motor_hl);
      if (!motor1) digitalWrite(step_pul[1], motor_hl);
      if (!motor2) digitalWrite(step_pul[2], motor_hl);
      if (!motor3) digitalWrite(step_pul[3], motor_hl);
    }
    long delay_time = avg_time;
    if (i < accel) delay_time = max_time - slope * i;
    delayMicroseconds(delay_time);
  }
  
}


void loop() {

  delay(1000);
//
  //moveSteppers(new long[2]{108, 0});
//
  //delay(1000);
//
  //moveSteppers(new long[2]{0, 0});
  int x = millis();
  move_motor(900, 90, 90, 1, 1, 1, 1);
  delay(10);
  move_motor(900, -90, -90, 1, 1, 1, 1);
  Serial.println(millis() - x);
  

}
