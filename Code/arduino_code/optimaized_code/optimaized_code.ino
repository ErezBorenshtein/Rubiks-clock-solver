#include <Bounce2.h>

#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19


//some delay time after the wheels completed roatation. tests show that some time is needed (around 5ms)
//#define ROTATION_DELAY 35  //!for testing without pins
#define ROTATION_DELAY 5  //full operation time


//how much time the dolenoid needs current in order to successfully move the pin. tests show that 35ms is the minimal time
//#define PUSHER_DELAY 35 //for full operation time
#define PUSHER_DELAY 40  //for testing - relaxed mode

#define TIMER_PIN 14

//motors rotation speed. 700 is the optimized
#define ROATATION_SPEED 700

volatile const int stepPul[4] = { 48, 46, 44, 42 };
volatile const int stepDir[4] = { 49, 47, 45, 43 };

Bounce debouncer = Bounce();

void disablePins() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(stepPul[i], LOW);
    digitalWrite(stepDir[i], LOW);
  }
}

class Solenoids {
public:

  int solenoidTUR;
  int solenoidTDR;
  int solenoidTDL;
  int solenoidTUL;

  //solenoids on the bottom side
  int solenoidBUR;
  int solenoidBDR;
  int solenoidBDL;
  int solenoidBUL;

  Solenoids() {
    //solenoids on the top side
    solenoidTUR = 22;
    solenoidTDR = 24;
    solenoidTDL = 26;
    solenoidTUL = 28;

    //solenoids on the bottom side
    solenoidBUR = 23;
    solenoidBDR = 25;
    solenoidBDL = 27;
    solenoidBUL = 29;

    for (int i = solenoidTUR; i <= solenoidBUL; i++) {
      pinMode(i, OUTPUT);
    }
  }


  void push(int pin, int side) {
    int actualPin = pin + side;  // Top or bottom solenoids
    digitalWrite(actualPin, HIGH);
    //String pins_names[] = {"UR-D","UR-U","DR-D","DR-U","DL-D","DL-U","UL-D","UL-U"};
    //Serial.println("pushed: "+pins_names[actualPin-22]);
  }

  void reset() {
    set(1, 1, 1, 1);
  }


  void set(int stateUR, int stateDR, int stateDL, int stateUL) {
    push(solenoidTUR, stateUR);
    push(solenoidTDR, stateDR);
    push(solenoidTDL, stateDL);
    push(solenoidTUL, stateUL);
    //Serial.println("setting solenoids");
    delay(PUSHER_DELAY);  //the lowest time that the solenoids can be on
    for (int i = solenoidTUR; i <= solenoidBUL; i++) {
      digitalWrite(i, LOW);
    }
    delay(5);
  }
};

class ClockSteppers {
public:
  //const int speed = 700;
  int speed = ROATATION_SPEED;
  const long turnSteps = 400 * 2;

  long currentPositions[4] = { 0, 0, 0, 0 };

  ClockSteppers() {
    for (int i = 0; i < 4; i++) {
      pinMode(stepPul[i], OUTPUT);
      pinMode(stepDir[i], OUTPUT);
    }

    disablePins();
  }

  void moveMotor(long steps[]) {

    long maxSteps = 0;
    for (int i = 0; i < 4; i++) {
      bool hl = true;
      if (steps[i] > 0) hl = false;
      digitalWrite(stepDir[i], hl);
      steps[i] = abs(steps[i]);
      steps[i] *= 2;  // pulse is defined by lowering and rising the pin so it should be doubled
      if (steps[i] > maxSteps) maxSteps = steps[i];
      //steps[i] = steps[i] * turnSteps / 360;
      //Serial.println("steps: "+String(steps[i]));
    }
    //Serial.println("maxSteps: "+String(maxSteps));

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
      //Serial.println("delay_time: "+String(delay_time));
      delayMicroseconds(delay_time);
    }
  }

  void moveTo(long toPositions[]) {
    long steps[4];
    for (int i = 0; i < 4; i++) {
      steps[i] = toPositions[i] - currentPositions[i];
      //Serial.println("steps: "+String(steps[i]));
    }

    moveMotor(steps);

    for (int i = 0; i < 4; i++) {
      currentPositions[i] = toPositions[i];
    }
  }
};


class Clock {
public:
  //const int stepsPerRotation = (400*(3.0/4))*2; // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
  const float stepsPerRotation = (400 * (12.0 / 22.0)) * 2;  // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
  //const int stepsPerRotation = (400*(9.9/18.15))*2; // 400 is the number of steps per rotation, 12/22 is the gear ratio, 8 is the microstepping
  //int offset[4] = {0,0,0,0}; //gear ratio offset for each wheel

  Solenoids solenoids;

  ClockSteppers clockSteppers;


  void setPins(int stateUR, int stateDR, int stateDL, int stateUL) {
    solenoids.set(stateUR, stateDR, stateDL, stateUL);
    //Serial.println("setting pins: "+String(stateUR)+String(stateDR)+String(stateDL)+String(stateUL));
  }


  void rotate(float positions_degs[4]) {
    long steps[4];
    for (int i = 0; i < 4; i++) {
      //steps[i] = (positions_degs[i]*(stepsPerRotation+offset[i])/360.0)+0.5;
      float roundFactor = positions_degs[i] > 0 ? 0.5 : -0.5;
      steps[i] = (positions_degs[i] * stepsPerRotation / 360.0) + roundFactor;
      //Serial.println("steps "+String(i)+": "+String(steps[i])+ "      degs: "+String(positions_degs[i]));
    }

    clockSteppers.moveTo(steps);
  }

  void reset() {
    solenoids.reset();
  }
};

class ClockOperator {
public:
  Clock clock;
  float wheels_states_degs[4];
  int current_pins_states[4];  //used for monitoring the state of the clock pins

  ClockOperator() {
    for (int i = 0; i < 4; i++) {
      wheels_states_degs[i] = 0.0;
    }
  }

  void rotate(String command) {
    //Serial.println(command);

    //TODO: check if the command is valid
    //check if pin's states is valid for rotation
    //int moving_wheel_pin_state = (command[3]-'0' == current_pins_states[0]); //The state of the pins around the moving wheel
    //for(int i=1;i<4;i++){
    //  int expected_pin_state = moving_wheel_pin_state==(command[i+3]-'0');//The state of the current pin near the wheel

    //  if(expected_pin_state != current_pins_states[i]){
    //    Serial.println("pin state is not valid expected: "+String(expected_pin_state)+" current: " + String(current_pins_states[i]));
    //    while(true){ //Stop the program
    //      delay(10);
    //    }
    //  }
    //}

    //calculate how much each wheel need to move
    float positions_degs[4];
    for (int i = 0; i < 4; i++) {
      float hours_deg = (command[i * 2 + 2] - '0') * 360.0 / 12.0;

      if (command[i * 2 + 1] == '-') {
        hours_deg *= -1;
      }

      //Serial.println("hours_deg: "+String(hours_deg));

      positions_degs[i] = wheels_states_degs[i] + hours_deg;
      wheels_states_degs[i] = positions_degs[i];  //update the current wheel state
                                                  //Serial.println("wheel "+String(i)+": "+String(positions_degs[i]));
    }
    clock.rotate(positions_degs);
  }


  void setPins(String command) {
    current_pins_states[0] = command[1] - '0';
    current_pins_states[1] = command[2] - '0';
    current_pins_states[2] = command[3] - '0';
    current_pins_states[3] = command[4] - '0';
    clock.setPins(command[1] - '0', command[2] - '0', command[3] - '0', command[4] - '0');
  }

  bool isCommandValid(String command) {

    if (command[0] != 'r' && command[0] != 'p') {

      Serial.println("ERR6");
      Serial.println(command[0]);
      return false;
    }

    if (command.length() != 9) {
      Serial.println("ERR5");
      Serial.println(command.length());
      return false;
    }

    if (command[0] == 'r') {
      for (int i = 0; i < 4; i++) {
        if (command[i * 2 + 1] != '+' && command[i * 2 + 1] != '-') {
          //Serial.println("command[i*2+1]: "+command[i*2+1]);
          Serial.println("ERR1");
          return false;
        }
        if (command[i * 2 + 2] < '0' && command[i * 2 + 2] > '6') {
          Serial.println("ERR2");
          return false;
        }
      }
    }
    if (command[0] == 'p') {
      if (command[5] != '0' && command[6] != '0' && command[7] != '0' && command[8] != '0') {
        Serial.println("ERR3");
        return false;
      }
      for (int i = 3; i < 7; i++)
        if (command[i] > '1' && command[i] < '0') {
          Serial.println("ERR4");
          return false;
        }
    }
    return true;
  }

  void runCommand(String command) {

    //Serial.println("running command: "+command);

    if (!isCommandValid(command)) {
      Serial.println("command is not valid");
      return;
    }
    
    //Serial.println("running command:" + command);
    if (command[0] == 'r') {
      
      rotate(command);
      delay(ROTATION_DELAY);
    }

    if (command[0] == 'p') {
      setPins(command);
    }

    //delay(500);
  }

  void solve(String solution) {

    int chunkSize = 9 + 1;
    int numOfCommands = solution.length() / chunkSize;
    Serial.println("num of commands: " + String(numOfCommands));
    for (int i = 0; i < numOfCommands; i++) {
      String command = solution.substring(i * chunkSize, (i + 1) * chunkSize - 1);
      //Serial.println("command"+command);
      runCommand(command);
    }
  }


  void reset() {
    Serial.println("Resetting...");
    for (int i = 0; i < 4; i++) {
      current_pins_states[i] = 0;
    }
    clock.reset();
  }

  void checkPins() {
    setPins("p0000  ");
    delay(100);
    reset();
  }
};


ClockOperator clockOperator;
String commands = "";

void powerEnabler() {
  int mode;
  //Serial.println("setting enble to low");
  delay(10);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do {
    mode = digitalRead(LIMIT_PIN);
    digitalWrite(LED_BUILTIN, mode);
    delay(3);
  } while (mode == 1);

  //Serial.println("setting enble to high");
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  delay(10);
  disablePins();
  delay(2000);
}

void waitForLimitLow() {
  while (true) {
    debouncer.update();
    if (debouncer.read() == LOW) {
      break;  // Switch is pressed (debounced)
    }
    delay(1);  // Small delay to reduce CPU load
  }
}

//this is the setup for testing the robot
void setup2() {
  Serial.begin(115200);
  Serial3.begin(115200);

  delay(50);

  debouncer.attach(LIMIT_PIN);
  debouncer.interval(10);  // 10 ms debounce interval

  Serial3.println("reset");

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //pinMode(TIMER_PIN, OUTPUT);
  digitalWrite(TIMER_PIN, LOW);
  //Serial.println("setting enable to low");
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  delay(10);

  waitForLimitLow();  // Wait until the switch is pressed
  //Serial.println("Limit switch activated!");
  debouncer.attach(255);  //detach the bounce from the pin

  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();
  //UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+
  //commands = "p01110000 r-2-2-2-2 p00110000 r-3-3-3-3 p00010000 r-3-3-3+4 p01010000 r+6+1+6+1 p01000000 r-4-5-4-4 p11000000 r-5-5+1+1 p11010000 r-2-2+1-2\n";
  //UR2+ DR4- DL5- UL1- U5- R1- D0+ L4+ ALL1+ y2 U4+ R1+ D3- L2+ ALL2-
  //commands = "p01110000 r+4-2-2-2 p00110000 r-2-2-2-2 p00010000 r-4-4-4+4 p01010000 r-3+6-3+6 p01000000 r+6-3+6+6 p11000000 r-6-6+4+4 p11010000 r+0+0-3+0\n";

  //UR1- DR0+ DL4+ UL2+ U0+ R0+ D2+ L4+ ALL6+ y2 U5- R1- D1+ L2+ ALL3+
  commands = "p01110000 r-5-2-2-2 p00110000 r-5-5+3+3 p00010000 r-2-2-2-1 p01010000 r-4-5-4-5 p01000000 r+5+6+5+5 p11000000 r-5-5-2-2 p11010000 r-1-1+1-1\n";
  //commands = "p01110000 r-3-4-4-4 p00110000 r+0+0+0+0 p00010000 r-3-3-3+0 p01010000 r+3-5+3-5 p01000000 r+2+2+2+2 p11000000 r+0+0+0+0 p11010000 r-2-2+2-2\n";

  //clockOperator.reset();  //needed only after optimizing pin settings
  delay(1500);
}


//this is the full operational
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  delay(50);

  Serial3.println("reset");

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  //pinMode(TIMER_PIN, OUTPUT);


  digitalWrite(TIMER_PIN, LOW);
  delay(10);
  digitalWrite(TIMER_PIN, HIGH);
  //delay(10);
  digitalWrite(TIMER_PIN, LOW);
  Serial.println ("Ready to start, close the robot");
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();

  // Wait for serial port to open
  while (!Serial) {
    delay(10);  // Wait for serial port to connect
  }

  Serial.println("ready");  //!important! part of the protocol
  delay (50);
  // while (commands == "") {
  //   if (Serial.available() > 0) {
  //     // Read the incoming data
  //     commands = Serial.readStringUntil('\n');
  //   }
  // }

  commands = "";
  char incomingChar;

  while (true) {
    if (Serial.available()) {
      incomingChar = Serial.read();
      if (incomingChar == '\n') {
        break;  // message complete
      }
      commands += incomingChar;
      Serial.print(incomingChar);
    }
  }
  Serial.println("");

  // Print the received string
  Serial.println("Received: " + commands);
  Serial.println (commands.length());
  //commands = "p0111   r+01000 r-20111 p0011   r+01100 r+00011 p0001   r-10001 r+11110 p0101   r+20101 r-21010 p0100   r-10100 r+11011 p1100   r+01100 r+00011 p1101   r-21101 r+00010\n";

  Serial.println("end of read");
  //clockOperator.reset();  //needed only after optimizing pin settings

  delay(1000);
}


//this is the real setup function that is used in the loop
void loop() {
  //Serial.println((String)"mil 0: "+(String)millis());

  if (Serial.available()) {
    //Serial.println((String)"mil 1: "+(String)millis());
    //String command = Serial.readStringUntil("\n");
    ////Serial.println(command);
    //Serial.println((String)"mil 2: "+(String)millis());
    //if(command=="start\n"){

    unsigned long start_time = millis();

    Serial3.println("start");
    Serial.println("------start");

    clockOperator.solve(commands);

    Serial3.println("stop");
    Serial.println("done");

    Serial.println("total time: " + String(millis() - start_time));
    clockOperator.reset();
    delay(1000000);
    //}
  }
  delay(3);
}


void test_motor_rotation() {

  clockOperator.runCommand("p11110000");
  clockOperator.runCommand("r+1+1+1+1");
  clockOperator.runCommand("r+3+3+3+3");
  clockOperator.runCommand("r-6-6-6-6");

  clockOperator.runCommand("p00010000");
  clockOperator.runCommand("r-3-3-3+6");
  clockOperator.runCommand("r+4+4+4-2");
  clockOperator.runCommand("r+4+4+4-2");
  clockOperator.runCommand("r-4-4-4+2");
  clockOperator.runCommand("r-1-1-1+3");

  clockOperator.runCommand("p01010000");
  clockOperator.runCommand("r-1+3-1+3");
  clockOperator.runCommand("r+6-3+6-3");
  clockOperator.runCommand("r+6-1+6-1");
  clockOperator.runCommand("r+1-1+1-1");
  clockOperator.runCommand("r+6-6+6-6");

  clockOperator.runCommand("p01110000");
  clockOperator.runCommand("r+1-1-1-1");
  clockOperator.runCommand("r+6-6-6-6");
  clockOperator.runCommand("r-6+6+6+6");
  clockOperator.runCommand("r-1-6-6-6");
  clockOperator.runCommand("r-3-5-5-5");

  clockOperator.runCommand("p11110000");
  clockOperator.runCommand("r-5-5-5-5");
  clockOperator.runCommand("r+1+1+1+1");
  clockOperator.runCommand("r-2-2-2-2");
  clockOperator.runCommand("r+6+6+6+6");


  clockOperator.runCommand("p11110000");
}

void run_command_with_delay(String command) {
  Serial.println("running command:" + command);
  clockOperator.runCommand(command);
  delay(1000);
}

void test_pins() {
  Serial.println("testing pins");
  run_command_with_delay("p00000000");
  run_command_with_delay("p10000000");
  run_command_with_delay("p01000000");
  run_command_with_delay("p00100000");
  run_command_with_delay("p00010000");
  run_command_with_delay("p11110000");
  run_command_with_delay("p11100000");
  run_command_with_delay("p11010000");
  run_command_with_delay("p10110000");
  run_command_with_delay("p01110000");
  delay(2000000);
  run_command_with_delay("p00010000");
  run_command_with_delay("p00100000");
  run_command_with_delay("p00110000");
  run_command_with_delay("p01000000");
  run_command_with_delay("p01010000");
  run_command_with_delay("p01100000");
  run_command_with_delay("p01110000");
  run_command_with_delay("p10000000");
  run_command_with_delay("p10010000");
  run_command_with_delay("p10100000");
  run_command_with_delay("p10110000");
  run_command_with_delay("p11000000");
  run_command_with_delay("p11010000");
  run_command_with_delay("p11110000");
  run_command_with_delay("p00000000");
  run_command_with_delay("p11110000");
  run_command_with_delay("p10000000");
}

void loop2() {

  //test_motor_rotation();
  test_pins();
  delay(10000000);
}