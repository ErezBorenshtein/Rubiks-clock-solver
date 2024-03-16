#include <AccelStepper.h>
#include <MultiStepper.h>

/*const int RELAY_PIN = 7;  // the Arduino pin, which connects to the IN pin of relay


void setup() {
  // initialize digital pin 3 as an output.
  pinMode(RELAY_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(RELAY_PIN, HIGH); // uhlock the door
  delay(5000);
  digitalWrite(RELAY_PIN, LOW);  // lock the door   
  delay(5000);
}
*/
/*
// Define the number of steps per revolution for your stepper motors
const int stepsPerRevolution = 200000;

// Define the motor pins
const int motor1StepPin = 2;
const int motor1DirPin = 5;

const int motor2StepPin = 3;
const int motor2DirPin = 6;

const int motor3StepPin = 4;
const int motor3DirPin = 7;

const int motor4StepPin = 12;
const int motor4DirPin = 13;

// Create instances of the AccelStepper class for each motor
AccelStepper stepperUR(1, motor1StepPin, motor1DirPin); // (Type:driver, STEP, DIR)
AccelStepper stepperDR(1, motor2StepPin, motor2DirPin);
AccelStepper stepperDL(1, motor3StepPin, motor3DirPin);
AccelStepper stepperUL(1, motor4StepPin, motor4DirPin);

// Create an array to hold the stepper motors
AccelStepper* steppers[] = {&stepperUR, &stepperDR, &stepperDL, &stepperUL};

// Create an instance of the MultiStepper class
MultiStepper multiStepper;

void setup() {

  Serial.begin(9600);
  // Set the maximum speed and acceleration for each motor
  for (int i = 0; i < 4; i++) {
    steppers[i]->setMaxSpeed(3800);
    steppers[i]->setAcceleration(10000);
  }

  // Add each stepper motor to the MultiStepper instance
  for (int i = 0; i < 4; i++) {
    multiStepper.addStepper(*steppers[i]);
  }
}

void loop() {
  // Set the target positions for each motor
  long positions[] = {stepsPerRevolution, -stepsPerRevolution, stepsPerRevolution, -stepsPerRevolution}; // Adjust as needed
  long positions2[] = {-stepsPerRevolution, stepsPerRevolution, -stepsPerRevolution, stepsPerRevolution}; // Adjust as needed

  // Move all motors to their target positions
  multiStepper.moveTo(positions);

  // Move all motors simultaneously
  while (multiStepper.run()) {
    // Do nothing while motors are moving
  }

  // Delay for a moment before the next movement
  delay(1000);

  multiStepper.moveTo(positions2);

  // Move all motors simultaneously
  while (multiStepper.run()) {
    // Do nothing while motors are moving
  }

  // Delay for a moment before the next movement
  delay(1000);
  
  stepperDR.moveTo(3200); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  stepperDR.runSpeedToPosition();
  delay(1000);
  stepperDR.moveTo(0); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  stepperDR.runSpeedToPosition();
  delay(5000);


}*/

#define UP 0
#define DOWN 1

class Solenoids{
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

    Solenoids(){
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
    }

    void setUp(){
      for(int i =solenoidTUR; i <= solenoidBUL;i++){
          pinMode(i,OUTPUT);

      }
    }

    void push(int pin, int side){
      if(side == 0){
        //return;
      }
      int actualPin = pin+side; // Top or bottom solenoids
      digitalWrite(actualPin,HIGH);
      Serial.println("pushed: "+String(actualPin));
    }

    void reset(){
      set(1,1,1,1);
      delay(10000);
    }

    void set(int stateUR, int stateDR, int stateDL, int stateUL){
      push(solenoidTUR,stateUR);
      push(solenoidTDR,stateDR);
      push(solenoidTDL,stateDL);
      push(solenoidTUL,stateUL);
      delay(18);
      //delay(1000);

      for(int i =solenoidTUR; i <= solenoidBUL;i++){
          digitalWrite(i,LOW);

      }
    }
};


class Clock{
  public:
  
    float oneHourDeg;
    int stepsPerHour;

    AccelStepper* stepperUR;
    AccelStepper* stepperDR;
    AccelStepper* stepperDL;
    AccelStepper* stepperUL;

    MultiStepper multiSteppers;

    Solenoids solenoids;

    AccelStepper* steppers[4];

    Clock(){

      /*
      //with 1/32 resolution(2 jumpers)
      
      oneHourDeg = 30.0*(0.7865); 
      stepsPerHour =(int)(3200.0/360.0)*oneHourDeg;*/

      oneHourDeg = 30.0*(0.8375); 
      stepsPerHour =(int)(3200.0/360.0)*oneHourDeg;

      steppers[0] = (stepperUR = new AccelStepper(1, 2, 5));
      steppers[1] = (stepperDR = new AccelStepper(1, 3, 6));
      steppers[2] = (stepperDL = new AccelStepper(1, 4, 7));
      steppers[3] = (stepperUL = new AccelStepper(1, 12, 13));

      solenoids.setUp();

      for(int i=0;i<4;i++){
        AccelStepper* stepper = steppers[i];

        multiSteppers.addStepper(*stepper);

        stepper->setMaxSpeed(3800); // Set maximum speed value for the stepper
        stepper->setAcceleration(10000); // Set acceleration value for the stepper
        stepper->setCurrentPosition(0); // Set the current position to 0 steps
        stepper->setSpeed(2500);

      }


      /*stepperUR->setMaxSpeed(3800); // Set maximum speed value for the stepper
      stepperUR->setAcceleration(10000); // Set acceleration value for the stepper
      stepperUR->setCurrentPosition(0); // Set the current position to 0 steps
      stepperUR->setSpeed(2500);
 
      stepperDR->setMaxSpeed(3800);
      stepperDR->setAcceleration(10000);
      stepperDR->setCurrentPosition(0);
      stepperDR->setSpeed(2500);*/
    }
    
    void setPins(int stateUR, int stateDR, int stateDL, int stateUL){
      solenoids.set(stateUR, stateDR, stateDL, stateUL);
    }
  
    void moveTo(int hour, int dir){
        //Serial.println("easda");
        stepperDR->moveTo(dir*hour*stepsPerHour);
        stepperDR->runToPosition();
        //unsigned long startTime = millis();
        while(stepperDR->isRunning()){
          delay(10);
        }
        //unsigned long duration = millis() - startTime;
        //Serial.println(duration);
    }


    void rotate(int positions[4]){
      long degs[4];
      for(int i=0; i<4; i++){
        degs[i] = positions[i]*stepsPerHour;
      }
      //Serial.println(degs[3]);
      multiSteppers.moveTo(degs);
      while(multiSteppers.run()){
        //Serial.println("running");
      }
      //Serial.println("while");
    }

    void reset(){
      solenoids.reset();
    }
};

class ClockOperator{
  public:
    Clock clock;
    int states[4];

    ClockOperator(){
      for (int i = 0; i < 4; i++) {
            states[i] = 0;
        }
    }

    void rotate(String command){
      //Serial.println(command);
      int positions[4];
      int hours = command[2]-'0';
      if(command[1] == '-'){
        hours*=-1;
      }
      //Serial.println(hours);

      for(int i=3 ; i<7 ; i++){
        //Serial.println(command[i]);
        if(command[i] == '0'){
          positions[i-3] =states[i-3];
        }
        else{

          positions[i-3] = states[i-3]+hours;
        }
      }
      /*Serial.println(positions[0]);
      Serial.println(positions[1]);
      Serial.println(positions[2]);
      Serial.println(positions[3]);*/
      clock.rotate(positions);
    }

    int* ClockOperator::getPositions(String command) {
    static int positions[4]; // Array to store positions

    int hours = command[2] - '0';
    if (command[1] == '-') {
        hours *= -1;
    }

    for (int i = 3; i < 7; i++) {
        if (command[i] == '0') {
            positions[i - 3] = states[i - 3]; // If the position is 0, use the current state
        } else {
            positions[i - 3] = states[i - 3] + hours; // Otherwise, add/subtract hours from the current state
        }
    }

    return positions;
}


    void setPins(String command){
        clock.setPins(command[1]-'0',command[2]-'0' ,command[3]-'0' ,command[4]-'0');
    }

    bool isCommandValid(String command){

      if (command[0] != 'r' && command[0] !='p')
        return false;

      if (command.length()!=7)
        return false;

      if (command[0] == 'r'){

        if (command[1] !='+' && command[1]!='-')
          return false;

        if (command[2] >'6')
          return false;
        for(int i =3; i<7;i++)
          if(command[i] >'1' && command[i]<'0')
            return false;
      }
      if(command[0] =='p'){
        if(command[5] != '0' && command[6]!= '0')
          return false;
        for(int i =3; i<7;i++)
          if(command[i] >'1' && command[i]<'0')
            return false;
      }
      return true;
    }

    void runCommand(String command){
      if(!isCommandValid(command)){
        Serial.println("command is not valid");
        return;
      }

      if(command[0] == 'r'){
        //Serial.println("rotate");
        rotate(command);
      }
      if(command[0] == 'p'){
        setPins(command);
      }
    }



    /*void solve(String solution){
      int chunkSize= 7+1;
      int numOfCommands = solution.length()/chunkSize;
      int allCommands[21][4];

      for(int i =0; i<numOfCommands;i++){
        getPositions(solution.substring(i*chunkSize,(i+1)*chunkSize-1))
        runCommand(solution.substring(i*chunkSize,(i+1)*chunkSize-1));
      }
    }*/

    void solve(String solution){
      int chunkSize = 7 + 1;
      int numOfCommands = solution.length() / chunkSize;
      int allCommands[numOfCommands][4]; 

      for (int i = 0; i < numOfCommands; i++) {
          int* positions = getPositions(solution.substring(i * chunkSize, (i + 1) * chunkSize - 1));

          for (int j = 0; j < 4; j++) {
              allCommands[i][j] = positions[j];
          }
      }


    }

    void reset(){
      clock.reset();
    }
};

String readSolution(){
  while(!Serial.available()){
    delay(10);
  }
  String solution = Serial.readString();
  Serial.println(solution);
  return solution;
}


ClockOperator clockOperator;

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  delay(5000); //DO NOT DELETE

  /*String solution = readSolution();
  Serial.println(solution);
  if(solution.length()>0){
    long start = millis();
    clockOperator.solve(solution);
    long duration = millis()-start;
    Serial.println(duration);
  }*/
  //clockOperator.reset();
  clockOperator.runCommand("p100000");
  //delay(1000);
  //clockOperator.runCommand("r+11000");
  //delay(1000);
  //clockOperator.runCommand("p011100");

  delay(1000000);


}
