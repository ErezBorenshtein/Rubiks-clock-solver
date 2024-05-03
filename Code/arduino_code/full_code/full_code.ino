#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

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
      int actualPin = pin+side; // Top or bottom solenoids
      digitalWrite(actualPin,HIGH);
      String pins_names[] = {"UR-D","UR-U","DR-D","DR-U","DL-D","DL-U","UL-D","UL-U"};
      //Serial.println("pushed: "+pins_names[actualPin-22]);
      
    }

    void reset(){
      set(1,1,1,1);
    }

    long last_push = 0;

    void set(int stateUR, int stateDR, int stateDL, int stateUL){
      push(solenoidTUR,stateUR);
      push(solenoidTDR,stateDR);
      push(solenoidTDL,stateDL);
      push(solenoidTUL,stateUL);
      //Serial.println(millis()-last_push);
      last_push = millis();
      //Serial.println();
      //delay(1000);
      delay(50);

      for(int i =solenoidTUR; i <= solenoidBUL;i++){
          digitalWrite(i,LOW);

      }
      //delay(500);
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
      ///with 1/32 resolution(2 jumpers)
      
      oneHourDeg = 30.0*(0.7865); 
      stepsPerHour =(int)(3200.0/360.0)*oneHourDeg;*/

      oneHourDeg = 30.0*(0.8375); 
      stepsPerHour =(int)(3200.0/360.0)*oneHourDeg;

      steppers[0] = (stepperUR = new AccelStepper(1, 2, 5));
      steppers[1] = (stepperDR = new AccelStepper(1, 3, 6));
      //steppers[1] = (stepperDR = new AccelStepper(1, 6, 3));
      steppers[2] = (stepperDL = new AccelStepper(1, 4, 7));
      steppers[3] = (stepperUL = new AccelStepper(1, 12, 13));
      solenoids.setUp();

      for(int i=0;i<4;i++){
        AccelStepper* stepper = steppers[i];

        multiSteppers.addStepper(*stepper);

        //stepper->setMaxSpeed(1000); // Set maximum speed value for the stepper
        stepper->setMaxSpeed(3500); // Set maximum speed value for the stepper
        stepper->setAcceleration(10000); // Set acceleration value for the stepper
        stepper->setCurrentPosition(0); // Set the current position to 0 steps
        //stepper->setSpeed(1000);
        stepper->setSpeed(3500);

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
    int wheels_states[4];
    int current_pins_states[4]; //used for monitoring the state of the clock pins

    ClockOperator(){
      for (int i = 0; i < 4; i++) {
            wheels_states[i] = 0;
        }
    }

    void rotate(String command){
      //Serial.println(command);
      
      //check if pin's states is valid for rotation
      int moving_wheel_pin_state = (command[3]-'0' == current_pins_states[0]); //The state of the pins around the moving wheel
      for(int i=1;i<4;i++){
        int expected_pin_state = moving_wheel_pin_state==(command[i+3]-'0');//The state of the current pin near the wheel

        if(expected_pin_state != current_pins_states[i]){
          Serial.println("pin state is not valid expected: "+String(expected_pin_state)+" current: " + String(current_pins_states[i]));
          while(true){ //Stop the program 
            delay(10);
          }
        }
      }

      //calculate how much each wheel need to move
      int positions[4];
      int hours = command[2]-'0';
      if(command[1] == '-'){
        hours*=-1;
      }
      //Serial.println(hours);

      for(int i=3 ; i<7 ; i++){
        //Serial.println(command[i]);
        if(command[i] == '0'){
          positions[i-3] =wheels_states[i-3];

        }
        else{

          positions[i-3] = wheels_states[i-3]+hours;
          wheels_states[i-3] = positions[i-3]; //update the current wheel state
        }
      }
      clock.rotate(positions);
    }

    void setPins(String command){
      current_pins_states[0] = command[1]-'0';
      current_pins_states[1] = command[2]-'0';
      current_pins_states[2] = command[3]-'0';
      current_pins_states[3] = command[4]-'0';
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
        if(command[5] != ' ' && command[6]!= ' ')
          return false;
        for(int i =3; i<7;i++)
          if(command[i] >'1' && command[i]<'0')
            return false;
      }
      return true;
    }

    void runCommand(String command){
      //waitUntilMotorEnabeld();

      //Serial.println(command);

      if(!isCommandValid(command)){
        Serial.println("command is not valid");
        return;
      }
      //Serial.println(1);
      if(command[0] == 'r' && command[2] != '0'){
        //Serial.println("rotate");
        rotate(command);
      }
      if(command[0] == 'p'){
        setPins(command);
      }
      //Serial.println(2);
      delay(100);
      //delay(100);
    }

    void solve(String solution){
      int chunkSize= 7+1;
      int numOfCommands = solution.length()/chunkSize;

      for(int i =0; i<numOfCommands;i++){
        runCommand(solution.substring(i*chunkSize,(i+1)*chunkSize-1));
      }
    }

    /*void solve(String solution){
      int chunkSize = 7 + 1;
      int numOfCommands = solution.length() / chunkSize;
      int allCommands[numOfCommands][4]; 

      for (int i = 0; i < numOfCommands; i++) {
          int* positions = getPositions(solution.substring(i * chunkSize, (i + 1) * chunkSize - 1));

          for (int j = 0; j < 4; j++) {
              allCommands[i][j] = positions[j];
          }
      }


    }*/

    void reset(){
      Serial.println("Resetting...");
      for(int i =0; i<4;i++){
        current_pins_states[i] =0;
      }
      clock.reset();
    }
};


String readSolution(){
  String commands;
  if (Serial.available() > 0) {
    // Read the incoming data
    commands = Serial.readStringUntil('\n');
    
    // Print the received string
    Serial.println("Received: " + commands);
  }
  return commands;
}

/*String readSolution(){
  String commands;
  while(!Serial.available()){
    delay(10);
  }
  commands = Serial.readStringUntil("\n");
  while(true){
    String message = Serial.readStringUntil("\n");
    if(message == "end")
      break;
    commands+= message;
    Serial.println("message delivered: "+message);
  }
  Serial.println("final commands: "+commands);
  delay(100);
  return commands;
}*/

ClockOperator clockOperator;
String commands;

void setup2() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  Serial.println("ready");

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  while(!Serial.available()){
    delay(10);
  }

  if (Serial.available() > 0) {
    // Read the incoming data
    commands = readSolution();
  }

  Serial.println("end of read");

  //clockOperator.reset();  needed only after optimizing pin settings
  delay(1000);
  

}

void setup(){
  Serial.begin(115200);
  
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(18), powerEnabler, CHANGE);
  
  Serial.println("somethng");
  //clockOperator.reset();  needed only after optimizing pin settings
  delay(1000);
}

void powerEnabler(){
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do{

    mode = digitalRead(LIMIT_PIN);

  }while(mode==1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}


void loop2() {

  unsigned long start_time = millis();
  //clockOperator.runCommand("p100100");
  //clockOperator.runCommand("r-21001");
  clockOperator.solve(commands);
  clockOperator.reset();
  Serial.println("total time: "+String(millis()- start_time));
  delay(1000000);

}

void loop(){
  clockOperator.runCommand("p0000  ");
  clockOperator.runCommand("p0001  ");
  clockOperator.runCommand("p0010  ");
  clockOperator.runCommand("p0011  ");
  clockOperator.runCommand("p0100  ");
  clockOperator.runCommand("p0101  ");
  clockOperator.runCommand("p0110  ");
  clockOperator.runCommand("p0111  ");
  clockOperator.runCommand("p1000  ");
  clockOperator.runCommand("p1001  ");
  clockOperator.runCommand("p1010  ");
  clockOperator.runCommand("p1011  ");
  clockOperator.runCommand("p1100  ");
  clockOperator.runCommand("p1101  ");
  clockOperator.runCommand("p1111  ");

}  