#include <AccelStepper.h>
#include <MultiStepper.h>

#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

//with cnc shield
//#define ROTATION_SPEED 3550
//#define ROTATION_DELAY 4

//with TB6600 drivers
//#define ROTATION_SPEED 13000 //max possible speed
#define ROTATION_SPEED 3000
#define ROTATION_DELAY 0

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
      //String pins_names[] = {"UR-D","UR-U","DR-D","DR-U","DL-D","DL-U","UL-D","UL-U"};
      //Serial.println("pushed: "+pins_names[actualPin-22]);
      
    }

    void reset(){
      set(1,1,1,1);
    }


    void set(int stateUR, int stateDR, int stateDL, int stateUL){
      push(solenoidTUR,stateUR);
      push(solenoidTDR,stateDR);
      push(solenoidTDL,stateDL);
      push(solenoidTUL,stateUL);

      delay(45); //the lowest time that the solenoids can be on
      for(int i =solenoidTUR; i <= solenoidBUL;i++){
          digitalWrite(i,LOW);
      }
      delay(5);
    }
};


class Clock{
  public:
    //const int stepsPerRotation = (400*(3.0/4))*2; // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
    const int stepsPerRotation = (400*(12.0/22))*2; // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
    int offset[4] = {0,1,0,0}; //gear ratio offset for each wheel
    

    AccelStepper* stepperUR;
    AccelStepper* stepperDR;
    AccelStepper* stepperDL;
    AccelStepper* stepperUL;

    MultiStepper multiSteppers;

    Solenoids solenoids;

    AccelStepper* steppers[4];

    Clock(){      
      //with 1.8 deg per step with 1/16 resolution(1 jumpers)
      //stepsPerHour =200;

      //with cnc shield
      //steppers[0] = (stepperUR = new AccelStepper(1, 2, 5));
      //steppers[1] = (stepperDR = new AccelStepper(1, 3, 6));
      //steppers[2] = (stepperDL = new AccelStepper(1, 4, 7));
      //steppers[3] = (stepperUL = new AccelStepper(1, 12, 13));

      //with TB6600 drivers
      steppers[0] = (stepperUR = new AccelStepper(1, 48, 49));
      steppers[1] = (stepperDR = new AccelStepper(1, 46, 47));
      steppers[2] = (stepperDL = new AccelStepper(1, 44, 45));
      steppers[3] = (stepperUL = new AccelStepper(1, 42, 43));

      solenoids.setUp();

      for(int i=0;i<4;i++){
        AccelStepper* stepper = steppers[i];

        multiSteppers.addStepper(*stepper);

        //int spd = 1000;
        int spd = ROTATION_SPEED;
        stepper->setMaxSpeed(spd); // Set maximum speed value for the stepper
        stepper->setAcceleration(100000); // Set acceleration value for the stepper
        stepper->setCurrentPosition(0); // Set the current position to 0 steps
        stepper->setSpeed(spd);

      }

    }
    
    void setPins(int stateUR, int stateDR, int stateDL, int stateUL){
      solenoids.set(stateUR, stateDR, stateDL, stateUL);
      //Serial.println("setting pins: "+String(stateUR)+String(stateDR)+String(stateDL)+String(stateUL));
    }
  
    void rotate(float positions_degs[4]){
      long steps[4];
      for(int i=0; i<4; i++){
        steps[i] = positions_degs[i]*(stepsPerRotation+offset[i])/360.0;
        //Serial.println("steps "+String(i)+": "+String(steps[i]));
      }
      multiSteppers.moveTo(steps);
      
      while(multiSteppers.run()){
        //delayMicroseconds(25);
      }
    }

    void reset(){
      solenoids.reset();
    }
};

class ClockOperator{
  public:
    Clock clock;
    float wheels_states_degs[4];
    int current_pins_states[4]; //used for monitoring the state of the clock pins

    ClockOperator(){
      for (int i = 0; i < 4; i++) {
          wheels_states_degs[i] = 0.0;
        }
    }

    void rotate(String command){
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
      for(int i=0 ; i<4 ; i++){   
          float hours_deg = (command[i*2+2]-'0')*360.0/12.0;

          if(command[i*2+1] == '-'){
            hours_deg*=-1;
          }

          //Serial.println("hours_deg: "+String(hours_deg));

          positions_degs[i] = wheels_states_degs[i]+hours_deg;
          wheels_states_degs[i] = positions_degs[i]; //update the current wheel state
          //Serial.println("wheel "+String(i)+": "+String(positions_degs[i]));
      }
      clock.rotate(positions_degs);
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

      if (command.length()!=9)
        return false;

      if (command[0] == 'r'){
        for(int i =0; i<4;i++){
          if(command[i*2+1] != '+' && command[i*2+1] != '-'){
            //Serial.println("command[i*2+1]: "+command[i*2+1]);
            return false;
          }
          if(command[i*2+2] < '0' && command[i*2+2] > '6')
            return false;
        }
      }
      if(command[0] =='p'){
        if(command[5] != '0' && command[6]!= '0'&& command[7]!= '0'&& command[8]!= '0')
          return false;
        for(int i =3; i<7;i++)
          if(command[i] >'1' && command[i]<'0')
            return false;
      }
      return true;
    }

    void runCommand(String command){

      //Serial.println("running command: "+command);

      if(!isCommandValid(command)){
        Serial.println("command is not valid");
        return;
      }

      if(command[0] == 'r'){
        rotate(command);
        delay(ROTATION_DELAY);
        
      }
      if(command[0] == 'p'){
        setPins(command);
        //delay(60);
      }
      delay(1000);
      
    }

    void solve(String solution){

      int chunkSize= 9+1;
      int numOfCommands = solution.length()/chunkSize;
      //Serial.println("num of commands: "+String(numOfCommands));
      for(int i =0; i<numOfCommands;i++){
        String command = solution.substring(i*chunkSize,(i+1)*chunkSize-1);
        //Serial.println("command"+command);
        runCommand(command);
      }
    }


    void reset(){
      Serial.println("Resetting...");
      for(int i =0; i<4;i++){
        current_pins_states[i] =0;
      }
      clock.reset();
    }

    void checkPins(){
       setPins("p0000  ");
       delay(100);
       reset();
    }
};


String readSolution(){
  String commands;
  Serial.println("reading solution");

  while(!Serial.available()){
    delay(10);
  }

  if (Serial.available() > 0) {
    // Read the incoming data
    commands = Serial.readStringUntil('\n');
    
    // Print the received string
    Serial.println("Received: " + commands);
  }
  return commands;
}


void powerEnabler(){
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do{

    mode = digitalRead(LIMIT_PIN);

  }while(mode==1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

ClockOperator clockOperator;
String commands;

void setup2() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();

  Serial.println("ready"); //!important! part of the protocol
  
  commands = readSolution();
  Serial.println("commands: "+commands);
  
  //commands = "p0111   r+01000 r-20111 p0011   r+01100 r+00011 p0001   r-10001 r+11110 p0101   r+20101 r-21010 p0100   r-10100 r+11011 p1100   r+01100 r+00011 p1101   r-21101 r+00010\n";

  Serial.println("end of read");
  //clockOperator.reset();  //needed only after optimizing pin settings
  delay(1000);
  
}

void setup() {
  Serial.begin(115200);

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();
  //UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+
  //commands = "p01110000 r-2-2-2-2 p00110000 r-3-3-3-3 p00010000 r-3-3-3+4 p01010000 r+6+1+6+1 p01000000 r-4-5-4-4 p11000000 r-5-5+1+1 p11010000 r-2-2+1-2\n";
  //UR2+ DR4- DL5- UL1- U5- R1- D0+ L4+ ALL1+ y2 U4+ R1+ D3- L2+ ALL2-
  commands = "p01110000 r+4-2-2-2 p00110000 r-2-2-2-2 p00010000 r-4-4-4+4 p01010000 r-3+6-3+6 p01000000 r+6-3+6+6 p11000000 r-6-6+4+4 p11010000 r+0+0-3+0\n";
  
  Serial.println("speed: "+(String)ROTATION_SPEED);
  clockOperator.reset();  //needed only after optimizing pin settings
  delay(1500);
  
}

void loop() {

  unsigned long start_time = millis();
  //clockOperator.runCommand("p100100");s
  //clockOperator.runCommand("r-21001");
  //Serial.println("commands: "+commands);
  clockOperator.solve(commands);
  //Serial.println("here");
  Serial.println("total time: "+String(millis()- start_time));
  clockOperator.reset();
  delay(1000000);

}

void loop2(){
  /*clockOperator.runCommand("p00000000");
  clockOperator.runCommand("p00010000");
  clockOperator.runCommand("p00100000");
  clockOperator.runCommand("p00110000");
  clockOperator.runCommand("p01000000");
  clockOperator.runCommand("p01010000");
  clockOperator.runCommand("p01100000");
  clockOperator.runCommand("p01110000");
  clockOperator.runCommand("p10000000");
  clockOperator.runCommand("p10010000");
  clockOperator.runCommand("p10100000");
  clockOperator.runCommand("p10110000");
  clockOperator.runCommand("p11000000");
  clockOperator.runCommand("p11010000");
  clockOperator.runCommand("p11110000");
  delay(2000);
  clockOperator.runCommand("p0101  ");
  clockOperator.runCommand("r-50101");
  clockOperator.runCommand("r+50101");
  clockOperator.runCommand("r-10101");
  clockOperator.runCommand("r+50101");
  clockOperator.runCommand("r-10101");
  clockOperator.runCommand("r-50101");
  clockOperator.runCommand("r-30101");
  clockOperator.runCommand("r+60101");
  clockOperator.runCommand("r-20101");
  clockOperator.runCommand("r+40101");
  
 clockOperator.runCommand("r+3+0+0+0");
 clockOperator.runCommand("r-6+0+0+0");
 clockOperator.runCommand("r+1+0+0+0");
 clockOperator.runCommand("r-6+0+0+0");
 clockOperator.runCommand("r+3+0+0+0");
 clockOperator.runCommand("r+6+0+0+0");
 clockOperator.runCommand("r+4+0+0+0")ֹ;*/ 
 //long x = millis();
 //clockOperator.runCommand("r+0+0+0+6");
 //Serial.println(millis()-x);
 //clockOperator.runCommand("r +0+0+0-1");
 //clockOperator.runCommand("r+d0+0+0+5");
 
  //clockOperator.runCommand("p01010000");
  //clockOperator.runCommand("p00000000");
  //long x = millis();
  clockOperator.clock.rotate(new float[4]{0,0,0,360*6});
  //Serial.println(millis()-x);
  //clockOperator.runCommand("p11000000");
  //clockOperator.runCommand("r+0+0+6+0");
  //Serial.println(millis()-x);
  //Serial.println(millis()-x);

  delay(5000);
}  