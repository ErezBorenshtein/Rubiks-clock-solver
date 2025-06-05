#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

#define ROTATION_DELAY 35 //!for testing without pins
//#define ROTATION_DELAY 5  //full operation time

//#define PUSHER_DELAY 25
//#define PUSHER_DELAY 35 //for full operation time
#define PUSHER_DELAY 70 //for testing

#define TIMER_PIN 14

volatile const int stepPul[4] = {48, 46, 44, 42};
volatile const int stepDir[4] = {49, 47, 45, 43};

void disablePins(){
  for(int i =0; i<4;i++){
    digitalWrite(stepPul[i],LOW);
    digitalWrite(stepDir[i],LOW);
  }
}

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

      delay(PUSHER_DELAY); //the lowest time that the solenoids can be on
      for(int i =solenoidTUR; i <= solenoidBUL;i++){
          digitalWrite(i,LOW);
      }
      delay(5);
    }
};

class ClockSteppers{
  public:
    //const int speed = 700;
    int speed = 700;
    const long turnSteps = 400 * 2;
    
    long currentPositions[4] = {0,0,0,0};

    ClockSteppers(){
      for(int i =0; i<4;i++){
        pinMode(stepPul[i],OUTPUT);
        pinMode(stepDir[i],OUTPUT);
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
        steps[i] *= 2; // pulse is defined by lowering and rising the pin so it should be doubled
        if(steps[i] > maxSteps) maxSteps = steps[i];
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

    void moveTo(long toPositions[]){
      long steps[4];
      for(int i =0; i<4;i++){
        steps[i] = toPositions[i] - currentPositions[i];
        //Serial.println("steps: "+String(steps[i]));
      }

      moveMotor(steps);

      for(int i =0; i<4;i++){
        currentPositions[i] = toPositions[i];
      }
    }

};


class Clock{
  public:
    //const int stepsPerRotation = (400*(3.0/4))*2; // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
    const float stepsPerRotation = (400*(12.0/22.0))*2; // 400 is the number of steps per rotation, 3/4 is the gear ratio, 8 is the microstepping
    //const int stepsPerRotation = (400*(9.9/18.15))*2; // 400 is the number of steps per rotation, 12/22 is the gear ratio, 8 is the microstepping
    //int offset[4] = {0,0,0,0}; //gear ratio offset for each wheel
    
    Solenoids solenoids;

    ClockSteppers clockSteppers;

    
    void setPins(int stateUR, int stateDR, int stateDL, int stateUL){
      solenoids.set(stateUR, stateDR, stateDL, stateUL);
      //Serial.println("setting pins: "+String(stateUR)+String(stateDR)+String(stateDL)+String(stateUL));
    }


    void rotate(float positions_degs[4]){
      long steps[4];
      for(int i=0; i<4; i++){
        //steps[i] = (positions_degs[i]*(stepsPerRotation+offset[i])/360.0)+0.5;
        float roundFactor = positions_degs[i]>0 ? 0.5: -0.5;
        steps[i] = (positions_degs[i]*stepsPerRotation/360.0)+roundFactor;
        //Serial.println("steps "+String(i)+": "+String(steps[i])+ "      degs: "+String(positions_degs[i]));
      }

      clockSteppers.moveTo(steps);

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
      }

      //delay(500);
      
    }

    void solve(String solution){

      int chunkSize= 9+1;
      int numOfCommands = solution.length()/chunkSize;
      Serial.println("num of commands: "+String(numOfCommands));
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


ClockOperator clockOperator;
String commands = "";

void powerEnabler(){
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do{

    mode = digitalRead(LIMIT_PIN);

  }while(mode==1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);

  disablePins();
  
}

//this is the setup for testing the robot
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  delay(50);

  Serial3.println("reset");

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  //pinMode(TIMER_PIN, OUTPUT);
  digitalWrite(TIMER_PIN, LOW);
  
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
void setup2() {
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
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();

  // Wait for serial port to open
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }

  Serial.println("ready"); //!important! part of the protocol
  
  while(commands == ""){
    if (Serial.available() > 0) {
      // Read the incoming data
      commands = Serial.readStringUntil('\n');
    }
  }
  // Print the received string
  Serial.println("Received: " + commands);
  
  //commands = "p0111   r+01000 r-20111 p0011   r+01100 r+00011 p0001   r-10001 r+11110 p0101   r+20101 r-21010 p0100   r-10100 r+11011 p1100   r+01100 r+00011 p1101   r-21101 r+00010\n";

  Serial.println("end of read");
  //clockOperator.reset();  //needed only after optimizing pin settings

  delay(1000);
  
}


//this is the real setup function that is used in the loop
void loop2() {
  //Serial.println((String)"mil 0: "+(String)millis());

  if(Serial.available()){
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

      Serial.println("total time: "+String(millis()- start_time));
      clockOperator.reset();
      delay(1000000);
    //}
  }
  delay(3);
}



void loop(){

  clockOperator.clock.clockSteppers.speed = 50;

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





  //clockOperator.runCommand("p11110000");
  //delay(1000);
  //clockOperator.runCommand("p00010000");
  //clockOperator.runCommand("p00100000");
  //clockOperator.runCommand("p00110000");
  //clockOperator.runCommand("p01000000");
  //clockOperator.runCommand("p01010000");
  //clockOperator.runCommand("p01100000");
  //clockOperator.runCommand("p01110000");
  //clockOperator.runCommand("p10000000");
  //clockOperator.runCommand("p10010000");
  //clockOperator.runCommand("p10100000");
  //clockOperator.runCommand("p10110000");
  //clockOperator.runCommand("p11000000");
  //clockOperator.runCommand("p11010000");
  //clockOperator.runCommand("p11110000");
  //delay(1000);
  //clockOperator.runCommand("p00000000");
  //delay(1000);
  //clockOperator.runCommand("p11110000");
  //delay(1000);
  //clockOperator.runCommand("p10000000");
  //delay(1000);
  //clockOperator.runCommand("r+6+0+0+0");
  //delay(1000);
  //delay(1000);
  //clockOperator.runCommand("r+0+3+0+0");
  //clockOperator.runCommand("r+0-2+0+0");
  //clockOperator.runCommand("r+0+1+0+0");
  //clockOperator.runCommand("r+0-5+0+0");
  //clockOperator.runCommand("r+0+3+0+0");
  //clockOperator.runCommand("r+0+6+0+0");
  //clockOperator.runCommand("r+0+4+0+0");
  //clockOperator.runCommand("r+0+6+0+0");
  //clockOperator.runCommand("r+0-1+0+0");
  //clockOperator.runCommand("r+0+5+0+0");
  //clockOperator.runCommand("r+0+4+0+0");


  
  
  //clockOperator.runCommand("r+1+3+3+1");
  //clockOperator.runCommand("r+4+6+6+6");
  //clockOperator.runCommand("r+2+5+5+2");
  //clockOperator.runCommand("r+0+6+6+0");
  //clockOperator.runCommand("r+3+4+4+3");
  //clockOperator.runCommand("r+1+0+0+1");
  //clockOperator.runCommand("r+5+3+3+5");
  //clockOperator.runCommand("r+2+6+6+2");
  //clockOperator.runCommand("r+4+1+1+4");
  //clockOperator.runCommand("r+3+0+0+3");
  //clockOperator.runCommand("r+6+2+2+6");
  //clockOperator.runCommand("r+1+5+5+1");
  //clockOperator.runCommand("r+0+4+4+0");
  //clockOperator.runCommand("r+2+3+3+2");
  //clockOperator.runCommand("r+6+1+1+6");
  //clockOperator.runCommand("r+4+0+0+4");
  //clockOperator.runCommand("r+5+2+2+5");
  //clockOperator.runCommand("r+3+6+6+3");
  //clockOperator.runCommand("r+0+1+1+0");
  //clockOperator.runCommand("r+2+4+4+2");

  delay(1000000);
 
  //clockOperator.runCommand("p01010000");
  //clockOperator.runCommand("p00000000");
  //long x = millis();
  //clockOperator.runCommand("r+5+0+0+0");
  //delay(1000);
  //clockOperator.runCommand("r-5+0+0+0");
  //clockOperator.runCommand("r-2+0+0+0");
  //clockOperator.runCommand("r+4+0+0+0");
  //clockOperator.runCommand("r-5+0+0+0");
  //clockOperator.runCommand("r+2+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+5+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-1+0+0+0");
  //clockOperator.runCommand("r+4+0+0+0");
  //clockOperator.runCommand("r-2+0+0+0");
  //clockOperator.runCommand("r+1+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //clockOperator.runCommand("r-3+0+0+0");
  //clockOperator.runCommand("r+3+0+0+0");
  //Serial.println(millis()-x);
  //unsigned long x = millis();
  //clockOperator.clock.rotate(new float[4]{0,0,0,0});
  //delay(10);
  //clockOperator.clock.rotate(new float[4]{108,0,0,0});
  //Serial.println(millis()-x);
  
  //clockOperator.clock.clockSteppers.moveTo(new long[4]{200,200,200,200});
  //clockOperator.clock.clockSteppers.moveTo(new long[4]{-200,-200,-200,-200});
  //Serial.println(millis()-x);
  //clockOperator.runCommand("p11000000");
  //clockOperator.runCommand("r+0+0+6+0");
  //Serial.println(millis()-x);
  //Serial.println(millis()-x);

  //clockOperator.clock.test();
  //disablePins();
  //delay(5000);
}  