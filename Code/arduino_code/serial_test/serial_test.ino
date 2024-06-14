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

AccelStepper stepper(1,2,5);
MultiStepper steppers;



#define LIMIT_PIN 18
#define MOTOR_ENABLE_PIN 19

void powerEnabler(){
  int mode;
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  do{

    mode = digitalRead(LIMIT_PIN);

  }while(mode==1);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}


void setup()
{  
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();


  Serial.begin(115200);

  stepper.setMaxSpeed(3900);
  stepper.setAcceleration(100000);
  stepper.setCurrentPosition(0);
  stepper.setSpeed(3900);

  steppers.addStepper(stepper);
}

void loop()
{  
  Serial.println("Moving to 3200");
  long positions[] = {1600};
  steppers.moveTo(positions);
  while(steppers.run());

  delay(5000);
}
