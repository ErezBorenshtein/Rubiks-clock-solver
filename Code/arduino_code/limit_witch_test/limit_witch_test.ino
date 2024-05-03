#include <Servo.h>

#define LIMIT_PIN 40
#define MOTOR_ENABLE_PIN 41
int lastEnableMode = 0;
void setup() {

  //start serial connection
  /*Serial.begin(9600);
  delay (1000);
  //configure pin 2 as an input and enable the internal pull-up resistor

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  
  Serial.println("Setup...");
  delay (1000);*/
  pinMode(13, OUTPUT);

}

void waitUntilMotorEnabeld(){
  while(lastEnableMode==0){
    checkMotorEnable();
    delay(10);
  }
}

void checkMotorEnable(){
  int sensorVal = digitalRead(LIMIT_PIN);
  if (lastEnableMode != sensorVal)
  {
    Serial.println("new mode: " + String(sensorVal));
    lastEnableMode = sensorVal;
    delay (100);
  }

  if (sensorVal == HIGH) {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

  }

  else {

    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    //resetFunc();  //call reset
  }
  
}

void loop() {
  //checkMotorEnable();
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(1000);

  
}