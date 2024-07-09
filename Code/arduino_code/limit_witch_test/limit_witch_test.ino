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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), powerEnabler, CHANGE);
  powerEnabler();
  
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
}
