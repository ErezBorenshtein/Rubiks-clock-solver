void setup() {
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
}

