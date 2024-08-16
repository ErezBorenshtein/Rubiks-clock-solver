void setup(){
  Serial.begin(9600);
  pinMode(8, OUTPUT);
}

void loop(){
  digitalWrite(8, HIGH);
  delay(100000);
}