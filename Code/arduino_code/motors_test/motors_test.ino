void setup(){
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}


void loop(){
  digitalWrite(2, HIGH);
  int x =millis();
  Serial.println("0");
  delay(1000);
  Serial.println(millis() -x );
  digitalWrite(2, LOW);
  
  delay(100000);
}