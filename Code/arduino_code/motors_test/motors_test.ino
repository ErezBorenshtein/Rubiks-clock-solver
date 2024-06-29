const long turn_steps = 400 * 2;
const int step_dir[4] = {49, 7, 11, 43};
const int step_pul[4] = {48, 8, 12, 42};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(step_dir[i], OUTPUT);
    pinMode(step_pul[i], OUTPUT);
  }
}


void move_motor(long spd, long steps1, long steps2, int motor0, int motor1, int motor2, int motor3) {
  bool hl1 = true;
  bool hl2 = true;
  if (steps1 > 0) hl1 = false;
  if (steps2 > 0) hl2 = false;
  if (steps1) {
    //Serial.println("hl1: " +(String)hl1);
    if (motor0) digitalWrite(step_dir[0], hl1);
    if (motor1) digitalWrite(step_dir[1], hl1);
    if (motor2) digitalWrite(step_dir[2], hl1);
    if (motor3) digitalWrite(step_dir[3], hl1);
  }
  if (steps2) {
    if (!motor0) digitalWrite(step_dir[0], hl2);
    if (!motor1) digitalWrite(step_dir[1], hl2);
    if (!motor2) digitalWrite(step_dir[2], hl2);
    if (!motor3) digitalWrite(step_dir[3], hl2);
  }
  //long steps1 = abs(steps1) * turn_steps / 360;
  //long steps2 = abs(steps2) * turn_steps / 360;

  steps1 = abs(steps1)*2;
  steps2 = abs(steps2)*2;
  Serial.println("steps1: " + (String)steps1);

  long avg_time = 1000000 * 60 / turn_steps / spd;
  long max_time = 375;
  long slope = 10;
  long accel = min(max(steps1, steps2), max(0, (max_time - avg_time) / slope));
  bool motor_hl = false;
  for (int i = 0; i < max(steps1, steps2); i++) {
    motor_hl = !motor_hl;
    if (i < steps1) {
      if (motor0) digitalWrite(step_pul[0], motor_hl);
      if (motor1) digitalWrite(step_pul[1], motor_hl);
      if (motor2) digitalWrite(step_pul[2], motor_hl);
      if (motor3) digitalWrite(step_pul[3], motor_hl);
    }
    if (i < steps2) {
      if (!motor0) digitalWrite(step_pul[0], motor_hl);
      if (!motor1) digitalWrite(step_pul[1], motor_hl);
      if (!motor2) digitalWrite(step_pul[2], motor_hl);
      if (!motor3) digitalWrite(step_pul[3], motor_hl);
    }
    long delay_time = avg_time;
    if (i < accel) delay_time = max_time - slope * i;
    Serial.println("delay_time: " + (String)delay_time);
    delayMicroseconds(delay_time);
  }
  
}


void loop() {

  delay(1000);
//
  //moveSteppers(new long[2]{108, 0});
//
  //delay(1000);
//
  //moveSteppers(new long[2]{0, 0});
  int x = millis();
  move_motor(700, 218, 0, 1, 0, 0, 0);
  Serial.println(millis() - x);

  delay(1000);
  

}
