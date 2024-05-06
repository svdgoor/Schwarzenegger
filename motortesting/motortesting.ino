const int p = 13;
const int MOTOR1_PIN1 = 4;  // Connect this to one terminal of Motor1
const int MOTOR1_PIN2 = 5;  // Connect this to the other terminal of Motor1
const int MOTOR2_PIN1 = 6;  // Connect this to one terminal of Motor2
const int MOTOR2_PIN2 = 7;  // Connect this to the other terminal of Motor2
const int msDelayPer360Turn = 500; // TODO: Test this

void setup() {
  pinMode(p, OUTPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
}

void loop() {
  digitalWrite(p, HIGH);
  turnL();
  delay(msDelayPer360Turn);
  digitalWrite(p, LOW);
  moveStop();
  delay(5000);
}

void moveStop(){
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void moveForward(){
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnL(){
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnR(){
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}
