#define p 13
const int MOTOR1_PIN1 = 4;  // Connect this to one terminal of Motor1
const int MOTOR1_PIN2 = 5;  // Connect this to the other terminal of Motor1
const int MOTOR2_PIN1 = 6;  // Connect this to one terminal of Motor2
const int MOTOR2_PIN2 = 7;  // Connect this to the other terminal of Motor2
void setup() {
  pinMode(p, OUTPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(p, HIGH);
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
  delay(500);
  digitalWrite(p, LOW);
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
  delay(500);
  // put your main code here, to run repeatedly:

}
